#include "rqt_turtle/turtle_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QColorDialog>
#include <QVariantMap>
#include <QTreeWidgetItem>

#include <std_srvs/Empty.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/TeleportRelative.h>
#include <ros/service.h>
#include <ros/param.h>
#include <ros/topic.h>


#include "ui_turtle_plugin.h"

#include "rqt_turtle/service_caller.h"

namespace rqt_turtle {

    TurtlePlugin::TurtlePlugin()
        : rqt_gui_cpp::Plugin()
        , m_pUi(new Ui::TurtlePluginWidget)
        , m_pWidget(0)
    {
        // Constructor is called first before initPlugin function, needless to say.

        // give QObjects reasonable names
        setObjectName("TurtlePlugin");

        //m_pServiceCaller = new ServiceCaller(m_pWidget);
    }

    void TurtlePlugin::initPlugin(qt_gui_cpp::PluginContext& context)
    {
        // access standalone command line arguments
        QStringList argv = context.argv();
        // create QWidget
        m_pWidget = new QWidget();
        // extend the widget with all attributes and children from UI file
        //m_pUi.setupUi(widget_);
        ROS_INFO("Init rqt_turtle plugin");
        m_pUi->setupUi(m_pWidget);
        // add widget to the user interface
        context.addWidget(m_pWidget);

        connect(m_pUi->btnReset, SIGNAL(clicked()), this, SLOT(on_btnReset_clicked()));
        connect(m_pUi->btnSpawn, SIGNAL(clicked()), this, SLOT(on_btnSpawn_clicked()));
        connect(m_pUi->btnKill, SIGNAL(clicked()), this, SLOT(on_btnKill_clicked()));
        connect(m_pUi->btnColor, SIGNAL(clicked()), this, SLOT(on_btnColor_clicked()));
        connect(m_pUi->btnDraw, SIGNAL(clicked()), this, SLOT(on_btnDraw_clicked()));
        connect(m_pUi->btnTeleportAbs, SIGNAL(clicked()), this, SLOT(on_btnTeleportAbs_clicked()));
        connect(m_pUi->btnTeleportRel, SIGNAL(clicked()), this, SLOT(on_btnTeleportRel_clicked()));
        connect(m_pUi->btnTogglePen, SIGNAL(clicked()), this, SLOT(on_btnTogglePen_clicked()));

        connect(m_pUi->treeTurtles, SIGNAL(itemSelectionChanged()), 
                this, SLOT(on_selection_changed()));

        

        updateTurtleTree();
    }

    void TurtlePlugin::updateTurtleTree()
    {
        // https://stackoverflow.com/questions/26785675/ros-get-current-available-topic-in-code-not-command
        // Use XML-RPC ROS Master API to get the topic names
        // Then filter for topics containing pose (which belongs to a turtle)
        ros::master::V_TopicInfo master_topics;
        ros::master::getTopics(master_topics);

        ros::NodeHandle nh = getNodeHandle();
        for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++)
        {
            const ros::master::TopicInfo& info = *it;
            ROS_INFO_STREAM("topic_" << it - master_topics.begin() << ": " << info.name);
            QString topic_name = QString::fromStdString(info.name);
            if (topic_name.contains(QString("/pose")))
            {
                QStringList topic_name_parts = topic_name.split(QRegExp("\\/"), QString::SkipEmptyParts);
                std::string turtle_name = topic_name_parts[0].toStdString();
                ROS_INFO("topic_name_part 0: %s", turtle_name.c_str());
                
                // Wait for a single pose message to arrive on the turtlesim::Pose topic
                turtlesim::PoseConstPtr pose = ros::topic::waitForMessage<turtlesim::Pose>(topic_name.toStdString());
                ROS_INFO("Pose received: x: %f, y: %f, theta: %f", pose->x, pose->y, pose->theta);

                // Create new turtle in turtle vector
                // Note: assume that the pen is toggled on
                QSharedPointer<Turtle> turtle = QSharedPointer<Turtle>(new Turtle(turtle_name, *pose));
                m_vTurtles.push_back(turtle);
            }
        }

        // Insert the turtles into the QTreeWidget
        for (auto turtle : m_vTurtles)
        {
            m_pUi->treeTurtles->insertTopLevelItem(0, turtle->toTreeItem(m_pUi->treeTurtles));
        }
    }

    void TurtlePlugin::shutdownPlugin()
    {
        // TODO unregister all publishers here
    }

    void TurtlePlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
    {
        // TODO save intrinsic configuration, usually using:
        // instance_settings.setValue(k, v)
    }

    void TurtlePlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
    {
        // TODO restore intrinsic configuration, usually using:
        // v = instance_settings.value(k)
    }

    /*bool hasConfiguration() const
    {
        return true;
    }

    void triggerConfiguration()
    {
        // Usually used to open a dialog to offer the user a set of configuration
    }*/

    void TurtlePlugin::on_btnReset_clicked()
    {
        ROS_INFO("Reset turtlesim.");
        std_srvs::Empty empty;
        ros::service::call<std_srvs::Empty>("reset", empty);

        // Clear the listViewWidget
        m_pUi->treeTurtles->clear();
    }

    void TurtlePlugin::on_btnSpawn_clicked()
    {
        ROS_DEBUG("Spawn clicked");
        std::string service_name = "/spawn";
        m_pServiceCaller = new ServiceCaller(m_pWidget, service_name);

        QString qstrTurtleName;
        QVariantMap request;
        bool ok = m_pServiceCaller->exec() == QDialog::Accepted;
        if (ok)
        {
            ROS_INFO("accepted");
            request = m_pServiceCaller->getRequest();
            qstrTurtleName = request["name"].toString();
        }

        if (!ok || qstrTurtleName.isEmpty())
        {
            ROS_INFO("Closed Service Caller Dialog or Turtle Name empty.");
            return;
        }
        auto existingTurtles = m_pUi->treeTurtles->findItems(qstrTurtleName, Qt::MatchExactly);
        const char * strTurtleName = qstrTurtleName.toStdString().c_str();
        if (existingTurtles.size() > 0)
        {
            ROS_INFO("Turtle with the name \"%s\" already exists.", strTurtleName);
            return;
        }

        ROS_INFO("Spawn turtle: %s.", strTurtleName);
        turtlesim::Spawn spawn;
        spawn.request.x = request["x"].toString().toFloat();
        spawn.request.y = request["y"].toString().toFloat();
        spawn.request.theta = request["theta"].toFloat();
        spawn.request.name = request["name"].toString().toStdString().c_str();
        ros::service::call<turtlesim::Spawn>("spawn", spawn);
        //m_pUi->tableTurtles->addItems(QStringList(qstrTurtleName));

        QTreeWidgetItem *item = new QTreeWidgetItem(m_pUi->treeTurtles);
        item->setText(0, qstrTurtleName); // Column 0 name
        item->setText(1, request["x"].toString()); // Column 1 x
        item->setText(2, request["y"].toString()); // Column 2 y
        item->setText(3, request["theta"].toString()); // Column 3 theta
        item->setText(4, request["theta"].toString()); // Column 4 pen on/off
        // TODO pen on/off
        m_pUi->treeTurtles->insertTopLevelItem(0, item);
    }

    void TurtlePlugin::on_btnColor_clicked()
    {
        int r, g, b;
        ros::param::get("/turtlesim/background_b", b);
        ros::param::get("/turtlesim/background_g", g);
        ros::param::get("/turtlesim/background_r", r);
        ROS_INFO("Current color (r,g,b) = (%i,%i,%i)", r, g, b);
        QColor qColor = QColorDialog::getColor();
        ROS_INFO("Color %s", qColor.name().toStdString().c_str());
        qColor.getRgb(&r, &g, &b);
        ROS_INFO("Setting color to (r,g,b) = (%i,%i,%i)", r, g, b);
        ros::param::set("/turtlesim/background_b", b);
        ros::param::set("/turtlesim/background_g", g);
        ros::param::set("/turtlesim/background_r", r);

        // Note: this will not set the color (only after reset is called)
    }

    void TurtlePlugin::on_btnDraw_clicked()
    {
        auto list = m_pUi->treeTurtles->selectedItems();
        ROS_INFO("%d", list.size());
        if (list.size() > 0)
        {
            QString turtleName = list[0]->text(0);
            ROS_INFO(turtleName.toStdString().c_str());

        }
    }

    void TurtlePlugin::on_btnKill_clicked()
    {
        ROS_INFO("Killing turtle %s", m_strSelectedTurtle.c_str());
        turtlesim::Kill kill;
        kill.request.name = m_strSelectedTurtle;
        ros::service::call<turtlesim::Kill>("kill", kill);

        // remove turtle from tree widget
        QList<QTreeWidgetItem*> list = m_pUi->treeTurtles->findItems(QString::fromStdString(m_strSelectedTurtle), Qt::MatchExactly);
        for (auto item : list)
        {
            delete item;
        }
        // TODO there seems to be a bug when the last turtle is deleted.
        //m_pUi->treeTurtles->addItems(QStringList(qstrTurtleName));
    }

    void TurtlePlugin::on_btnTeleportAbs_clicked()
    {
        std::string strServiceName = "/" + m_strSelectedTurtle + "/teleport_absolute";
        QVariantMap request = teleport(strServiceName);

        if (request.empty())
        {
            return;
        }

        turtlesim::TeleportAbsolute sTeleportAbsolute;
        sTeleportAbsolute.request.x = request["x"].toString().toFloat();
        sTeleportAbsolute.request.y = request["y"].toString().toFloat();
        sTeleportAbsolute.request.theta = request["theta"].toString().toFloat();
        ROS_INFO("Teleport %s to x: %f, y: %f, theta: %f",
                    m_strSelectedTurtle.c_str(),
                    sTeleportAbsolute.request.x,
                    sTeleportAbsolute.request.y,
                    sTeleportAbsolute.request.theta);
        ros::service::call<turtlesim::TeleportAbsolute>(strServiceName, sTeleportAbsolute);
        auto response = sTeleportAbsolute.response;
    }

    void TurtlePlugin::on_btnTeleportRel_clicked()
    {
        std::string strServiceName = "/" + m_strSelectedTurtle + "/teleport_relative";
        QVariantMap request = teleport(strServiceName);

        if (request.empty())
        {
            return;
        }

        turtlesim::TeleportRelative sTeleportRelative;
        sTeleportRelative.request.linear = request["linear"].toString().toFloat();
        sTeleportRelative.request.angular = request["angular"].toString().toFloat();
        ROS_INFO("Teleport %s to linear: %f, angular: %f",
                    m_strSelectedTurtle.c_str(),
                    sTeleportRelative.request.linear,
                    sTeleportRelative.request.angular);
        ros::service::call<turtlesim::TeleportRelative>(strServiceName, sTeleportRelative);
        auto response = sTeleportRelative.response;
    }


    QVariantMap TurtlePlugin::teleport(std::string strServiceName)
    {
        
        m_pServiceCaller = new ServiceCaller(m_pWidget, strServiceName);

        QString qstrTurtleName;
        QVariantMap request;
        bool ok = m_pServiceCaller->exec() == QDialog::Accepted;
        if (ok)
        {
            ROS_DEBUG("accepted");
            request = m_pServiceCaller->getRequest();
            qstrTurtleName = request["name"].toString();
        }
        else
        {
            ROS_DEBUG("ServiceCaller Dialog closed");
            return QVariantMap();
        }

        return request;
    }


    void TurtlePlugin::on_selection_changed()
    {
        auto current = m_pUi->treeTurtles->currentItem(); // TODO use member list if multiple turtles are selected
        m_strSelectedTurtle = current->text(0).toStdString();
        ROS_INFO("Turtle %s selected", m_strSelectedTurtle.c_str());
    }


} // namespace

// Deprecated
// See: http://wiki.ros.org/pluginlib#pluginlib.2Fpluginlib_groovy.Simplified_Export_Macro
//PLUGINLIB_DECLARE_CLASS(rqt_turtle, TurtlePlugin, rqt_turtle::TurtlePlugin, rqt_gui_cpp::Plugin)
PLUGINLIB_EXPORT_CLASS(rqt_turtle::TurtlePlugin, rqt_gui_cpp::Plugin)