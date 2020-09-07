#include "rqt_turtle/turtle_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QColorDialog>
#include <QVariantMap>
#include <QTreeWidgetItem>
#include <QMessageBox>

#include <std_srvs/Empty.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/TeleportRelative.h>
#include <turtlesim/SetPen.h>
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
        , ac_("turtle_shape", true)
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
                turtles_[QString::fromStdString(turtle_name)] = turtle;
            }
        }

        // Insert the turtles into the QTreeWidget
        for (auto turtle : turtles_)
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

        // Clear the QTreeWidget
        m_pUi->treeTurtles->clear();
        turtles_.clear();

        updateTurtleTree();
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
        item->setText(4, QString("on")); // Column 4 pen on/off (pen is always on by default)
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
        ROS_INFO("Waiting for action server to start.");

        if (!ac_.isServerConnected())
        {
            QMessageBox msgBox;
            msgBox.setText("Waiting for action server to start");
            msgBox.setInformativeText("Please run 'rosrun turtle_actionlib shape_server' and press Ok or cancel \
                                       to avoid blocking rqt_turtle gui while wating for shape_server.");
            msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
            int ret = msgBox.exec();
            if (ret == QMessageBox::Cancel)
            {
                return;
            }
        }
        
        // wait for the action server to start
        ac_.waitForServer(); //will wait for infinite time

        ROS_INFO("Action server started, sending goal.");
        // send a goal to the action
        turtle_actionlib::ShapeGoal shape;
        shape.edges = 3;
        shape.radius = 2.0;
        ac_.sendGoal(shape);

        //wait for the action to return
        bool finished_before_timeout = ac_.waitForResult(ros::Duration(30.0));


        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac_.getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else
            ROS_INFO("Action did not finish before the time out.");

        //exit
        return; // TODO fix

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
        if (selected_turtles_.empty())
        {
            ROS_INFO("No turtles selected");
            return;
        }

        for (auto selected_turtle : selected_turtles_)
        {
            ROS_INFO("Killing turtle %s", str(selected_turtle).c_str());
            turtlesim::Kill kill;
            kill.request.name = str(selected_turtle);
            ros::service::call<turtlesim::Kill>("kill", kill);

            // remove turtle from tree widget
            QList<QTreeWidgetItem*> list = m_pUi->treeTurtles->findItems(selected_turtle, Qt::MatchExactly);
            for (auto item : list)
            {
                // Remove turtle from turtles_ QMap
                turtles_.remove(item->text(0));
                // Remove the turtle from the QTreeWidget treeTurle
                delete item;
            }
            
            //m_pUi->treeTurtles->addItems(QStringList(qstrTurtleName));
        }
    }

    void TurtlePlugin::on_btnTeleportAbs_clicked()
    {
        if (selected_turtles_.empty())
        {
            ROS_INFO("No turtles selected");
            return;
        }

        // Create ServiceCaller for first selected turtle
        auto it_selected_turtle = selected_turtles_.begin();
        std::string turtle_name = str(*it_selected_turtle);
        std::string strServiceName = "/" + turtle_name + "/teleport_absolute";
        QVariantMap request = teleport(strServiceName);

        if (request.empty())
        {
            return;
        }

        for (auto selected_turtle : selected_turtles_)
        {
            turtle_name = str(selected_turtle);
            strServiceName = "/" + turtle_name + "/teleport_absolute";

            turtlesim::TeleportAbsolute sTeleportAbsolute;
            sTeleportAbsolute.request.x = request["x"].toString().toFloat();
            sTeleportAbsolute.request.y = request["y"].toString().toFloat();
            sTeleportAbsolute.request.theta = request["theta"].toString().toFloat();
            ROS_INFO("Teleport %s to x: %f, y: %f, theta: %f",
                        turtle_name.c_str(),
                        sTeleportAbsolute.request.x,
                        sTeleportAbsolute.request.y,
                        sTeleportAbsolute.request.theta);
            ros::service::call<turtlesim::TeleportAbsolute>(strServiceName, sTeleportAbsolute);
            auto response = sTeleportAbsolute.response;
        }
    }

    void TurtlePlugin::on_btnTeleportRel_clicked()
    {
        if (selected_turtles_.empty())
        {
            ROS_INFO("No turtles selected");
            return;
        }

        // Create ServiceCaller for first selected turtle
        auto it_selected_turtle = selected_turtles_.begin();
        std::string turtle_name = str(*it_selected_turtle);
        std::string strServiceName = "/" + turtle_name + "/teleport_relative";
        QVariantMap request = teleport(strServiceName);

        if (request.empty())
        {
            return;
        }

        for (auto selected_turtle : selected_turtles_)
        {
            turtle_name = str(selected_turtle);
            strServiceName = "/" + turtle_name + "/teleport_relative";

            turtlesim::TeleportRelative sTeleportRelative;
            sTeleportRelative.request.linear = request["linear"].toString().toFloat();
            sTeleportRelative.request.angular = request["angular"].toString().toFloat();
            ROS_INFO("Teleport %s to linear: %f, angular: %f",
                        turtle_name.c_str(),
                        sTeleportRelative.request.linear,
                        sTeleportRelative.request.angular);
            ros::service::call<turtlesim::TeleportRelative>(strServiceName, sTeleportRelative);
            auto response = sTeleportRelative.response;
        }
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


    void TurtlePlugin::on_btnTogglePen_clicked()
    {
        
        if (selected_turtles_.empty())
        {
            ROS_INFO("No turtles selected");
            return;
        }

        for (auto selected_turtle : selected_turtles_)
        {
            QSharedPointer<Turtle> turtle = turtles_[selected_turtle];
            if (turtle->pen_.off)
            {
                turtle->pen_.off = false;
            }
            else
            {
                turtle->pen_.off = true;
            }
            std::string service_name = "/" + turtle->name_ + "/set_pen";
            turtlesim::SetPen set_pen;
            set_pen.request.r = turtle->pen_.r;
            set_pen.request.g = turtle->pen_.g;
            set_pen.request.b = turtle->pen_.b;
            set_pen.request.width = turtle->pen_.width;
            set_pen.request.off = turtle->pen_.off;
            ros::service::call<turtlesim::SetPen>(service_name, set_pen);
            ROS_INFO("Set pen for turtle %s: %s", str(selected_turtle).c_str(), turtle->pen_.off ? "Off" : "On");
        }
    }


    void TurtlePlugin::on_selection_changed()
    {
        //auto current = m_pUi->treeTurtles->currentItem(); // TODO use member list if multiple turtles are selected
        // Get list of selected turtles
        auto selected_items = m_pUi->treeTurtles->selectedItems();
        selected_turtles_.clear();
        if (selected_items.empty())
        {
            return;
        }
        //m_strSelectedTurtle = current->text(0).toStdString();
        QString turtle_name;
        std::stringstream ss;
        for (auto item : selected_items)
        {
            turtle_name = item->text(0);
            selected_turtles_.push_back(turtle_name);
            ss << str(turtle_name) << " ";
            
        }
        ROS_INFO("Selected %s", ss.str().c_str());
        
    }


} // namespace

// Deprecated
// See: http://wiki.ros.org/pluginlib#pluginlib.2Fpluginlib_groovy.Simplified_Export_Macro
//PLUGINLIB_DECLARE_CLASS(rqt_turtle, TurtlePlugin, rqt_turtle::TurtlePlugin, rqt_gui_cpp::Plugin)
PLUGINLIB_EXPORT_CLASS(rqt_turtle::TurtlePlugin, rqt_gui_cpp::Plugin)