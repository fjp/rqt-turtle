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
#include <turtlesim/SetPen.h>
#include <ros/service.h>
#include <ros/param.h>
#include <ros/topic.h>


#include "ui_turtle_plugin.h"

#include "rqt_turtle/service_caller.h"
#include "rqt_turtle/draw.h"


namespace rqt_turtle {

    TurtlePlugin::TurtlePlugin()
        : rqt_gui_cpp::Plugin()
        , ui_(new Ui::TurtlePluginWidget)
        , widget_(0)
    {
        // Constructor is called first before initPlugin function, needless to say.

        // give QObjects reasonable names
        setObjectName("TurtlePlugin");
    }

    void TurtlePlugin::initPlugin(qt_gui_cpp::PluginContext& context)
    {
        ROS_INFO("Init rqt_turtle plugin");
        // access standalone command line arguments
        QStringList argv = context.argv();
        // create QWidget
        widget_ = new QWidget();
        // extend the widget with all attributes and children from UI file
        ui_->setupUi(widget_);
        // add widget to the user interface
        context.addWidget(widget_);

        connect(ui_->btnReset, SIGNAL(clicked()), this, SLOT(on_btnReset_clicked()));
        connect(ui_->btnSpawn, SIGNAL(clicked()), this, SLOT(on_btnSpawn_clicked()));
        connect(ui_->btnKill, SIGNAL(clicked()), this, SLOT(on_btnKill_clicked()));
        connect(ui_->btnColor, SIGNAL(clicked()), this, SLOT(on_btnColor_clicked()));
        connect(ui_->btnDraw, SIGNAL(clicked()), this, SLOT(on_btnDraw_clicked()));
        connect(ui_->btnTeleportAbs, SIGNAL(clicked()), this, SLOT(on_btnTeleportAbs_clicked()));
        connect(ui_->btnTeleportRel, SIGNAL(clicked()), this, SLOT(on_btnTeleportRel_clicked()));
        connect(ui_->btnTogglePen, SIGNAL(clicked()), this, SLOT(on_btnTogglePen_clicked()));

        connect(ui_->treeTurtles, SIGNAL(itemSelectionChanged()), 
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

                // Create new turtle in turtle map
                // Note: assume that the pen is toggled on
                QSharedPointer<Turtle> turtle = QSharedPointer<Turtle>(new Turtle(turtle_name, *pose));
                turtles_[QString::fromStdString(turtle_name)] = turtle;
            }
        }

        // Insert the turtles into the QTreeWidget
        for (auto turtle : turtles_)
        {
            ui_->treeTurtles->insertTopLevelItem(0, turtle->toTreeItem(ui_->treeTurtles));
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
        ui_->treeTurtles->clear();
        turtles_.clear();

        updateTurtleTree();
    }

    void TurtlePlugin::on_btnSpawn_clicked()
    {
        ROS_DEBUG("Spawn clicked");
        std::string service_name = "/spawn";
        service_caller_dialog_ = QSharedPointer<ServiceCaller>(new ServiceCaller(widget_, service_name));

        QString qstr_turtle_name;
        QVariantMap request;
        bool ok = service_caller_dialog_->exec() == QDialog::Accepted;
        if (ok)
        {
            ROS_INFO("accepted");
            request = service_caller_dialog_->getRequest();
            qstr_turtle_name = request["name"].toString();
        }

        if (!ok || qstr_turtle_name.isEmpty())
        {
            ROS_INFO("Closed Service Caller Dialog or Turtle Name empty.");
            return;
        }
        auto existing_turtles = ui_->treeTurtles->findItems(qstr_turtle_name, Qt::MatchExactly);
        const char * str_turtle_name = qstr_turtle_name.toStdString().c_str();
        if (existing_turtles.size() > 0)
        {
            ROS_INFO("Turtle with the name \"%s\" already exists.", str_turtle_name);
            return;
        }

        ROS_INFO("Spawn turtle: %s.", str_turtle_name);
        turtlesim::Spawn spawn;
        spawn.request.x = request["x"].toString().toFloat();
        spawn.request.y = request["y"].toString().toFloat();
        spawn.request.theta = request["theta"].toFloat();
        spawn.request.name = request["name"].toString().toStdString().c_str();
        ros::service::call<turtlesim::Spawn>("spawn", spawn);

        QTreeWidgetItem *item = new QTreeWidgetItem(ui_->treeTurtles);
        item->setText(0, qstr_turtle_name); // Column 0 name
        item->setText(1, request["x"].toString()); // Column 1 x
        item->setText(2, request["y"].toString()); // Column 2 y
        item->setText(3, request["theta"].toString()); // Column 3 theta
        item->setText(4, QString("on")); // Column 4 pen on/off (pen is always on by default)
        ui_->treeTurtles->insertTopLevelItem(0, item);

        // Create new turtle in turtle map
        // Note: assume that the pen is toggled on
        QSharedPointer<Turtle> turtle = QSharedPointer<Turtle>(new Turtle(qstr_turtle_name.toStdString(), 
                                                        spawn.request.x, spawn.request.y, spawn.request.theta));
        turtles_[qstr_turtle_name] = turtle;
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
        std_srvs::Empty reset;
        ros::service::call("/clear", reset);
    }

    void TurtlePlugin::on_btnDraw_clicked()
    {
        draw_dialog_ = QSharedPointer<Draw>(new Draw(widget_, turtles_));

        draw_dialog_->open();
        //bool ok = draw_dialog_->exec() == QDialog::Accepted;
        bool ok = true;

        // Remove ?
        /*
        auto list = m_pUi->treeTurtles->selectedItems();
        ROS_INFO("%d", list.size());
        if (list.size() > 0)
        {
            QString turtleName = list[0]->text(0);
            std::vector<std::vector<cv::Point> > contours = draw_dialog_->contours();
            ROS_INFO("Found %d contours", (int)contours.size());
            DrawImage(contours);
        }
        */
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
            QList<QTreeWidgetItem*> list = ui_->treeTurtles->findItems(selected_turtle, Qt::MatchExactly);
            for (auto item : list)
            {
                // Remove turtle from turtles_ QMap
                turtles_.remove(item->text(0));
                // Remove the turtle from the QTreeWidget treeTurle
                delete item;
            }
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
        service_caller_dialog_ = QSharedPointer<ServiceCaller>(new ServiceCaller(widget_, strServiceName));

        QString qstrTurtleName;
        QVariantMap request;
        bool ok = service_caller_dialog_->exec() == QDialog::Accepted;
        if (ok)
        {
            ROS_DEBUG("accepted");
            request = service_caller_dialog_->getRequest();
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
                turtle->setPen(false);
            }
            else
            {
                turtle->setPen(true);
            }
            
            ROS_INFO("Set pen for turtle %s: %s", turtle->name_.c_str(), turtle->pen_.off ? "Off" : "On");
        }
    }

    void TurtlePlugin::setPen(QSharedPointer<Turtle> turtle)
    {
        turtlesim::SetPen set_pen;
        set_pen.request.r = turtle->pen_.r;
        set_pen.request.g = turtle->pen_.g;
        set_pen.request.b = turtle->pen_.b;
        set_pen.request.width = turtle->pen_.width;
        set_pen.request.off = turtle->pen_.off;

        std::string service_name = "/" + turtle->name_ + "/set_pen";
        ros::service::call<turtlesim::SetPen>(service_name, set_pen);
    }


    void TurtlePlugin::on_selection_changed()
    {
        //auto current = m_pUi->treeTurtles->currentItem(); // TODO use member list if multiple turtles are selected
        // Get list of selected turtles
        auto selected_items = ui_->treeTurtles->selectedItems();
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