#include "rqt_turtle/turtle_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

#include <std_srvs/Empty.h>
#include <turtlesim/Spawn.h>
#include <ros/service.h>

#include "ui_turtle_plugin.h"

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
        // access standalone command line arguments
        QStringList argv = context.argv();
        // create QWidget
        widget_ = new QWidget();
        // extend the widget with all attributes and children from UI file
        //ui_.setupUi(widget_);
        ROS_INFO("INIT");
        ui_->setupUi(widget_);
        // add widget to the user interface
        context.addWidget(widget_);

        connect(ui_->btnReset, SIGNAL(clicked()), this, SLOT(on_btnReset_clicked()));
        connect(ui_->btnSpawn, SIGNAL(clicked()), this, SLOT(on_btnSpawn_clicked()));
        connect(ui_->btnDraw, SIGNAL(clicked()), this, SLOT(on_btnDraw_clicked()));
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
    }

    void TurtlePlugin::on_btnSpawn_clicked()
    {
        ROS_INFO("Spawn turtle.");
        turtlesim::Spawn spawn;
        spawn.request.x = 5.0;
        spawn.request.y = 5.0;
        spawn.request.theta = 0.0;
        spawn.request.name = "new_turtle";
        ros::service::call<turtlesim::Spawn>("spawn", spawn);
        ui_->lvTurtles->addItems(QStringList("new_turtle"));
    }

    void TurtlePlugin::on_btnDraw_clicked()
    {
        auto list = ui_->lvTurtles->selectedItems();
        ROS_INFO("%d", list.size());
        if (list.size() > 0)
        {
            QString turtleName = list[0]->text();
            ROS_INFO(turtleName.toStdString().c_str());
        }
        
    }

} // namespace

// Deprecated
// See: http://wiki.ros.org/pluginlib#pluginlib.2Fpluginlib_groovy.Simplified_Export_Macro
//PLUGINLIB_DECLARE_CLASS(rqt_turtle, TurtlePlugin, rqt_turtle::TurtlePlugin, rqt_gui_cpp::Plugin)
PLUGINLIB_EXPORT_CLASS(rqt_turtle::TurtlePlugin, rqt_gui_cpp::Plugin)