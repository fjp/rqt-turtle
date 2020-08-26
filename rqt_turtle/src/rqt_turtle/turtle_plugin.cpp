#include "rqt_turtle/turtle_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

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
        ui_->setupUi(this)
        // add widget to the user interface
        context.addWidget(widget_);
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

} // namespace

PLUGINLIB_DECLARE_CLASS(rqt_turtle, TurtlePlugin, rqt_turtle::TurtlePlugin, rqt_gui_cpp::Plugin)