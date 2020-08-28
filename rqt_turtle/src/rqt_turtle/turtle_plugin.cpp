#include "rqt_turtle/turtle_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QInputDialog>
#include <QListWidgetItem>

#include <std_srvs/Empty.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/TeleportAbsolute.h>
#include <ros/service.h>


#include "ui_turtle_plugin.h"
#include "ui_TopicWidget.h"

namespace rqt_turtle {

    TurtlePlugin::TurtlePlugin()
        : rqt_gui_cpp::Plugin()
        , m_pUi(new Ui::TurtlePluginWidget)
        , m_pWidget(0)
        , m_pTopicWidget(new Ui::TopicWidget)
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
        m_pWidget = new QWidget();
        // extend the widget with all attributes and children from UI file
        //m_pUi.setupUi(widget_);
        ROS_INFO("INIT");
        m_pUi->setupUi(m_pWidget);
        // add widget to the user interface
        context.addWidget(m_pWidget);

        connect(m_pUi->btnReset, SIGNAL(clicked()), this, SLOT(on_btnReset_clicked()));
        connect(m_pUi->btnSpawn, SIGNAL(clicked()), this, SLOT(on_btnSpawn_clicked()));
        connect(m_pUi->btnDraw, SIGNAL(clicked()), this, SLOT(on_btnDraw_clicked()));
        connect(m_pUi->btnTeleportAbs, SIGNAL(clicked()), this, SLOT(on_btnTeleportAbs_clicked()));

        connect(m_pUi->lvTurtles, SIGNAL(itemSelectionChanged()), 
                this, SLOT(on_selection_changed()));

        
        m_pTopicDialog = new QDialog(0,0);
        m_pUiTopicWidget->setupUi(m_pTopicDialog);

        m_pTopicDialog->show();
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
        m_pUi->lvTurtles->clear();
    }

    void TurtlePlugin::on_btnSpawn_clicked()
    {
        bool ok;
        QString qstrTurtleName = QInputDialog::getText(m_pWidget, tr("Spawn Turtle"),
                                            tr("Name:"), QLineEdit::Normal,
                                            tr("MyTurtle"), &ok);
        if (!ok || qstrTurtleName.isEmpty())
        {
            return;
        }
        auto existingTurtles = m_pUi->lvTurtles->findItems(qstrTurtleName, Qt::MatchExactly);
        const char * strTurtleName = qstrTurtleName.toStdString().c_str();
        if (existingTurtles.size() > 0)
        {
            ROS_INFO("Turtle with the name \"%s\" already exists.", strTurtleName);
            return;
        }

        ROS_INFO("Spawn turtle: %s.", strTurtleName);
        turtlesim::Spawn spawn;
        spawn.request.x = random() % 12;
        spawn.request.y = random() % 12;
        spawn.request.theta = random() % 12;
        spawn.request.name = qstrTurtleName.toStdString();
        ros::service::call<turtlesim::Spawn>("spawn", spawn);
        m_pUi->lvTurtles->addItems(QStringList(qstrTurtleName));
    }

    void TurtlePlugin::on_btnDraw_clicked()
    {
        auto list = m_pUi->lvTurtles->selectedItems();
        ROS_INFO("%d", list.size());
        if (list.size() > 0)
        {
            QString turtleName = list[0]->text();
            ROS_INFO(turtleName.toStdString().c_str());

        }
        
    }

    void TurtlePlugin::on_btnTeleportAbs_clicked()
    {
        std::string strServiceName = "/" + m_strSelectedTurtle + "/teleport_absolute";
        turtlesim::TeleportAbsolute sTeleportAbsolute;
        auto request = sTeleportAbsolute.request;
        request.x = 1.0;
        request.y = 1.0;
        ROS_INFO("Teleport %s to x: %d, y: %d", m_strSelectedTurtle.c_str(), request.x, request.y);
        ros::service::call<turtlesim::TeleportAbsolute>(strServiceName, sTeleportAbsolute);
        auto response = sTeleportAbsolute.response;
        
    }


    void TurtlePlugin::on_selection_changed()
    {
        auto current = m_pUi->lvTurtles->currentItem(); // TODO use member list if multiple turtles are selected
        m_strSelectedTurtle = current->text().toStdString();
        ROS_INFO("Turtle %s selected", m_strSelectedTurtle.c_str());
    }


} // namespace

// Deprecated
// See: http://wiki.ros.org/pluginlib#pluginlib.2Fpluginlib_groovy.Simplified_Export_Macro
//PLUGINLIB_DECLARE_CLASS(rqt_turtle, TurtlePlugin, rqt_turtle::TurtlePlugin, rqt_gui_cpp::Plugin)
PLUGINLIB_EXPORT_CLASS(rqt_turtle::TurtlePlugin, rqt_gui_cpp::Plugin)