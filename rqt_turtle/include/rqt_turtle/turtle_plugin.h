#ifndef rqt_turtle__turtle_plugin_H
#define rqt_turtle__turtle_plugin_H

#include <rqt_gui_cpp/plugin.h>
#include <QWidget>


class QListWidgetItem;

namespace Ui {
    class TurtlePluginWidget;
}

namespace rqt_turtle {

    class ServiceCaller;


    class TurtlePlugin
        : public rqt_gui_cpp::Plugin
    {
        Q_OBJECT
    public:
        TurtlePlugin();
        virtual void initPlugin(qt_gui_cpp::PluginContext& context);
        virtual void shutdownPlugin();
        virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
        virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

        // Comment in to signal that the plugin has a way to configure it
        //bool hasConfiguration() const;
        //void triggerConfiguration();
    private:
        Ui::TurtlePluginWidget* m_pUi;
        QWidget* m_pWidget;

        ServiceCaller* m_pServiceCaller;
        

        std::string m_strSelectedTurtle;


    private slots:
        void on_btnReset_clicked();
        void on_btnSpawn_clicked();
        void on_btnColor_clicked();
        void on_btnDraw_clicked();
        void on_btnKill_clicked();

        void on_btnTeleportAbs_clicked();

        void on_selection_changed();
    };

} // namespace

#endif // rqt_turtle__turtle_plugin_H