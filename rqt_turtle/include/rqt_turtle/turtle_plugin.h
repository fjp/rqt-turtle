#ifndef rqt_turtle__turtle_plugin_H
#define rqt_turtle__turtle_plugin_H

#include <rqt_gui_cpp/plugin.h>

#include <QWidget>
#include <QSharedPointer>
#include <QVariantMap>

#include <rqt_turtle/turtle.h>


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
        /**
         * @brief Construct a new Turtle Plugin object
         * 
         * @details All qt signal to slot connections are done here.
         */
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
        QVector<QString> m_vSelectedTurtles;

        // Vector to keep track of all turtles (keep turtles on the heap using vector of shared pointers)
        QMap<QString, QSharedPointer<Turtle> > turtles_;


        /**
         * @brief Teleport selected turtle(s)
         * 
         * @details teleport is used for both slots, on_btnTeleportAbs_clicked and 
         * on_btnTeleportRel_clicked. It will create a new ServiceCaller widget,
         * which allows to enter the coordinates where the turtle should be teleported.
         * Depending on the pressed button ('Teleport Abs' or 'Teleport Rel'), either
         * the '/teleport_absoulte' or '/teleport_relative' service will be executed with the
         * provided coordinates.
         * 
         * @param teleport_type Can be one of /teleport_absolute or /teleport_relative
         */
        QVariantMap teleport(std::string teleport_type);

        void updateTurtleTree();


    private slots:
        /**
         * @brief Callback for Reset push button
         * 
         * @details Pressing the Reset button will call the '/reset' service
         * and clear the QTreeWidget holding the currently active turtles.
         * 
         */
        void on_btnReset_clicked();

        /**
         * @brief Callback/Slot for Spawn button to create a new turtle.
         * 
         * @details This opens a new ServiceCaller widget with information
         * to spawn a new turtle (x, y, theta, name). Press the Call button
         * to create the a new turtle with the specified pose.
         * 
         */
        void on_btnSpawn_clicked();

        /**
         * @brief Change the background color of turtlesim
         * 
         * @details Pressing the Color button opens a QColorDialog.
         * After selecting the color a /reset is needed to actually update the 
         * background color of turtlesim. For this the Reset push button can be used.
         * 
         */
        void on_btnColor_clicked();
        void on_btnDraw_clicked();

        /**
         * @brief Callback/Slot to kill/delete the selected turtles.
         * 
         * @details For all turtles currently selected in the QTreeWidget,
         * the /kill service is called and their entry is removed
         * form the QTreeWidget.
         * 
         */
        void on_btnKill_clicked();

        /**
         * @brief Callback/Slot for Teleport Abs push button.
         * 
         * @details This will open a ServiceCaller widget
         * to enter the absolute coordinates where the selected turtles
         * should be teleported. The main logic is inside the teleport() method,
         * which is reused by on_btnTeleportRel_clicked().
         * 
         */
        void on_btnTeleportAbs_clicked();

        /**
         * @brief Callback/Slot for Teleport Abs push button
         * 
         * @details This will open a ServiceCaller widget
         * to enter the absolute coordinates where the selected turtles
         * should be teleported. The main logic is inside the teleport() method,
         * which is reused by on_btnTeleportAbs_clicked().
         * 
         */
        void on_btnTeleportRel_clicked();


        void on_btnTogglePen_clicked();

        /**
         * @brief Keeps track of the slected turtles
         * 
         * @details When selecting different turtles in the QTreeWidget,
         * the member storing the selected turtles is updated.
         * 
         */
        void on_selection_changed();
    };

} // namespace

#endif // rqt_turtle__turtle_plugin_H