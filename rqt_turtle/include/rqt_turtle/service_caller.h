#ifndef rqt_turtle__service_caller_H
#define rqt_turtle__service_caller_H

#include <rqt_gui_cpp/plugin.h>
#include <QDialog>
#include <QVariantMap>
#include <QTreeWidget>

class QListWidgetItem;

namespace Ui {
    class ServiceCallerWidget;
}

namespace rqt_turtle {


    class ServiceCaller : public QDialog
    {
        Q_OBJECT
    public:
        ServiceCaller(QWidget* parent, std::string service_name);

        QString getTurtleName();

        QVariantMap getRequest();

        

    private:
        /**
         * @brief Execute a command in the terminal and get the string result.
         * 
         * @param cmd The command to execute.
         * @return std::string The output of the executed command.
         */
        std::string exec_cmd(std::string str_cmd);


        void createTreeItems(std::string service_name);


        Ui::ServiceCallerWidget* m_pUi;
        QDialog* m_pServiceCallerDialog;
        std::string service_name_;


    private slots:
        void on_btnCall_clicked();
    };

} // namespace

#endif // rqt_turtle__service_caller_H