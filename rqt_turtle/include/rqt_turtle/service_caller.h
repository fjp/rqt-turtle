#ifndef rqt_turtle__service_caller_H
#define rqt_turtle__service_caller_H

#include <rqt_gui_cpp/plugin.h>
#include <QDialog>

#include <turtlesim/Pose.h>

class QListWidgetItem;

namespace Ui {
    class ServiceCallerWidget;
}

namespace rqt_turtle {


    class ServiceCaller : public QDialog
    {
        Q_OBJECT
    public:
        ServiceCaller(QWidget* parent);

        bool lookupService(const std::string &name);//, std::string &serv_host, uint32_t &serv_port)


        QString getTurtleName();
        turtlesim::Pose getTurtleLocation();

        Ui::ServiceCallerWidget* m_pUi;
        QDialog* m_pServiceCallerDialog;


        std::string execute_command(const char* cmd);


    private slots:
        void on_btnCall_clicked();
    };

} // namespace

#endif // rqt_turtle__service_caller_H