#ifndef rqt_turtle__service_caller_H
#define rqt_turtle__service_caller_H

#include <rqt_gui_cpp/plugin.h>
#include <QDialog>

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


        QString getTurtleName();

        Ui::ServiceCallerWidget* m_pUi;
        QDialog* m_pServiceCallerDialog;


    private slots:
        void on_btnCall_clicked();
    };

} // namespace

#endif // rqt_turtle__service_caller_H