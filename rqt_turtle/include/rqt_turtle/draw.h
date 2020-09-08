#ifndef rqt_turtle__draw_H
#define rqt_turtle__draw_H

#include <rqt_gui_cpp/plugin.h>
#include <QDialog>

namespace Ui {
    class DrawWidget;
}

namespace rqt_turtle {


    class Draw : public QDialog
    {
        Q_OBJECT
    public:
        Draw(QWidget* parent, ros::NodeHandle nh);

    private:
        Ui::DrawWidget* ui_;
        QDialog* draw_dialog_;
        ros::NodeHandle& nh_;


    private slots:
        void on_btnDraw_clicked();
        void on_btnCancel_clicked();

        void on_btnOpen_clicked();
    };

} // namespace

#endif // rqt_turtle__draw_H