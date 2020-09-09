#ifndef rqt_turtle__draw_H
#define rqt_turtle__draw_H

#include <rqt_gui_cpp/plugin.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <turtle_actionlib/ShapeAction.h>

#include <QDialog>

namespace Ui {
    class DrawWidget;
}

namespace rqt_turtle {


    class Draw : public QDialog
    {
        Q_OBJECT
    public:
        Draw(QWidget* parent);

    private:
        Ui::DrawWidget* ui_;
        QDialog* draw_dialog_;

        // create the action client
        // true causes the client to spin its own thread
        // http://docs.ros.org/noetic/api/actionlib/html/classactionlib_1_1SimpleActionClient.html
        actionlib::SimpleActionClient<turtle_actionlib::ShapeAction> ac_;

        void drawShape();

    private slots:
        void on_btnDraw_clicked();
        void on_btnCancel_clicked();

        void on_btnOpen_clicked();
    };

} // namespace

#endif // rqt_turtle__draw_H