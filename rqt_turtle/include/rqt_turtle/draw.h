#ifndef rqt_turtle__draw_H
#define rqt_turtle__draw_H

#include <rqt_gui_cpp/plugin.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <turtle_actionlib/ShapeAction.h>

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <QDialog>

#include <rqt_turtle/turtle.h>

namespace Ui {
    class DrawWidget;
}

namespace rqt_turtle {


    class Draw : public QDialog
    {
        Q_OBJECT
    public:
        Draw(QWidget* parent, QMap<QString, QSharedPointer<Turtle>>& turtles);

        void setTurtleWorkers(QVector<QString> turtle_workers);

    private:
        Ui::DrawWidget* ui_;
        QDialog* draw_dialog_;

        QString file_name_;
        float turtlesim_size_;

        cv::Mat img_src_;
        cv::Mat img_src_gray_;
        cv::Mat img_canny_;
        int low_threshold_;

        QMap<QString, QSharedPointer<Turtle>>& turtles_;
        QVector<QString> turtle_workers_;
        std::vector<std::vector<cv::Point> > contours_;

        // create the action client
        // true causes the client to spin its own thread
        // http://docs.ros.org/noetic/api/actionlib/html/classactionlib_1_1SimpleActionClient.html
        actionlib::SimpleActionClient<turtle_actionlib::ShapeAction> ac_;

        cv::Mat resizeImage(const cv::Mat& img);

        void drawShape();
        void previewEdgeImage();
        void drawImage();

        void setImage(const QImage &image);
        void setEdgeImage(const cv::Mat& image);

        void cannyThreshold(int pos);

    private slots:
        void on_btnDraw_clicked();
        void on_btnCancel_clicked();

        void on_btnOpen_clicked();

        void on_sliderLowThreshold_valueChanged(int low_threshold);
    };

} // namespace

#endif // rqt_turtle__draw_H