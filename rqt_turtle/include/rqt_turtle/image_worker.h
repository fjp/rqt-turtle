#ifndef rqt_turtle__worker_H
#define rqt_turtle__worker_H

//import sys 
//import time
#include <QObject>
#include <QRunnable>
#include <QThreadPool>

#include <turtlesim/TeleportAbsolute.h>

#include <rqt_turtle/turtle.h>

#include <opencv2/core/types.hpp>


namespace rqt_turtle {

    class ImageWorkerKilledException{};


    class JobRunner : public QObject, public QRunnable
    {
        Q_OBJECT

        bool is_killed_;

        Turtle turtle_;
        std::vector<std::vector<cv::Point> > contours_;
        int num_contours_;
        int num_points_;
        int idx_contour_;
        int idx_point_;
        int percent_;
        float turtlesim_size_;

        turtlesim::TeleportAbsolute sTeleportAbs_;

    public:
        JobRunner(Turtle turtle, std::vector<std::vector<cv::Point> > contours, float turtlesim_size = 500.0);


        void run() override;

    signals:
        void progress(QString name, int value);
        void finished(QString name);

    public slots:
        void kill();
    };

}

#endif // rqt_turtle__worker_H