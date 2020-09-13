#include "rqt_turtle/draw.h"

#include <QDialog>
#include <QFileDialog>
#include <QMessageBox>
#include <QImageReader>

#include <turtlesim/TeleportAbsolute.h>

// ROS releated headers

// Generated ui header
#include "ui_Draw.h"



namespace rqt_turtle {

    Draw::Draw(QWidget* parent, QMap<QString, QSharedPointer<Turtle>>& turtles)
        : ui_(new Ui::DrawWidget)
        , draw_dialog_(this)
        , ac_("turtle_shape", true)
        , turtles_(turtles)
    {
        // give QObjects reasonable names
        setObjectName("Draw");

        ROS_INFO("Initialize Draw Dialog");

        ui_->setupUi(draw_dialog_);

        //connect(ui_->btnDraw, SIGNAL(clicked()), this, SLOT(on_btnDraw_clicked()));
        //connect(ui_->btnCancel, SIGNAL(clicked()), this, SLOT(on_btnCancel_clicked()));
        //connect(ui_->btnOpen, SIGNAL(clicked()), this, SLOT(on_btnOpen_clicked()));

        turtlesim_size_ = 500;
    }

    void Draw::setTurtleWorkers(QVector<QString> turtle_workers)
    {
        /// Fill list with selected turtles which will be used as workers
        turtle_workers_ = turtle_workers;
        ui_->listWorkers->addItems(turtle_workers.toList());
    }

    void Draw::on_btnDraw_clicked()
    {
        if (ui_->tabs->currentWidget() == ui_->tabTurtleAction)
        {
            ROS_INFO("Draw Shape");
            drawShape();
        }
        if (ui_->tabs->currentWidget() == ui_->tabImage)
        {
            ROS_INFO("Draw Image");
            drawImage();
        }

        accept();
    }

    void Draw::on_btnCancel_clicked()
    {
        ROS_INFO("Cancel clicked");

        reject();
    }

    void Draw::on_btnOpen_clicked()
    {
        ROS_INFO("Open clicked");

        file_name_ = QFileDialog::getOpenFileName(this,
            tr("Open Image"), QDir::homePath(), tr("Image Files (*.png *.jpg *.bmp)"));

        ROS_INFO("File name %s", file_name_.toStdString().c_str());

        QImageReader reader(file_name_);
        reader.setAutoTransform(true);
        const QImage image = reader.read();
        if (image.isNull()) {
            QMessageBox::information(this, QGuiApplication::applicationDisplayName(),
                                    tr("Cannot load %1: %2")
                                    .arg(QDir::toNativeSeparators(file_name_), reader.errorString()));
            return;// false;
        }

        setImage(image);

        ROS_INFO("Find Contours");

        img_src_ = cv::imread(file_name_.toStdString(), 1);

        if (!img_src_.data)
        {
            ROS_INFO("No image data");
            return;
        }
        
        //img_dst_.create(img_src_.size(), img_src_.type());
        cv::cvtColor(img_src_, img_src_gray_, cv::COLOR_BGR2GRAY);
        on_sliderLowThreshold_valueChanged(50);

        connect(ui_->sliderLowThreshold, SIGNAL(valueChanged(int)), this, SLOT(on_sliderLowThreshold_valueChanged(int)));

        //lowThreshold_ = 0;
        //const int max_lowThreshold = 100;
        //const char* window_name = "Edge Map";
        //cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE );
        //cv::createTrackbar("Min Threshold:", window_name, &this->lowThreshold_, max_lowThreshold, Draw::trackbarCallback, (void*)(this));
        //cannyThreshold(0);
        //cv::waitKey(0);
    }

    void Draw::setImage(const QImage &image)
    {
        //image_ = image;
        //ui_->lblImage->setPixmap(QPixmap::fromImage(image).scaledToWidth(ui_->lblImage->width()));
        ui_->lblImage->setPixmap(QPixmap::fromImage(image)
            .scaled(QSize(turtlesim_size_, turtlesim_size_), Qt::KeepAspectRatio));
        //ui_->lblImage->setScaledContents(true);
        //float scaleFactor = 0.5;

        //ui_->lblImage->resize(scaleFactor * ui_->lblImage->pixmap()->size());

        //if (!fitToWindowAct->isChecked())
        //ui_->lblImage->adjustSize();
    }

    void Draw::setEdgeImage(const cv::Mat& image)
    {
        // Convert cv::Mat to QImage
        // https://stackoverflow.com/questions/5026965/how-to-convert-an-opencv-cvmat-to-qimage
        QImage img_detected_edges = QImage((uchar*) image.data, image.cols, image.rows, image.step, QImage::Format_RGB888);
        ui_->lblEdgeImage->setPixmap(QPixmap::fromImage(img_detected_edges)
            .scaled(QSize(turtlesim_size_, turtlesim_size_), Qt::KeepAspectRatio));
    }

    void Draw::on_sliderLowThreshold_valueChanged(int value)
    {
        /// Detect edges using Canny
        cannyThreshold(value);
        previewEdgeImage();
    }

    void Draw::previewEdgeImage()
    {
        // https://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/find_contours/find_contours.html
        std::vector<cv::Vec4i> hierarchy;
        cv::RNG rng(12345);

        /// Find contours
        cv::findContours(img_canny_, contours_, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

        /// Draw contours
        cv::Mat drawing = cv::Mat::zeros(img_canny_.size(), CV_8UC3);
        ROS_INFO("Low threshold %d yields %d contours", low_threshold_, (int)contours_.size());
        for(int i = 0; i < contours_.size(); i++)
        {
            cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
            cv::drawContours(drawing, contours_, i, color, 2, 8, hierarchy, 0, cv::Point());
        }

        setEdgeImage(drawing);
    }

    void Draw::drawImage()
    {
        auto turtle = turtles_[QString("turtle1")];

        turtlesim::TeleportAbsolute sTeleportAbsolute;
        int num_contours =(int)contours_.size();
        int idx = 0;
        ROS_INFO("Draw Image with %d contours", num_contours);
        for (auto contour : contours_)
        {
            ROS_INFO("Drawing contour %d of %d", idx, num_contours);
            turtle->setPen(true);
            int idxp = 0;
            for (auto point : contour)
            {
                sTeleportAbsolute.request.x = point.x;// * 0.01;
                sTeleportAbsolute.request.y = point.y;// * 0.01;
                sTeleportAbsolute.request.theta = 0.0; // todo use two points to calculate angle

                if ((idx+idxp) % 100 == 0)
                {
                    ROS_INFO("Point (x,y)= (%f,%f)", sTeleportAbsolute.request.x, sTeleportAbsolute.request.y);
                }
                
                ros::service::call<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute", sTeleportAbsolute);
                auto response = sTeleportAbsolute.response;
                turtle->setPen(false);

                idxp++;
            }
            idx++;
        }
    }

    void Draw::cannyThreshold(int low_threshold)
    {
        low_threshold_ = low_threshold;
        const int kernel_size = 3;
        const int max_lowThreshold = 100;
        const int ratio = 3;

        cv::blur(img_src_gray_, img_canny_, cv::Size(3,3));
        cv::Canny(img_canny_, img_canny_, low_threshold_, low_threshold_*ratio, kernel_size);
    }

    void Draw::drawShape()
    {
        ROS_INFO("Waiting for action server to start.");

        if (!ac_.isServerConnected())
        {
            QMessageBox msgBox;
            msgBox.setText("Action server not connected");
            msgBox.setInformativeText("Please run 'rosrun turtle_actionlib shape_server' and press Ok or cancel \
                                       to avoid blocking rqt_turtle gui while wating for shape_server.");
            msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
            int ret = msgBox.exec();
            if (ret == QMessageBox::Cancel)
            {
                return;
            }
        }
        
        // wait for the action server to start
        ac_.waitForServer(); //will wait for infinite time

        ROS_INFO("Action server started, sending goal.");
        // send a goal to the action
        turtle_actionlib::ShapeGoal shape;
        shape.edges = ui_->lineEditEdges->text().toInt();
        shape.radius = ui_->lineEditEdges->text().toFloat();
        ac_.sendGoal(shape);

        //wait for the action to return
        float timeout = ui_->lineEditTimeout->text().toFloat();
        bool finished_before_timeout = ac_.waitForResult(ros::Duration(timeout));


        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac_.getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else
            ROS_INFO("Action did not finish before the time out.");

        //exit
        return; // TODO fix
    }


} // namespace