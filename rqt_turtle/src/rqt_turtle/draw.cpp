#include "rqt_turtle/draw.h"

#include <QDialog>
#include <QFileDialog>
#include <QMessageBox>
#include <QImageReader>
#include <QTimer>

#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>

// ROS releated headers

// Generated ui header
#include "ui_Draw.h"
#include "ui_Task.h"


#include <rqt_turtle/action_worker.h>
#include <rqt_turtle/image_worker.h>



namespace rqt_turtle {

    Draw::Draw(QWidget* parent, QMap<QString, QSharedPointer<Turtle>>& turtles)
        : ui_(new Ui::DrawWidget)
        , ui_task_(new Ui::TaskWidget)
        , draw_dialog_(this)
        , task_dialog_(new QDialog(this))
        , ac_("turtle_shape", true)
        , cancel_goal_(false)
        , turtles_(turtles)
    {
        // give QObjects reasonable names
        setObjectName("Draw");

        ROS_INFO("Initialize Draw Dialog");
        ui_->setupUi(draw_dialog_);
        ROS_INFO("Initialize Task Dialog");
        ui_task_->setupUi(task_dialog_);

        //connect(ui_->btnDraw, SIGNAL(clicked()), this, SLOT(on_btnDraw_clicked()));
        //connect(ui_->btnCancel, SIGNAL(clicked()), this, SLOT(on_btnCancel_clicked()));
        //connect(ui_->btnOpen, SIGNAL(clicked()), this, SLOT(on_btnOpen_clicked()));

        turtlesim_size_ = 500.0;
    }

    // https://stackoverflow.com/questions/28562401/resize-an-image-to-a-square-but-keep-aspect-ratio-c-opencv
    cv::Mat Draw::resizeImage(const cv::Mat& img)
    {
        int width = img.cols,
        height = img.rows;

        cv::Mat square = cv::Mat::zeros(turtlesim_size_, turtlesim_size_, img.type());

        int max_dim = (width >= height) ? width : height;
        float scale = ((float) turtlesim_size_) / max_dim;
        cv::Rect roi;
        if (width >= height)
        {
            roi.width = turtlesim_size_;
            roi.x = 0;
            roi.height = height * scale;
            roi.y = (turtlesim_size_ - roi.height) / 2;
        }
        else
        {
            roi.y = 0;
            roi.height = turtlesim_size_;
            roi.width = width * scale;
            roi.x = (turtlesim_size_ - roi.width) / 2;
        }

        cv::resize(img, square(roi), roi.size());

        return square;
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
            accept();
        }
        //accept();
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
        img_src_ = resizeImage(img_src_);

        if (!img_src_.data)
        {
            ROS_INFO("No image data");
            return;
        }
        else
        {
            ROS_INFO("Image rows, cols: %d, %d", img_src_.rows, img_src_.cols);
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
        image_worker_progress_.clear();
        QVector<JobRunner*> runners;
        turtlesim::Spawn spawn;
        QStringList names;
        for (int i = 0; i < contours_.size(); ++i)
        {
            QString name = QString("t") + QString::number(i);
            names.push_back(name);
            Turtle turtle(name.toStdString(), 0.0, 0.0, 0.0);
            spawn.request.name = turtle.name_.c_str();
            spawn.request.x = turtle.pose_.x;
            spawn.request.y = turtle.pose_.y;
            spawn.request.theta = turtle.pose_.theta;
            ros::service::call<turtlesim::Spawn>("/spawn", spawn);
            std::vector<std::vector<cv::Point> > contours(contours_.begin() + i, contours_.begin() + i + 1);
            ROS_INFO("%d contour with %d points", (int)contours.size(), (int)contours[0].size());
            JobRunner* runner = new JobRunner(turtle, contours, 500);
            runners.push_back(runner);
            // TODO implement progress correctly
            connect(runner, SIGNAL(progress(QString, int)), this, SLOT(update_image_progress(QString, int)));
            connect(runner, SIGNAL(finished(QString)), this, SLOT(cleanup_image_workers(QString)));
            connect(ui_task_->btnCancel, SIGNAL(clicked()), runner, SLOT(kill()));
        }

        connect(ui_task_->btnCancel, SIGNAL(clicked()), task_dialog_, SLOT(reject()));
        connect(ui_task_->btnCancel, SIGNAL(clicked()), this, SLOT(cancelDrawImage()));
        
        ROS_INFO("Start runners");
        for (auto runner : runners)
        {
            threadpool_.start(runner);
        }

        timer_ = QSharedPointer<QTimer>(new QTimer(this)); 
        timer_->setInterval(100);
        connect(timer_.data(), SIGNAL(timeout()), this, SLOT(refresh_progress()));
        timer_->start();

        task_dialog_->exec();
    }


    void Draw::update_image_progress(QString name, int progress)
    {
        ROS_INFO("Update image progress of %s: %d", name.toStdString().c_str(), progress);
        image_worker_progress_[name] = progress;
    }

    int Draw::calculate_progress()
    { 
        if (image_worker_progress_.isEmpty())
        {
            return 0;
        }
        
        auto values = image_worker_progress_.values();
        int sum = 0;
        for (auto value : values)
        {
            sum += value;
        }
        int progress = (float)sum / (float)image_worker_progress_.size();
        return progress;
    }

    void Draw::refresh_progress()
    {
        //# Calculate total progress.
        int progress = calculate_progress();
        ROS_INFO("Refresh progress: %d", progress);
        ui_task_->progressBar->setValue(progress);
        ui_task_->label->setText(QString("%1 workers").arg(image_worker_progress_.size()));
    }

    void Draw::cleanup_image_workers(QString name)
    {
        auto values = image_worker_progress_.values();
        if (std::all_of(values.constBegin(), values.constEnd(), [](int value){ return 100 == value; }))
        {
            // Update the progress bar if we've removed a value. 
            //refresh_progress();
            task_dialog_->accept();
            cancelDrawImage();
            image_worker_progress_.clear(); // Empty the map.
        }
    }

    void Draw::cancelDrawImage()
    {
        timer_->stop();
        auto names = image_worker_progress_.keys();
        for (auto name : names)
        {
            turtlesim::Kill kill;
            kill.request.name = name.toStdString();
            ros::service::call("kill", kill);
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

        /// Wait for the action server to start
        ac_.waitForServer(); //will wait for infinite time
        ROS_INFO("Action server started, sending goal.");

        /// Create ActionWorker which will send a goal to the action server
        int edges = ui_->lineEditEdges->text().toInt();
        float radius = ui_->lineEditRadius->text().toFloat();
        float timeout = ui_->lineEditTimeout->text().toFloat();
        ActionWorker* action_worker = new ActionWorker(ac_, edges, radius, timeout);

        ui_->btnDraw->setText(QString("Cancel Goal"));
        disconnect(ui_->btnDraw, SIGNAL(clicked()), this, SLOT(on_btnDraw_clicked()));
        connect(ui_->btnDraw, SIGNAL(clicked()), action_worker, SLOT(kill()));
        connect(ui_->btnDraw, SIGNAL(clicked()), this, SLOT(on_btnCancelGoal_clicked()));
        
        threadpool_.start(action_worker);
    }

    void Draw::on_btnCancelGoal_clicked()
    {
        ui_->btnDraw->setText("Draw");
        disconnect(ui_->btnDraw, SIGNAL(clicked()), this, SLOT(on_btnCancelGoal_clicked()));
        connect(ui_->btnDraw, SIGNAL(clicked()), this, SLOT(on_btnDraw_clicked()));
    }


} // namespace