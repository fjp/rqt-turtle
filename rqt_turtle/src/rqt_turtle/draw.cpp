#include "rqt_turtle/draw.h"

#include <QDialog>
#include <QFileDialog>
#include <QMessageBox>
#include <QImageReader>



// ROS releated headers

// Generated ui header
#include "ui_Draw.h"



namespace rqt_turtle {

    Draw::Draw(QWidget* parent)
        : ui_(new Ui::DrawWidget)
        , draw_dialog_(this)
        , ac_("turtle_shape", true)
    {
        // give QObjects reasonable names
        setObjectName("Draw");

        ROS_INFO("Initialize Draw Dialog");

        ui_->setupUi(draw_dialog_);

        //connect(ui_->btnDraw, SIGNAL(clicked()), this, SLOT(on_btnDraw_clicked()));
        //connect(ui_->btnCancel, SIGNAL(clicked()), this, SLOT(on_btnCancel_clicked()));
        //connect(ui_->btnOpen, SIGNAL(clicked()), this, SLOT(on_btnOpen_clicked()));

        
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
        
        img_dst_.create(img_src_.size(), img_src_.type());
        cv::cvtColor(img_src_, img_src_gray_, cv::COLOR_BGR2GRAY);
        on_sliderLowThreshold_valueChanged(0);

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
        ui_->lblImage->setPixmap(QPixmap::fromImage(image).scaledToWidth(ui_->lblImage->width()));
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
        ui_->lblEdgeImage->setPixmap(QPixmap::fromImage(img_detected_edges).scaledToWidth(ui_->lblImage->width()));
    }

    void Draw::on_sliderLowThreshold_valueChanged(int value)
    {
        /// Detect edges using Canny
        cannyThreshold(value);
        drawImage();
    }

    void Draw::drawImage()
    {
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::RNG rng(12345);

        /// Find contours
        cv::findContours(img_canny_, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

        /// Draw contours
        cv::Mat drawing = cv::Mat::zeros(img_canny_.size(), CV_8UC3);
        for(int i = 0; i < contours.size(); i++)
        {
            cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
            cv::drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
        }

        setEdgeImage(drawing);

    }

    // Callback for createTrackbar
    // https://answers.opencv.org/question/214973/how-to-create-a-class-for-trackbars-in-general/
    //void Draw::trackbarCallback(int pos, void* usrptr)
    //{
    //    // cast user data back to "this"
    //    Draw* draw = (Draw*)usrptr;
    //    draw->cannyThreshold(pos);
    //}

    void Draw::cannyThreshold(int low_threshold)
    {
        low_threshold_ = low_threshold;
        const int kernel_size = 3;
        const int max_lowThreshold = 100;
        const int ratio = 3;

        cv::blur(img_src_gray_, img_canny_, cv::Size(3,3));
        cv::Canny(img_canny_, img_canny_, low_threshold_, low_threshold_*ratio, kernel_size);

        //img_dst_ = cv::Scalar::all(0);
        //img_src_.copyTo(img_dst_, img_canny_);

        //setEdgeImage(img_dst_);
        
        //const char* window_name = "Edge Map";
        //cv::imshow(window_name, img_dst_);
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