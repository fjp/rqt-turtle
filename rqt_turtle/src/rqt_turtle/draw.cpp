#include "rqt_turtle/draw.h"

#include <QDialog>
#include <QFileDialog>
#include <QMessageBox>
#include <QImageReader>

#include <opencv2/opencv.hpp>

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

        connect(ui_->btnDraw, SIGNAL(clicked()), this, SLOT(on_btnDraw_clicked()));
        connect(ui_->btnCancel, SIGNAL(clicked()), this, SLOT(on_btnCancel_clicked()));
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

        ROS_INFO("file name %s", file_name_.toStdString().c_str());

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

    void Draw::drawImage()
    {
        ROS_INFO("Find Contours");

        cv::Mat image;
        image = cv::imread(file_name_.toStdString(), 1);

        if (!image.data)
        {
            ROS_INFO("No image data");
            return;
        }
        cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Display Image", image);

        cv::waitKey(0);
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