#include "rqt_turtle/draw.h"

#include <QDialog>
#include <QMessageBox>

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
        connect(ui_->btnOpen, SIGNAL(clicked()), this, SLOT(on_btnOpen_clicked()));
    }

    void Draw::on_btnDraw_clicked()
    {
        if (ui_->tabs->currentWidget() == ui_->tabTurtleAction)
        {
            ROS_INFO("Draw Shape");
            drawShape();
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