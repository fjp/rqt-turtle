#include <rqt_turtle/turtle.h>

#include <turtlesim/SetPen.h>


Turtle::Turtle(std::string name)
{
    name_ = name;

    pose_.x = 0.0;
    pose_.y = 0.0;
    pose_.theta = 0.0;

    pen_.r = DEFAULT_PEN_R;
    pen_.g = DEFAULT_PEN_G;
    pen_.b = DEFAULT_PEN_B;
    pen_.width = DEFAULT_PEN_WIDTH;
    pen_.off = false;
}


Turtle::Turtle(std::string name, float x, float y, float theta, 
    uint8_t r, uint8_t g, uint8_t b, uint8_t width, bool off)
{
    ROS_INFO("Created Turlte %s", name.c_str());
    name_ = name;

    pose_.x = x;
    pose_.y = y;
    pose_.theta = theta;

    pen_.r = r;
    pen_.g = g;
    pen_.b = b;
    pen_.width = width;
    pen_.off = off;
}


Turtle::Turtle(std::string name, turtlesim::Pose pose)
{
    name_ = name;

    pose_ = pose;

    pen_.r = DEFAULT_PEN_R;
    pen_.g = DEFAULT_PEN_G;
    pen_.b = DEFAULT_PEN_B;
    pen_.width = DEFAULT_PEN_WIDTH;
    pen_.off = false;
}


Turtle::Turtle(std::string name, turtlesim::Pose pose, turtlesim::SetPenRequest pen)
{
    name_ = name;
    pose_ = pose;
    pen_ = pen;
}


QTreeWidgetItem* Turtle::toTreeItem(QTreeWidget* parent)
{
    QTreeWidgetItem* item = new QTreeWidgetItem(parent);
    item->setText(0, QString::fromStdString(name_));
    item->setText(1, QString::number(pose_.x));
    item->setText(2, QString::number(pose_.y));
    item->setText(3, QString::number(pose_.theta));
    item->setText(4, pen_.off ? QString("off") : QString("on"));
    return item;
}


void Turtle::setPen(bool off)
{
    pen_.off = off;

    turtlesim::SetPen set_pen;
    set_pen.request.r = pen_.r;
    set_pen.request.g = pen_.g;
    set_pen.request.b = pen_.b;
    set_pen.request.width = pen_.width;
    set_pen.request.off = pen_.off;

    std::string service_name = "/" + name_ + "/set_pen";
    ros::service::call<turtlesim::SetPen>(service_name, set_pen);
}