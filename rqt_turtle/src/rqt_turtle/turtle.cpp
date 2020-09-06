#include <rqt_turtle/turtle.h>


Turtle::Turtle(std::string name)
{
    name_ = name;

    pose_.x = 0.0;
    pose_.y = 0.0;
    pose_.theta = 0.0;

    pen_.r = 0;
    pen_.g = 0;
    pen_.b = 0;
    pen_.width = 0;
    pen_.off = false;
}


Turtle::Turtle(std::string name, float x, float y, float theta, 
    uint8_t r, uint8_t g, uint8_t b, uint8_t width, bool off)
{
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

    pen_.r = 100;
    pen_.g = 100;
    pen_.b = 100;
    pen_.width = 1;
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