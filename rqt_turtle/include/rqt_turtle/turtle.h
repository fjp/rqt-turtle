#ifndef rqt_turtle__turtle_H
#define rqt_turtle__turtle_H

#include <ros/ros.h>

#include <QTreeWidgetItem>

#include <turtlesim/Pose.h>
#include <turtlesim/SetPenRequest.h>

#define DEFAULT_PEN_R 0xb3
#define DEFAULT_PEN_G 0xb8
#define DEFAULT_PEN_B 0xff

#define DEFAULT_PEN_WIDTH 3

class Turtle
{
public:
    Turtle(std::string name);
    Turtle(std::string name, turtlesim::Pose pose);
    Turtle(std::string name, turtlesim::Pose pose, turtlesim::SetPenRequest pen);
    Turtle(std::string name, float x, float y, float theta, 
        uint8_t r = DEFAULT_PEN_R, uint8_t g = DEFAULT_PEN_G, uint8_t b = DEFAULT_PEN_B, uint8_t width = 3, bool off = false);

    QTreeWidgetItem* toTreeItem(QTreeWidget* parent);

    void setPen(bool off);

    std::string name_;
    turtlesim::Pose pose_;
    turtlesim::SetPenRequest pen_;

};

#endif // rqt_turtle__turtle_H