#include <ros/ros.h>

#include <QTreeWidgetItem>

#include <turtlesim/Pose.h>
#include <turtlesim/SetPenRequest.h>

class Turtle
{
public:
    Turtle(std::string name);
    Turtle(std::string name, turtlesim::Pose pose);
    Turtle(std::string name, turtlesim::Pose pose, turtlesim::SetPenRequest pen);
    Turtle(std::string name, float x, float y, float theta, 
        uint8_t r = 100, uint8_t g = 100, uint8_t b = 100, uint8_t width = 1, bool off = false);

    QTreeWidgetItem* toTreeItem(QTreeWidget* parent);

    std::string name_;
    turtlesim::Pose pose_;
    turtlesim::SetPenRequest pen_;

};