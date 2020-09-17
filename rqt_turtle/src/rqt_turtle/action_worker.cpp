#include <rqt_turtle/action_worker.h>

namespace rqt_turtle {

    ActionWorker::ActionWorker(actionlib::SimpleActionClient<turtle_actionlib::ShapeAction>& ac, int edges, float radius, float timeout)
        : ac_(ac)
    {
        edges_ = edges;
        radius_ = radius;
        timeout_ = timeout;
        is_killed_ = false;
    }

    void ActionWorker::run()
    {
        try
        {
            /// Send a goal to the action
            turtle_actionlib::ShapeGoal shape;
            shape.edges = edges_;
            shape.radius = radius_;

            /// Cancel action if it takes too long
            ros::Time end = ros::Time::now() + ros::Duration(timeout_);

            ac_.sendGoal(shape);

            actionlib::SimpleClientGoalState state = ac_.getState();
            bool loop = true;
            ros::Rate rate(1);
            while (loop)
            {   
                if (true == is_killed_) /// cancel_goal_
                {
                    ac_.cancelGoal();
                    ROS_INFO("Goal canceled");
                    break;
                }
                state = ac_.getState();
                switch (state.state_)
                {
                case actionlib::SimpleClientGoalState::PENDING:
                    ROS_INFO("PENDING -  The goal has yet to be processed by the action server.");
                    break;
                case actionlib::SimpleClientGoalState::ACTIVE:
                    ROS_INFO("ACTIVE - The goal is currently being processed by the action server.");
                    break;
                case actionlib::SimpleClientGoalState::RECALLED:
                    is_killed_ = true;
                    ROS_INFO("RECALLED - The goal has not been processed and a cancel request has been received from the action client, \
                                but the action server has not confirmed the goal is canceled.");
                    break;
                case actionlib::SimpleClientGoalState::REJECTED:
                    is_killed_ = true;
                    ROS_INFO("REJECTED - The goal was rejected by the action server without being processed and without a request from \
                                the action client to cancel");
                    break;
                case actionlib::SimpleClientGoalState::PREEMPTED:
                    is_killed_ = true;
                    ROS_INFO("PREEMPTED - The goal is being processed, and a cancel request has been received from the action client, \
                                but the action server has not confirmed the goal is canceled.");
                    break;
                case actionlib::SimpleClientGoalState::ABORTED:
                    is_killed_ = true;
                    ROS_INFO("ABORTED - The goal was terminated by the action server without an external request from the action client to cancel.");
                    break;
                case actionlib::SimpleClientGoalState::SUCCEEDED:
                    loop = false;
                    ROS_INFO("SUCCEEDED - The goal was achieved successfully by the action server.");
                    break;
                case actionlib::SimpleClientGoalState::LOST:
                    is_killed_ = true;
                    ROS_INFO("LOST");
                    break;
                }
                if (ros::Time::now() > end)
                {
                    is_killed_ = true;
                    ROS_INFO("Time out - Action took too long");
                }
                rate.sleep();
            }

            
        }
        catch (...)
        {
            ROS_INFO("Killed Action Worker");
        }
    }

    void ActionWorker::kill()
    {
        ROS_INFO("Kill received");
        is_killed_ = true;
    }

}