#include <rqt_turtle/image_worker.h>

namespace rqt_turtle {

    JobRunner::JobRunner(Turtle turtle, std::vector<std::vector<cv::Point> > contours, float turtlesim_size)
        : turtle_(turtle)
    {
        is_killed_ = false;

        contours_ = contours;
        num_contours_ = contours.size();
        num_points_ = 0;
        for (auto contour : contours_)
        {
            num_points_ += contour.size();
        }
        idx_contour_ = 0;
        idx_point_ = 0;
        percent_ = 0;

        turtlesim_size_ = turtlesim_size;
    }

    void JobRunner::run()
    {
        try
        {
            idx_point_ = 0;
            ROS_INFO("%s draws %d contours", turtle_.name_.c_str(), num_contours_);
            for (auto contour : contours_)
            {
                ROS_INFO("%s draws contour %d of %d", turtle_.name_.c_str(), idx_contour_, num_contours_);
                turtle_.setPen(true);
                for (auto point : contour)
                {
                    percent_ = (int)(100.0 * (float)(idx_point_+1) / (float)num_points_);
                    emit progress(QString::fromStdString(turtle_.name_), percent_);

                    /// Normalize to turtle coordinates and flip on horizontal axis
                    sTeleportAbs_.request.x = point.x / turtlesim_size_ * 11.0;
                    sTeleportAbs_.request.y = (point.y / turtlesim_size_ * 11.0) - 11.0 / 2.0;
                    sTeleportAbs_.request.y = sTeleportAbs_.request.y * -1.0;
                    sTeleportAbs_.request.y = sTeleportAbs_.request.y + 11.0 / 2.0;
                    sTeleportAbs_.request.theta = 0.0; // todo use two points to calculate angle

                    if (is_killed_)
                    {
                        throw ImageWorkerKilledException();
                    }

                    ros::service::call<turtlesim::TeleportAbsolute>("/" + turtle_.name_ + "/teleport_absolute", sTeleportAbs_);
                    auto response = sTeleportAbs_.response;
                    turtle_.setPen(false);

                    idx_point_++;
                }
            }
        }
        catch (...)
        {
            ROS_INFO("Killed ImageWorker %s", turtle_.name_.c_str());
        }

        emit finished(QString::fromStdString(turtle_.name_));
    }

    void JobRunner::kill()
    {
        ROS_INFO("Kill received");
        is_killed_ = true;
    }

}