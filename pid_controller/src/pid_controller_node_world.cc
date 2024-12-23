#include "pid_controller_node.h"
#include <tf2/utils.h>
#include <utils.hpp>

namespace pid_controller
{

    PIDController::PIDController(ros::NodeHandle &nh)
        : nh_(nh),
          tf2_listener_(tf_buffer_),
          error_x_prev_(0.0),
          error_y_prev_(0.0),
          integral_x_(0.0),
          integral_y_(0.0),
          path_received_(false),
          costmap_received_(false)
    {

        std::string path_topic, map_topic, cmd_topic, diff_vel_topic;
        // 参数初始化
        nh_.param("kp_x", kp_x_, 1.0);
        nh_.param("ki_x", ki_x_, 0.0);
        nh_.param("kd_x", kd_x_, 0.1);
        nh_.param("kp_y", kp_y_, 1.0);
        nh_.param("ki_y", ki_y_, 0.0);
        nh_.param("kd_y", kd_y_, 0.1);
        nh_.param("max_vel_x", max_vel_x_, 0.5);
        nh_.param("max_vel_y", max_vel_y_, 0.5);
        nh_.param("tolerance", tolerance_, 0.1);
        nh_.param("global_frame", global_frame_, std::string("map"));
        nh_.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
        nh_.param("lookahead_distance_base", lookahead_distance_base_, 5.0);
        nh_.param("lookahead_distance_factor", lookahead_distance_factor_, 0.5);
        nh_.param("costmap_threshold", costmap_threshold_, 50.0);
        nh_.param("draw_back", draw_back_, 0.8);

        nh_.param("path_topic", path_topic, std::string("/move_base1/NavfnROS/plan"));
        nh_.param("map_topic", map_topic, std::string("/move_base1/local_costmap/costmap"));
        nh_.param("cmd_topic", cmd_topic, std::string("/cmd_vel_test"));
        nh_.param("diff_vel_topic", diff_vel_topic, std::string("/cmd_vel_diff"));
        nh_.param("physic_vmax_rate", physic_vmax_rate_, 1.0);

        // 订阅全局路径和局部代价地图
        path_sub_ = nh_.subscribe<nav_msgs::Path>(path_topic, 1, &PIDController::pathCallback, this);
        costmap_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(map_topic, 1, &PIDController::costmapCallback, this);
        diff_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>(diff_vel_topic, 10, &PIDController::diffVelCallback, this);

        // 发布速度指令
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic, 1);
        // 发布前瞻点用于可视化
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("lookahead_marker", 1);
    }

    PIDController::~PIDController() {}

    void PIDController::pathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        // 接收全局路径
        global_path_ = *msg;
        path_received_ = true;
        ROS_INFO("Received global path with %zu waypoints.", global_path_.poses.size());
    }

    void PIDController::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        // 接收局部代价地图
        local_costmap_ = *msg;
        costmap_received_ = true;
        // ROS_INFO("Received local costmap."); // 这句可能太频繁了，可以根据需要打开
    }

    void PIDController::diffVelCallback(const geometry_msgs::Twist::ConstPtr &diff_vel)
    {
        diff_vel_global = *diff_vel;
    }

    geometry_msgs::Twist PIDController::calculateVelocity(const geometry_msgs::PoseStamped &current_pose, nav_msgs::Path &path)
    {
        // auto path = path_.clone();
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;

        if (path.poses.empty())
        {
            ROS_WARN("Global path is empty.");
            // return cmd_vel;
            path = last_global_path_;
        }
        else
        {
            last_global_path_ = path;
        }

        // 找到前瞻点
        geometry_msgs::PoseStamped target_pose;
        for (size_t i = size_t(path.poses.size() * 0.5); i > 0; --i)
        {
            if (calculateDistance(current_pose.pose.position, path.poses[i].pose.position) <= lookahead_distance_)
            {
                target_pose = path.poses[i];
                break;
            }
            // 如果路径结束还没有找到符合的点，则选取最后一个点作为目标
            if (i == path.poses.size() - 1)
            {
                target_pose = path.poses.back();
            }
        }

        // 发布前瞻点用于可视化
        marker_pub_.publish(visualutils::pointMarkerMsgGenerator(target_pose, "target_pose", global_frame_));
        marker_pub_.publish(visualutils::pointMarkerMsgGenerator(current_pose, "current_pose", global_frame_));
        // publishLookaheadMarker(target_pose, "target_pose");
        // publishLookaheadMarker(current_pose, "current_pose");

        // 检查前瞻点是否为障碍物
        if (isObstacle(target_pose.pose.position))
        {
            ROS_WARN("Target pose is an obstacle. Stopping.");
            return cmd_vel;
        }
        // 计算当前位置和前瞻点之间的误差
        double error_x = target_pose.pose.position.x - current_pose.pose.position.x;
        double error_y = target_pose.pose.position.y - current_pose.pose.position.y;

        // 计算距离差
        double distance_error = calculateDistance(current_pose.pose.position, target_pose.pose.position);
        // 如果距离小于容忍距离，则认为到达目标点
        if (distance_error < tolerance_)
        {
            ROS_INFO("Reached goal point.");
            return cmd_vel;
        }

        double yaw = tf2::getYaw(current_pose.pose.orientation);
        // double ex = error_x * cos(yaw) + error_y * sin(yaw);
        // double ey = -error_x * sin(yaw) + error_y * cos(yaw);
        // // 接下来都是对于雷达坐标系做的控制了
        // error_x = ex;
        // error_y = ey;

        // 计算 PID 控制量
        double dt = 0.1; // 假设控制周期为 0.1s
        double p_term_x = kp_x_ * error_x;
        double i_term_x = ki_x_ * integral_x_;
        double d_term_x = kd_x_ * (error_x - error_x_prev_) / dt;

        double p_term_y = kp_y_ * error_y;
        double i_term_y = ki_y_ * integral_y_;
        double d_term_y = kd_y_ * (error_y - error_y_prev_) / dt;

        auto vx = p_term_x + i_term_x + d_term_x;
        auto vy = p_term_y + i_term_y + d_term_y;
        if (abs(vx) > max_vel_x_)
            vx = max_vel_x_ * vx / abs(vx);
        if (abs(vy) > max_vel_y_)
            vy = max_vel_y_ * vy / abs(vy);

        ROS_INFO("ex:%.3f ey:%.3f vxw:%.3f vyw:%.3f", error_x, error_y, vx, vy);
        vx /= physic_vmax_rate_;
        vx /= physic_vmax_rate_;

        // 转换到车体坐标系
        cmd_vel.linear.x = vx * cos(yaw) + vy * sin(yaw);
        cmd_vel.linear.y = -vx * sin(yaw) + vy * cos(yaw);

        // cmd_vel.linear.x = max_vel_x_;
        // cmd_vel.linear.y = max_vel_y_;

        // 更新误差和积分项
        error_x_prev_ = error_x;
        error_y_prev_ = error_y;
        integral_x_ += error_x * dt;
        integral_y_ += error_y * dt;

        return cmd_vel;
    }

    geometry_msgs::PoseStamped PIDController::transformPose(const geometry_msgs::PoseStamped &pose,
                                                            const std::string &target_frame)
    {
        geometry_msgs::PoseStamped transformed_pose;
        try
        {
            // 使用 tf2_ros 进行坐标转换
            transformed_pose = tf_buffer_.transform(pose, target_frame);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        return transformed_pose;
    }

    double PIDController::calculateDistance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
    {
        // 计算两点之间的欧几里得距离
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

    bool PIDController::isObstacle(const geometry_msgs::Point &point)
    {
        // 检查点是否在局部代价地图中是障碍物
        if (!costmap_received_)
        {
            ROS_WARN("Local costmap not received yet.");
            return false;
        }
        int index = getCostmapIndex(point.x, point.y);
        if (index < 0)
        {
            return false;
        }
        return local_costmap_.data[index] >= costmap_threshold_;
    }

    int PIDController::getCostmapIndex(double x, double y)
    {
        // 根据坐标获取代价地图中的索引
        if (!costmap_received_)
        {
            ROS_WARN("Local costmap not received yet.");
            return -1;
        }
        // 将坐标转换为地图坐标系下的坐标
        double resolution = local_costmap_.info.resolution;
        int map_x = (int)((x - local_costmap_.info.origin.position.x) / resolution);
        int map_y = (int)((y - local_costmap_.info.origin.position.y) / resolution);

        // 检查是否越界
        if (map_x < 0 || map_x >= local_costmap_.info.width || map_y < 0 || map_y >= local_costmap_.info.height)
        {
            // ROS_WARN("Point is outside the costmap."); // 这句可能太频繁了，可以根据需要打开
            return -1;
        }

        return map_y * local_costmap_.info.width + map_x;
    }

    void PIDController::publishLookaheadMarker(const geometry_msgs::PoseStamped &pose, std::string ns_)
    {
        // 发布前瞻点用于可视化
        // visualization_msgs::Marker marker;
        // marker.header.frame_id = global_frame_;
        // marker.header.stamp = ros::Time::now();
        // marker.ns = ns_;
        // marker.id = 0;
        // marker.type = visualization_msgs::Marker::SPHERE;
        // marker.action = visualization_msgs::Marker::ADD;
        // marker.pose = pose.pose;
        // marker.scale.x = 0.2;
        // marker.scale.y = 0.2;
        // marker.scale.z = 0.2;
        // marker.color.a = 1.0;
        // marker.color.r = 0.0;
        // marker.color.g = 1.0;
        // marker.color.b = 0.0;
        // auto t = visualutils::pointMarkerMsgGenerator()
        // marker_pub_.publish(marker);
    }
    void PIDController::publishVelocity()
    {
        if (path_received_ && costmap_received_)
        {
            // 获取当前机器人位姿
            geometry_msgs::PoseStamped current_pose;
            current_pose.header.frame_id = robot_base_frame_;
            current_pose.header.stamp = ros::Time(0); // 使用最新的时间戳
            current_pose.pose.orientation.w = 1.0;    //  默认姿态
            geometry_msgs::PoseStamped current_pose_global;

            try
            {
                lookahead_distance_ = lookahead_distance_factor_;
                current_pose_global = tf_buffer_.transform(current_pose, global_frame_);
                geometry_msgs::Twist cmd_vel_body = calculateVelocity(current_pose_global, global_path_);
                ROS_INFO("x:%.4f y:%.4f fwd:%.2f", cmd_vel_body.linear.x, cmd_vel_body.linear.y, lookahead_distance_);

                cmd_vel_pub_.publish(cmd_vel_body);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
            }
        }
    }
} // namespace pid_controller

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_controller_node");
    ros::NodeHandle nh;

    pid_controller::PIDController controller(nh);

    ros::Rate loop_rate(100); // 控制频率为 10Hz

    while (ros::ok())
    {
        controller.publishVelocity();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}