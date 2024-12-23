#ifndef PID_CONTROLLER_NODE_H
#define PID_CONTROLLER_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

namespace pid_controller
{

    class PIDController
    {
    public:
        PIDController(ros::NodeHandle &nh);
        ~PIDController();

        void pathCallback(const nav_msgs::Path::ConstPtr &msg);
        void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
        void diffVelCallback(const geometry_msgs::Twist::ConstPtr &diff_vel);

        geometry_msgs::Twist calculateVelocity(const geometry_msgs::PoseStamped &current_pose,
                                               nav_msgs::Path &path);

        // void calculateVelocity(const geometry_msgs::PoseStamped &current_pose,
        //                                        const nav_msgs::Path &path);
        geometry_msgs::PoseStamped transformPose(const geometry_msgs::PoseStamped &pose,
                                                 const std::string &target_frame);
        double calculateDistance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2);
        bool isObstacle(const geometry_msgs::Point &point);
        int getCostmapIndex(double x, double y);
        void publishLookaheadMarker(const geometry_msgs::PoseStamped &pose);
        void publishLookaheadMarker(const geometry_msgs::PoseStamped &pose, std::string);

        void publishVelocity();

        bool path_received_;                    // 是否收到路径
        bool costmap_received_;                 // 是否收到代价地图
        nav_msgs::Path global_path_;            // 全局路径
        nav_msgs::Path last_global_path_;            // 全局路径
        nav_msgs::OccupancyGrid local_costmap_; // 局部代价地图
        std::string global_frame_;              // 全局坐标系
        std::string robot_base_frame_;          // 机器人坐标系

        // private:
        ros::NodeHandle nh_;
        ros::Subscriber path_sub_;
        ros::Subscriber costmap_sub_;
        ros::Subscriber diff_vel_sub_;
        ros::Publisher cmd_vel_pub_;
        ros::Publisher marker_pub_; // 用于发布前瞻点的发布器

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf2_listener_;

        geometry_msgs::Twist cmd_vel_global;
        geometry_msgs::Twist diff_vel_global;

        double kp_x_;      // X方向比例系数
        double ki_x_;      // X方向积分系数
        double kd_x_;      // X方向微分系数
        double kp_y_;      // Y方向比例系数
        double ki_y_;      // Y方向积分系数
        double kd_y_;      // Y方向微分系数
        double max_vel_x_; // X方向最大速度
        double max_vel_y_; // Y方向最大速度
        double tolerance_; // 到达目标点容忍距离
        double lookahead_distance_base_;
        double lookahead_distance_factor_; // 前瞻距离
        double lookahead_distance_;        // 前瞻距离
        double costmap_threshold_;         // 代价地图阈值
        double draw_back_;

        double error_x_prev_; // 上一次X方向误差
        double error_y_prev_; // 上一次Y方向误差
        double integral_x_;   // X方向积分项
        double integral_y_;   // Y方向积分项

        double physic_vmax_rate_;
        // int point_index;
    };

} // namespace pid_controller

#endif // PID_CONTROLLER_NODE_H