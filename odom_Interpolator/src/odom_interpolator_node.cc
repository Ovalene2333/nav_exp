#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <deque>

class OdomInterpolator
{
public:
    OdomInterpolator() : nh_("~")
    {
        // 获取参数
        nh_.param<std::string>("odom_topic", odom_topic_, "/Odometry");
        nh_.param<std::string>("interpolated_odom_topic", interpolated_odom_topic_, "/odom_interpolated");
        nh_.param<std::string>("diff_vel_topic", diff_vel_topic_, "/cmd_vel_diff");
        nh_.param<double>("interpolation_rate", interpolation_rate_, 100.0);

        // 发布和订阅
        odom_sub_ = nh_.subscribe(odom_topic_, 10, &OdomInterpolator::odomCallback, this);
        interpolated_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(interpolated_odom_topic_, 10);
        diff_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(diff_vel_topic_, 10);

        // 初始化时间
        last_odom_time_ = ros::Time::now();
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // 缓存里程计数据
        odom_buffer_.push_back(*msg);

        // 保持缓冲区大小
        if (odom_buffer_.size() > 10)
        {
            odom_buffer_.pop_front();
        }

        // 计算时间间隔
        ros::Time current_time = ros::Time::now();
        ros::Duration dt = current_time - last_odom_time_;

        // 检查是否需要插值
        if (dt.toSec() >= 1.0 / interpolation_rate_)
        {
            interpolateAndPublish();
            last_odom_time_ = current_time;
        }
    }

    void interpolateAndPublish()
    {
        // 确保缓冲区有足够数据
        if (odom_buffer_.size() < 2)
        {
            return;
        }

        // 获取最近的两个里程计数据
        nav_msgs::Odometry odom_prev = odom_buffer_[odom_buffer_.size() - 2];
        nav_msgs::Odometry odom_curr = odom_buffer_[odom_buffer_.size() - 1];

        // 计算时间差
        double dt = (odom_curr.header.stamp - odom_prev.header.stamp).toSec();

        // 计算速度（位置差分）
        double vx = (odom_curr.pose.pose.position.x - odom_prev.pose.pose.position.x) / dt;
        double vy = (odom_curr.pose.pose.position.y - odom_prev.pose.pose.position.y) / dt;
        double vz = (odom_curr.pose.pose.position.z - odom_prev.pose.pose.position.z) / dt;

        // 计算角速度（姿态差分）
        // tf::Quaternion q_prev, q_curr;
        // tf::quaternionMsgToTF(odom_prev.pose.pose.orientation, q_prev);
        // tf::quaternionMsgToTF(odom_curr.pose.pose.orientation, q_curr);
        // tf::Quaternion q_diff = q_curr * q_prev.inverse();
        // tf::Matrix3x3 m_diff(q_diff);
        // double roll_diff, pitch_diff, yaw_diff;
        // m_diff.getRPY(roll_diff, pitch_diff, yaw_diff);
        // double v_roll = roll_diff / dt;
        // double v_pitch = pitch_diff / dt;
        // double v_yaw = yaw_diff / dt;

        // 插值位姿
        // 线性插值位置
        ros::Time current_time = ros::Time::now();
        double interpolation_factor = (current_time - odom_prev.header.stamp).toSec() / dt;
        nav_msgs::Odometry interpolated_odom = odom_prev;
        interpolated_odom.header.stamp = current_time;
        interpolated_odom.pose.pose.position.x = odom_prev.pose.pose.position.x + vx * (current_time - odom_prev.header.stamp).toSec();
        interpolated_odom.pose.pose.position.y = odom_prev.pose.pose.position.y + vy * (current_time - odom_prev.header.stamp).toSec();
        interpolated_odom.pose.pose.position.z = odom_prev.pose.pose.position.z + vz * (current_time - odom_prev.header.stamp).toSec();

        // 线性插值姿态(四元数球面线性插值)
        // tf::Quaternion q_interpolated = q_prev.slerp(q_curr, interpolation_factor);
        // tf::quaternionTFToMsg(q_interpolated, interpolated_odom.pose.pose.orientation);

        // 设置速度
        interpolated_odom.twist.twist.linear.x = vx;
        interpolated_odom.twist.twist.linear.y = vy;
        interpolated_odom.twist.twist.linear.z = vz;
        // interpolated_odom.twist.twist.angular.x = v_roll;
        // interpolated_odom.twist.twist.angular.y = v_pitch;
        // interpolated_odom.twist.twist.angular.z = v_yaw;

        geometry_msgs::Twist cmd_vel_world;
        cmd_vel_world.linear.x = vx;
        cmd_vel_world.linear.y = vy;

        // 发布插值后的里程计数据
        interpolated_odom_pub_.publish(interpolated_odom);
        diff_vel_pub_.publish(cmd_vel_world);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher interpolated_odom_pub_, diff_vel_pub_;
    std::string odom_topic_;
    std::string interpolated_odom_topic_, diff_vel_topic_;
    double interpolation_rate_;
    ros::Time last_odom_time_;
    std::deque<nav_msgs::Odometry> odom_buffer_; // 里程计数据缓冲区
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_interpolator");
    OdomInterpolator interpolator;
    ros::spin();
    return 0;
}