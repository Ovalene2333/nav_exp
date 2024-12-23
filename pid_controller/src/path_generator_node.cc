#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <queue>
#include <vector>
#include <cmath>
#include <map>
#include <utils.hpp>

// 定义一个简单的位置结构
struct Position
{
    int x;
    int y;

    bool operator==(const Position &other) const
    {
        return x == other.x && y == other.y;
    }

    bool operator<(const Position &other) const
    {
        if (x != other.x)
        {
            return x < other.x;
        }
        return y < other.y;
    }
};

// 计算哈希值
namespace std
{
    template <>
    struct hash<Position>
    {
        size_t operator()(const Position &p) const
        {
            return ((hash<int>()(p.x) ^ (hash<int>()(p.y) << 1)) >> 1);
        }
    };
}

class LocalPathPlanner
{
public:
    LocalPathPlanner() : nh_("~"), tf_listener_(tf_buffer_)
    {
        // 订阅地图和全局路径
        map_sub_ = nh_.subscribe("/move_base1/local_costmap/costmap", 1, &LocalPathPlanner::mapCallback, this);
        global_path_sub_ = nh_.subscribe("/move_base1/NavfnROS/plan", 1, &LocalPathPlanner::globalPathCallback, this);

        // 发布路径Marker
        path_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("local_path_marker", 1);

        // 配置A* 搜索的参数
        step_size_ = 3;         // 设置步长，可以调整
        heuristic_scale_ = 1.0; // 启发式函数的权重，可以调整
        path_pub_rate_ = 10.0;
        timer_ = nh_.createTimer(ros::Duration(1.0 / path_pub_rate_), &LocalPathPlanner::timerCallback, this);

        robot_frame_ = "base_link"; // 设置机器人frame，可以调整
        map_frame_ = "map";
    }

    void timerCallback(const ros::TimerEvent &event)
    {
        if (map_data_.data.empty() || global_path_.poses.empty())
        {
            ROS_INFO_THROTTLE(2.0, "Waiting for map and global path data...");
            return;
        }

        // 1. 获取机器人位置
        geometry_msgs::PoseStamped robot_pose;
        if (!getRobotPose(robot_pose))
        {
            ROS_WARN_THROTTLE(2.0, "Failed to get robot pose");
            return;
        }

        // 2. 计算目标点
        Position start_pos = worldToMap(robot_pose.pose.position.x, robot_pose.pose.position.y);
        Position goal_pos;
        if (!getGoalPosition(robot_pose.pose, goal_pos))
        {
            ROS_WARN_THROTTLE(2.0, "Failed to get Goal position");
            return;
        }

        // 3. 进行 A* 搜索
        std::vector<Position> path = aStarSearch(start_pos, goal_pos);

        // 4. 可视化路径
        publishPathMarker(path);
    }

    // 地图回调函数
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        map_data_ = *msg;
    }

    // 全局路径回调函数
    void globalPathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        global_path_ = *msg;
    }

    // 将世界坐标转换为地图坐标
    Position worldToMap(double wx, double wy)
    {
        int mx = static_cast<int>((wx - map_data_.info.origin.position.x) / map_data_.info.resolution);
        int my = static_cast<int>((wy - map_data_.info.origin.position.y) / map_data_.info.resolution);
        return {mx, my};
    }

    // 将地图坐标转换为世界坐标
    geometry_msgs::Point mapToWorld(int mx, int my)
    {
        geometry_msgs::Point p;
        p.x = map_data_.info.origin.position.x + (mx + 0.5) * map_data_.info.resolution;
        p.y = map_data_.info.origin.position.y + (my + 0.5) * map_data_.info.resolution;
        p.z = 0.0;
        return p;
    }

    // 获取机器人当前位置
    bool getRobotPose(geometry_msgs::PoseStamped &robot_pose)
    {
        try
        {
            geometry_msgs::TransformStamped transformStamped = tf_buffer_.lookupTransform(map_frame_, robot_frame_, ros::Time(0));
            robot_pose.header.frame_id = map_frame_;
            robot_pose.header.stamp = ros::Time::now();
            robot_pose.pose.position.x = transformStamped.transform.translation.x;
            robot_pose.pose.position.y = transformStamped.transform.translation.y;
            robot_pose.pose.position.z = transformStamped.transform.translation.z;
            robot_pose.pose.orientation = transformStamped.transform.rotation;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(2.0, "%s", ex.what());
            return false;
        }
        return true;
    }
    // 获取目标点位置, 从全局路径中取前瞻距离的点作为目标点
    bool getGoalPosition(const geometry_msgs::Pose &robot_pose, Position &goal_pos)
    {
        if (global_path_.poses.empty())
        {
            return false;
        }
        double lookahead_distance = 3.0; // 前瞻距离可以调整
        double accumulated_distance = 0.0;
        for (size_t i = 0; i < global_path_.poses.size() - 1; ++i)
        {
            geometry_msgs::Pose current_pose = global_path_.poses[i].pose;
            geometry_msgs::Pose next_pose = global_path_.poses[i + 1].pose;
            double distance = std::sqrt(std::pow(next_pose.position.x - current_pose.position.x, 2) + std::pow(next_pose.position.y - current_pose.position.y, 2));
            accumulated_distance += distance;
            if (accumulated_distance >= lookahead_distance)
            {
                goal_pos = worldToMap(next_pose.position.x, next_pose.position.y);
                return true;
            }
        }
        // 如果全局路径长度不足前瞻距离，则以最后一个点作为目标点
        goal_pos = worldToMap(global_path_.poses.back().pose.position.x, global_path_.poses.back().pose.position.y);

        geometry_msgs::PoseStamped goal_point;
        goal_point.header.frame_id = map_frame_;
        goal_point.header.stamp = ros::Time::now();
        goal_point.pose.position.x = global_path_.poses.back().pose.position.x;
        goal_point.pose.position.y = global_path_.poses.back().pose.position.y;
        
        visualutils::pointMarkerMsgGenerator(goal_point, "goal", map_frame_);
        return true;
    }

    // A* 搜索算法
    std::vector<Position> aStarSearch(const Position &start, const Position &goal)
    {
        std::priority_queue<std::pair<double, Position>, std::vector<std::pair<double, Position>>, std::greater<std::pair<double, Position>>> open_set;
        std::map<Position, Position> came_from;
        std::map<Position, double> cost_so_far;

        open_set.push({0, start});
        cost_so_far[start] = 0;

        while (!open_set.empty())
        {
            Position current = open_set.top().second;
            open_set.pop();

            if (current == goal)
            {
                break; // 到达目标点
            }

            // 定义可能的移动方向，更大的步长
            int dx[] = {-step_size_, -step_size_, -step_size_, 0, 0, step_size_, step_size_, step_size_};
            int dy[] = {-step_size_, 0, step_size_, -step_size_, step_size_, -step_size_, 0, step_size_};
            // ROS_INFO("A* search...");

            for (int i = 0; i < 8; ++i)
            {
                Position next = {current.x + dx[i], current.y + dy[i]};

                if (isPositionValid(next) && !isOccupied(next))
                {
                    double new_cost = cost_so_far[current] + 1; // 简单假设每步代价为1

                    if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next])
                    {
                        cost_so_far[next] = new_cost;
                        double priority = new_cost + heuristic_scale_ * heuristic(next, goal);
                        open_set.push({priority, next});
                        came_from[next] = current;
                    }
                    // ROS_INFO("A* searching...");
                }
            }
        }
        // 重建路径
        std::vector<Position> path;
        Position current = goal;

        if (came_from.find(current) == came_from.end())
        {
            ROS_WARN("A* failed to find path!");
            return path;
        }
        while (!(current == start))
        {
            path.push_back(current);
            current = came_from[current];
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());

        return path;
    }
    // 检查位置是否在地图内
    bool isPositionValid(const Position &pos)
    {
        if (pos.x < 0 || pos.x >= map_data_.info.width ||
            pos.y < 0 || pos.y >= map_data_.info.height)
        {
            return false;
        }
        return true;
    }

    // 检查地图位置是否被占用
    bool isOccupied(const Position &pos)
    {
        int index = pos.y * map_data_.info.width + pos.x;
        if (index >= 0 && index < map_data_.data.size())
        {
            return map_data_.data[index] > 50; // 假设大于50的栅格为被占用
        }
        return true;
    }

    // 欧几里得距离启发式函数
    double heuristic(const Position &a, const Position &b)
    {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    // 发布路径Marker
    void publishPathMarker(const std::vector<Position> &path)
    {
        visualization_msgs::Marker line_strip;
        visualization_msgs::Marker points;
        line_strip.header.frame_id = map_frame_;
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "local_path";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = 0;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.scale.x = 0.1; // 线条宽度
        line_strip.color.r = 0.0f;
        line_strip.color.g = 1.0f;
        line_strip.color.b = 0.0f;
        line_strip.color.a = 1.0f;
        points.header = line_strip.header;
        points.ns = "local_path";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = 1;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.2;
        points.scale.y = 0.2;
        points.color.r = 1.0f;
        points.color.g = 0.0f;
        points.color.b = 0.0f;
        points.color.a = 1.0f;

        for (const auto &pos : path)
        {
            geometry_msgs::Point p = mapToWorld(pos.x, pos.y);
            line_strip.points.push_back(p);
            points.points.push_back(p);
        }

        path_marker_pub_.publish(line_strip);
        path_marker_pub_.publish(points);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Subscriber global_path_sub_;
    ros::Publisher path_marker_pub_;
    nav_msgs::OccupancyGrid map_data_;
    nav_msgs::Path global_path_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    double step_size_;
    double heuristic_scale_;
    double path_pub_rate_;
    ros::Timer timer_;
    std::string robot_frame_;
    std::string map_frame_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_generator_node");
    LocalPathPlanner planner;
    ros::spin();
    return 0;
}