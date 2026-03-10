#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include <laser_angle_filter/LaserFilterConfig.h>
#include <cmath>
#include <limits>

class LaserFilterNode
{
public:
    LaserFilterNode() : nh_("~")
    {
        std::string scan_in_topic, scan_out_topic;
        nh_.param<std::string>("scan_in_topic", scan_in_topic, "/scan");
        nh_.param<std::string>("scan_out_topic", scan_out_topic, "/scan_filtered");

        dyn_server_ = new dynamic_reconfigure::Server<laser_angle_filter::LaserFilterConfig>(nh_);
        dynamic_reconfigure::Server<laser_angle_filter::LaserFilterConfig>::CallbackType f;
        f = boost::bind(&LaserFilterNode::reconfigureCallback, this, _1, _2);
        dyn_server_->setCallback(f);

        sub_ = nh_.subscribe(scan_in_topic, 10, &LaserFilterNode::scanCallback, this);
        pub_ = nh_.advertise<sensor_msgs::LaserScan>(scan_out_topic, 10);

        ROS_INFO("Laser Filter Node Started! Using 0-360 Degree Logic.");
    }

    ~LaserFilterNode()
    {
        delete dyn_server_;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    dynamic_reconfigure::Server<laser_angle_filter::LaserFilterConfig>* dyn_server_;

    double a1_min_rad_, a1_max_rad_;
    double a2_min_rad_, a2_max_rad_;

    double deg2rad(double deg) { return deg * M_PI / 180.0; }

    void reconfigureCallback(laser_angle_filter::LaserFilterConfig &config, uint32_t level)
    {
        // 这里的 min 和 max 不再强制排序，因为用户可能输入 min=350, max=10 (跨越0度)
        a1_min_rad_ = deg2rad(config.angle1_min);
        a1_max_rad_ = deg2rad(config.angle1_max);
        
        a2_min_rad_ = deg2rad(config.angle2_min);
        a2_max_rad_ = deg2rad(config.angle2_max);

        ROS_INFO("Filter1: [%.1f, %.1f], Filter2: [%.1f, %.1f]", 
                 config.angle1_min, config.angle1_max, config.angle2_min, config.angle2_max);
    }

    // 检查角度是否在过滤范围内 (兼容跨越0度线的情况)
    bool isAngleInFilter(double angle, double min_rad, double max_rad)
    {
        if (min_rad <= max_rad) {
            // 正常情况，例如 10度 到 90度
            return (angle >= min_rad && angle <= max_rad);
        } else {
            // 跨越 0度线情况，例如 350度 到 10度
            return (angle >= min_rad || angle <= max_rad);
        }
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        sensor_msgs::LaserScan filtered_scan = *msg;

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            double current_angle = msg->angle_min + i * msg->angle_increment;

            // 1. 将雷达发出的任意角度强制转换到 0 ~ 2π (即 0 ~ 360度) 之间
            current_angle = fmod(current_angle, 2.0 * M_PI);
            if (current_angle < 0.0) {
                current_angle += 2.0 * M_PI;
            }

            // 2. 检查过滤
            bool in_filter1 = isAngleInFilter(current_angle, a1_min_rad_, a1_max_rad_);
            bool in_filter2 = isAngleInFilter(current_angle, a2_min_rad_, a2_max_rad_);

            if (in_filter1 || in_filter2)
            {
                // 无效化点云
                filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
            }
        }

        pub_.publish(filtered_scan);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_filter_node");
    LaserFilterNode node;
    ros::spin();
    return 0;
}