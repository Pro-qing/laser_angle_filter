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
        // 1. 获取输入输出话题名参数 (带默认值)
        std::string scan_in_topic, scan_out_topic;
        nh_.param<std::string>("scan_in_topic", scan_in_topic, "/scan");
        nh_.param<std::string>("scan_out_topic", scan_out_topic, "/scan_filtered");

        // 2. 初始化动态调参服务器
        dyn_server_ = new dynamic_reconfigure::Server<laser_angle_filter::LaserFilterConfig>(nh_);
        dynamic_reconfigure::Server<laser_angle_filter::LaserFilterConfig>::CallbackType f;
        f = boost::bind(&LaserFilterNode::reconfigureCallback, this, _1, _2);
        dyn_server_->setCallback(f);

        // 3. 初始化订阅者和发布者
        sub_ = nh_.subscribe(scan_in_topic, 10, &LaserFilterNode::scanCallback, this);
        pub_ = nh_.advertise<sensor_msgs::LaserScan>(scan_out_topic, 10);

        ROS_INFO("Laser Filter Node Started!");
        ROS_INFO("Subscribing to: %s", scan_in_topic.c_str());
        ROS_INFO("Publishing to: %s", scan_out_topic.c_str());
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

    // 存储过滤角度 (弧度)
    double a1_min_rad_, a1_max_rad_;
    double a2_min_rad_, a2_max_rad_;

    // 角度转弧度
    double deg2rad(double deg) { return deg * M_PI / 180.0; }

    // 动态调参回调函数
    void reconfigureCallback(laser_angle_filter::LaserFilterConfig &config, uint32_t level)
    {
        // 将度数转换为弧度，并确保 min 小于 max
        a1_min_rad_ = deg2rad(std::min(config.angle1_min, config.angle1_max));
        a1_max_rad_ = deg2rad(std::max(config.angle1_min, config.angle1_max));
        
        a2_min_rad_ = deg2rad(std::min(config.angle2_min, config.angle2_max));
        a2_max_rad_ = deg2rad(std::max(config.angle2_min, config.angle2_max));

        ROS_INFO("Updated Filter Ranges (deg): Filter1[%.1f, %.1f], Filter2[%.1f, %.1f]", 
                 config.angle1_min, config.angle1_max, config.angle2_min, config.angle2_max);
    }

    // 雷达数据处理回调函数
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        // 复制一份雷达数据用于修改
        sensor_msgs::LaserScan filtered_scan = *msg;

        // 遍历所有激光束
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            // 计算当前激光点的角度
            double current_angle = msg->angle_min + i * msg->angle_increment;

            // 判断当前角度是否在两个过滤区间内
            bool in_filter1 = (current_angle >= a1_min_rad_ && current_angle <= a1_max_rad_);
            bool in_filter2 = (current_angle >= a2_min_rad_ && current_angle <= a2_max_rad_);

            if (in_filter1 || in_filter2)
            {
                // 如果在过滤区间内，将其距离设为无穷大（即过滤掉）
                filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
            }
        }

        // 发布过滤后的雷达数据
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