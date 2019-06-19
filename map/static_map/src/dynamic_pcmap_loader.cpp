#include <ros/ros.h>
#include <ros/duration.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <list>

namespace dynamic_pc_map_loader
{
class App
{
private:
    ros::NodeHandle nh, pnh;
    ros::Subscriber sub_initial_pose, sub_current_pose;
    ros::Publisher pub_points_map;

    std::string param_piece_map_folder;
    double param_piece_map_step;
    double param_update_interval;

    ros::Time last_update_time;
    std::vector<std::string> areas;
    bool is_initialed;
    int before_x_index, before_y_index;
    int current_x_index, current_y_index;

    void initialPoseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &initial_pose);

    void currentPoseCB(const geometry_msgs::PoseStamped::ConstPtr &current_pose);

    void _init(const geometry_msgs::Pose pose);

    void _poseCB(const geometry_msgs::Pose pose);

    void _read_and_pub_pcmap(int x, int y);

public:
    App(ros::NodeHandle _nh, ros::NodeHandle _pnh) : nh(_nh), pnh(_pnh)
    {
        is_initialed = false;
    }
    void run();
};

void App::run()
{
    pnh.param<std::string>("piece_map_folder", param_piece_map_folder, "none");
    pnh.param<double>("piece_map_step", param_piece_map_step, 30.0);
    pnh.param<double>("update_interval", param_update_interval, 3.0);

    pub_points_map = nh.advertise<sensor_msgs::PointCloud2>("/dynamic_points_map", 1);
    sub_initial_pose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, boost::bind(&App::initialPoseCB, this, _1));
    sub_current_pose = nh.subscribe<geometry_msgs::PoseStamped>("/ndt/current_pose", 1, boost::bind(&App::currentPoseCB, this, _1));
    ros::spin();
}

void App::initialPoseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &initial_pose)
{
    geometry_msgs::Pose pose = initial_pose->pose.pose;
    current_x_index = int(pose.position.x / param_piece_map_step);
    current_y_index = int(pose.position.y / param_piece_map_step);
    if (!is_initialed)
    {
        _init(pose);
    }
    else
    {
        _poseCB(pose);
    }
}

void App::currentPoseCB(const geometry_msgs::PoseStampedConstPtr &current_pose)
{
    geometry_msgs::Pose pose = current_pose->pose;
    current_x_index = int(pose.position.x / param_piece_map_step);
    current_y_index = int(pose.position.y / param_piece_map_step);
    if (!is_initialed)
    {
        _init(pose);
    }
    else
    {
        _poseCB(pose);
    }
}

void App::_init(geometry_msgs::Pose pose)
{
    is_initialed = true;
    last_update_time = ros::Time::now();
    before_x_index = current_x_index;
    before_y_index = current_y_index;
    _read_and_pub_pcmap(current_x_index, current_y_index);
}

void App::_poseCB(geometry_msgs::Pose pose)
{
    ros::Time now = ros::Time::now();
    if ((now - last_update_time).toSec() < param_update_interval)
    {
        return;
    }
    if (current_x_index != before_x_index || current_y_index != before_y_index)
    {
        before_x_index = current_x_index;
        before_y_index = current_y_index;
        _read_and_pub_pcmap(current_x_index, current_y_index);
    }
}

void App::_read_and_pub_pcmap(int x, int y)
{
    ros::Time t1 = ros::Time::now();
    std::stringstream file_name, file_left, file_right, file_front, file_back, file_leftback, file_leftfront, file_rightback, file_rightfront;
    file_name << param_piece_map_folder << "x" << x << "y" << y << ".pcd";
    file_left << param_piece_map_folder << "x" << x - 1 << "y" << y << ".pcd";
    file_right << param_piece_map_folder << "x" << x + 1 << "y" << y << ".pcd";
    file_front << param_piece_map_folder << "x" << x << "y" << y + 1 << ".pcd";
    file_back << param_piece_map_folder << "x" << x << "y" << y - 1 << ".pcd";
    file_leftback << param_piece_map_folder << "x" << x - 1 << "y" << y - 1 << ".pcd";
    file_leftfront << param_piece_map_folder << "x" << x - 1 << "y" << y + 1 << ".pcd";
    file_rightback << param_piece_map_folder << "x" << x + 1 << "y" << y - 1 << ".pcd";
    file_rightfront << param_piece_map_folder << "x" << x + 1 << "y" << y + 1 << ".pcd";
    std::string files[] = {file_name.str(), file_left.str(), file_right.str(), file_front.str(), file_back.str(), file_leftback.str(), file_leftfront.str(), file_rightback.str(), file_rightfront.str()};

    pcl::PointCloud<pcl::PointXYZ> pcl_map;
    // std::cout << "current loc: (" << x << " , " << y << ")" << std::endl;
    for (auto file : files)
    {
        if (boost::filesystem::exists(file) == true)
        {
            pcl::PointCloud<pcl::PointXYZ> temp_pc;
            temp_pc.clear();
            pcl::io::loadPCDFile(file, temp_pc);
            pcl_map += temp_pc;
            // std::cout << "load in " << file << ", size: " << temp_pc.size() << std::endl;
        }
    }
    if (pcl_map.size() == 0)
    {
        ROS_ERROR("[dynamic_map_loader] No pcd map loaded!");
        return;
    }
    // std::cout << "total size = " << pcl_map.size() << std::endl;

    sensor_msgs::PointCloud2 msg_map;
    pcl::toROSMsg(pcl_map, msg_map);
    msg_map.header.frame_id = "map";
    pub_points_map.publish(msg_map);
    ros::Time t2 = ros::Time::now();
    // std::cout << "load multi pcd files in " << (t2 - t1).toSec() << " s" << std::endl;
}

}; // namespace dynamic_pc_map_loader

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamic_pcmap_loader");
    ros::NodeHandle nh, pnh("~");
    dynamic_pc_map_loader::App app = dynamic_pc_map_loader::App(nh, pnh);
    app.run();
    return 0;
}
