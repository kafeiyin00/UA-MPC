#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "lidarprojection.hpp"

std::string lid_topic,rotorangle_topic;
bool bSimulation;
ros::Publisher pub_rotorspeed,pub_rotorframepoints;

std::shared_ptr<ROC::lidarprojector> lpr;

std::deque<ROC::rotorState> rotorStates;
std::deque<std::vector<ROC::Point3D>> buf_points;

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // ROS_INFO("11111");
    std::vector<ROC::Point3D> points;
    if (bSimulation)
    {
        pcl::PointCloud<pcl::PointXYZ> pts_orig, pts_trans;
        pcl::fromROSMsg(*msg, pts_orig);

        for (size_t i = 0; i < pts_orig.size(); i++)
        {
            pcl::PointXYZ pt = pts_orig.points[i];

            ROC::Point3D tmp_pt;
            tmp_pt.time = msg->header.stamp.toSec();
            tmp_pt.point = Eigen::Vector3d(pt.x, pt.y, pt.z);
            points.push_back(tmp_pt);
        }
        buf_points.push_back(points);

        while (buf_points.front().back().time < rotorStates.front().time)
        {
            buf_points.pop_front();
        }

        if (buf_points.front().back().time > rotorStates.back().time)
        {
            return;
        }

        std::vector<ROC::Point3D> points_trans;
        lpr->project_sensor2rotorframe(rotorStates, buf_points.front(), points_trans);
        buf_points.pop_front();
        ROS_INFO("receive points, %d", points_trans.size());
        for (size_t i = 0; i < points_trans.size(); i++)
        {
            pcl::PointXYZ pt;

            ROC::Point3D tmp_pt = points_trans[i];

            pt.x = tmp_pt.point(0);
            pt.y = tmp_pt.point(1);
            pt.z = tmp_pt.point(2);

            pts_trans.push_back(pt);
        }

        sensor_msgs::PointCloud2 rotorframepoints_msg;
        pcl::toROSMsg(pts_trans, rotorframepoints_msg);
        rotorframepoints_msg.header = msg->header;
        rotorframepoints_msg.header.frame_id = "rotor";
        pub_rotorframepoints.publish(rotorframepoints_msg);
        // pub for LO
    }
}

void rotorangle_cbk(const optimalcontroller::Float64Stamped::ConstPtr &msg)
{
    // ROS_INFO("2222");
    ROC::rotorState tmprs;
    tmprs.time = msg->header.stamp.toSec();
    tmprs.rotorangle = msg->data;
    rotorStates.push_back(tmprs);
    // ROS_INFO("receive rotor angle, %d", rotorStates.size());
    while (rotorStates.size() > 2000)
    {
        rotorStates.pop_front();
    }
}


int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "constantSpeedControl");
    ros::NodeHandle nh;

    nh.param<bool>("common/simulation", bSimulation, true);
    nh.param<std::string>("common/act_lid_topic", lid_topic, "/livox/lidar");
    nh.param<std::string>("common/rotorangle_topic", rotorangle_topic, "/livox/lidar");

    pub_rotorspeed = nh.advertise<std_msgs::Float32>("/rotorspeedcontrol", 10);
    pub_rotorframepoints = nh.advertise<sensor_msgs::PointCloud2>("/rotorframepoints", 10);

    // sub
    ros::Subscriber sub_pcl = nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_rotorangle = nh.subscribe(rotorangle_topic, 200000, rotorangle_cbk);

    lpr = std::make_shared<ROC::lidarprojector>(bSimulation);

    ros::Rate rate(100);

    while (ros::ok())
    {
        std_msgs::Float32 float_msg;
        float_msg.data = 1.8;  // You can change this value or modify it as needed

        // Publish the message
        // ROS_INFO("Publishing float data: %f", float_msg.data);
        pub_rotorspeed.publish(float_msg);

        // Wait until the next iteration
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}