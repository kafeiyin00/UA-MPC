
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <vehicle_odom_player.hpp>
#include <tf2_ros/transform_broadcaster.h>


using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_generator");

    ros::NodeHandle n("~");

    tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped transform_uav;

    double init_x, init_y, init_z,mass;
    double simulation_rate;
    std::string traj_filepath;
    std::string quad_name;
    n.param("mass", mass, 0.9);
    n.param("init_state_x", init_x, 0.0);
    n.param("init_state_y", init_y, 0.0);
    n.param("init_state_z", init_z, 1.0);
    n.param("simulation_rate", simulation_rate, 200.0);
    n.param("quadrotor_name", quad_name, std::string("quadrotor"));
    n.param("traj_filepath", traj_filepath, std::string("traj_filepath"));

    vechicle_odom_player vop;
    vop.load_trajectory_from_csv(traj_filepath);

    ROS_INFO("----load csv trj size: %d", vop.waypoints.size());

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 10);

    ros::Rate rate(simulation_rate);
    rate.sleep();

    ros::Time last_time = ros::Time::now();
    ros::Time start_time = last_time;
    while(n.ok())
    {
        ros::spinOnce();

        ros::Time now_time = ros::Time::now();
        last_time = now_time;

        waypoint wpt = vop.getState((now_time.toSec()-start_time.toSec()));

        //publish odometry
        nav_msgs::Odometry odom;
        odom.header.frame_id = "world";
        odom.header.stamp = now_time;
        Vector3d pos,vel,acc,angular_vel,angular_vel_world;
        pos = wpt.p;
        // vel = quadrotor.getVel();
        // acc = quadrotor.getAcc();
        // angular_vel = quadrotor.getAngularVel();
        Vector4d quat(wpt.q.w(),wpt.q.x(),wpt.q.y(),wpt.q.z());
 
        Matrix3d R_body2world;
        angular_vel_world = R_body2world*angular_vel;
        // R_body2world = quadrotor.getR();
        odom.pose.pose.position.x = pos(0);
        odom.pose.pose.position.y = pos(1);
        odom.pose.pose.position.z = pos(2);
        odom.pose.pose.orientation.w = quat(0);
        odom.pose.pose.orientation.x = quat(1);
        odom.pose.pose.orientation.y = quat(2);
        odom.pose.pose.orientation.z = quat(3);
        odom.twist.twist.linear.x = vel(0);
        odom.twist.twist.linear.y = vel(1);
        odom.twist.twist.linear.z = vel(2);
        odom.twist.twist.angular.x = angular_vel_world(0);
        odom.twist.twist.angular.y = angular_vel_world(1);
        odom.twist.twist.angular.z = angular_vel_world(2);
        odom_pub.publish(odom);

        // ROS_INFO("Odom = %f,%f,%f, %f,%f,%f,%f",pos(0),pos(1),pos(2),quat(0),quat(1),quat(2),quat(3));

        //imu generate
        sensor_msgs::Imu imu_msg;
        imu_msg.header.frame_id = "/" + quad_name;
        imu_msg.header.stamp = now_time;
        imu_msg.orientation.w = quat(0);
        imu_msg.orientation.x = quat(1);
        imu_msg.orientation.y = quat(2);
        imu_msg.orientation.z = quat(3);
        imu_msg.angular_velocity.x = angular_vel(0);
        imu_msg.angular_velocity.y = angular_vel(1);
        imu_msg.angular_velocity.z = angular_vel(2);
        acc = R_body2world.inverse() * (acc + Eigen::Vector3d(0,0,-9.8));
        imu_msg.linear_acceleration.x = acc(0);
        imu_msg.linear_acceleration.y = acc(1);
        imu_msg.linear_acceleration.z = acc(2);
        imu_pub.publish(imu_msg);

        // transform
        transform_uav.header.frame_id = "world";  // 原始坐标系
        transform_uav.child_frame_id = "vehicle";   // 新的坐标系
        transform_uav.header.stamp = now_time;
        transform_uav.transform.translation.x = pos(0);  // 平移x轴
        transform_uav.transform.translation.y = pos(1);  // 平移y轴
        transform_uav.transform.translation.z = pos(2);  // 平移z轴
        transform_uav.transform.rotation.w = quat(0);     // 旋转
        transform_uav.transform.rotation.x = quat(1);     // 旋转
        transform_uav.transform.rotation.y = quat(2);     // 旋转
        transform_uav.transform.rotation.z = quat(3);     // 旋转
        tf_broadcaster.sendTransform(transform_uav);

        rate.sleep();
    }

    return 0;
  
}