#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common.h>

// #include <pcl/point_types.h>
#include <pcl/io/io.h>
// #include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/impl/kdtree.hpp>
#include <ros/package.h>
#include <vector>

typedef pcl::PointXYZ PointT;

using namespace std;
string file_name;

int add_boundary = 0;
double downsample_res;
double map_offset_x,map_offset_y,map_offset_z;

int minus_twopointcloud(pcl::PointCloud<pcl::PointXYZ>& cloud_input, pcl::PointCloud<pcl::PointXYZ>& cloud_input2, pcl::PointCloud<pcl::PointXYZ>& cloud_output)
{
  //cloud_input minus cloud_input2
  pcl::search::KdTree<pcl::PointXYZ> minus_kdtree;
  minus_kdtree.setInputCloud(cloud_input2.makeShared());

  vector<int> pointIdxRadiusSearch;
  vector<float> pointRadiusSquaredDistance;
  for(int i = 0; i < cloud_input.points.size();i++)
  {
    if (minus_kdtree.radiusSearch(cloud_input.points[i], 0.6, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
      continue;
    }
    cloud_output.points.push_back(cloud_input.points[i]);
  }
  
  ROS_INFO("AFTER MINUS, points count = %d", cloud_output.points.size());

  return cloud_output.points.size();

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_recorder");
  ros::NodeHandle node("~");

  node.getParam("add_boundary", add_boundary);
  // node.getParam("is_bridge", is_bridge);
  node.getParam("downsample_res", downsample_res);

  node.getParam("map_offset_x", map_offset_x);
  node.getParam("map_offset_y", map_offset_y);
  node.getParam("map_offset_z", map_offset_z);

  ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 10, true);
  file_name = argv[1];

  ros::Duration(1.0).sleep();

  /* load cloud from pcd */
  pcl::PointCloud<pcl::PointXYZ> cloud_temp, cloud;
  int status = pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloud_temp);
  cloud = cloud_temp;

  if (status == -1)
  {
    cout << "can't read file." << endl;
    return -1;
  }


  ROS_INFO("SUCCESS LOAD PCD FILE");

  //filter
  pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
  _voxel_sampler.setLeafSize(downsample_res, downsample_res, downsample_res);
  _voxel_sampler.setInputCloud(cloud.makeShared());
  _voxel_sampler.filter(cloud);

  pcl::PointXYZ global_mapmin;
  pcl::PointXYZ global_mapmax;
  
  pcl::getMinMax3D(cloud,global_mapmin,global_mapmax);

  ROS_INFO("Map bound: x=%f,%f, y=%f,%f, z=%f,%f",global_mapmin.x,global_mapmax.x,global_mapmin.y,global_mapmax.y,global_mapmin.z,global_mapmax.z);

  if(add_boundary == 1)
  {
  // for (double x = -10; x <= 10; x += 0.05)
  //   for (double y = -10; y <= 10; y += 0.05)
  //   {
  //     cloud.push_back(pcl::PointXYZ(x, y, 0));
  //   }
  
  //add boundary
  pcl::PointCloud<pcl::PointXYZ> cloud_boundary;
  int add_length = 1;
  int max_x = (int)global_mapmax.x + add_length;
  int min_x = (int)global_mapmin.x - add_length;
  int max_y = (int)global_mapmax.y + add_length;
  int min_y = (int)global_mapmin.y - add_length;
  int max_z = (int)global_mapmax.z + add_length;
  int min_z = (int)global_mapmin.z - add_length;
  int box_x = (max_x - min_x)*10;
  int box_y = (max_y - min_y)*10;
  int box_z = (max_z - min_z)*10;
  //draw six plane
  cloud_boundary.width = (box_x+1) * (box_y+1) *(box_z+1) - (box_x-1) * (box_y-1) *(box_z-1);//points number
  cloud_boundary.height=1;
  cloud_boundary.points.resize(cloud_boundary.width*cloud_boundary.height);
  int point_count = 0;
  //draw 2 xy plane
  for (float i = min_x;i <= max_x;i = i+0.1)
  {
    for (float j = min_y;j <= max_y;j = j+0.1)
    {
      // cloud_boundary.points[point_count].x = i;
      // cloud_boundary.points[point_count].y = j;
      // cloud_boundary.points[point_count].z = min_z;
      // point_count++;
      // cloud_boundary.points[point_count].x = i;
      // cloud_boundary.points[point_count].y = j;
      // cloud_boundary.points[point_count].z = max_z;
      // point_count++;
      cloud.push_back(pcl::PointXYZ(i, j, min_z));
      cloud.push_back(pcl::PointXYZ(i, j, max_z));
    }
  }
  //draw 4 plane
  for (float k = min_z;k <= max_z;k = k+0.1)
  {
    //draw x two lines
    for (float i = min_x;i <= max_x;i = i+0.1)
    {
      cloud_boundary.points[point_count].x = i;
      cloud_boundary.points[point_count].y = min_y;
      cloud_boundary.points[point_count].z = k;
      point_count++;
      cloud_boundary.points[point_count].x = i;
      cloud_boundary.points[point_count].y = max_y;
      cloud_boundary.points[point_count].z = k;
      point_count++;
      cloud_boundary.points[point_count].x = i;
      cloud_boundary.points[point_count].y = min_y-0.1;
      cloud_boundary.points[point_count].z = k;
      point_count++;
      cloud_boundary.points[point_count].x = i;
      cloud_boundary.points[point_count].y = max_y+0.1;
      cloud_boundary.points[point_count].z = k;
      point_count++;
    }
    for (float j = min_y; j <= max_y; j =j+0.1)
    {
      cloud_boundary.points[point_count].x = min_x;
      cloud_boundary.points[point_count].y = j;
      cloud_boundary.points[point_count].z = k;
      point_count++;
      cloud_boundary.points[point_count].x = max_x;
      cloud_boundary.points[point_count].y = j;
      cloud_boundary.points[point_count].z = k;
      point_count++;
      cloud_boundary.points[point_count].x = min_x-0.1;
      cloud_boundary.points[point_count].y = j;
      cloud_boundary.points[point_count].z = k;
      point_count++;
      cloud_boundary.points[point_count].x = max_x+0.1;
      cloud_boundary.points[point_count].y = j;
      cloud_boundary.points[point_count].z = k;
      point_count++;
    }
  }

  cloud = cloud_boundary+cloud;

  ROS_INFO("ADD BOUNDARY!!!");
  }

  // pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
  _voxel_sampler.setLeafSize(downsample_res, downsample_res, downsample_res);
  _voxel_sampler.setInputCloud(cloud.makeShared());
  _voxel_sampler.filter(cloud);

  // Process map
  for (int i = 0; i < cloud.points.size(); ++i)
  {
    auto pt = cloud.points[i];
    pcl::PointXYZ pr;
    pr.x = pt.x + map_offset_x;
    pr.y = pt.y + map_offset_y;
    pr.z = pt.z + map_offset_z;
    cloud.points[i] = pr;
  }

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = "world";
  ROS_INFO("Map point size = %d", cloud.points.size());

  int count = 0;
  while (ros::ok())
  {
    ros::Duration(1.0).sleep();
    cloud_pub.publish(msg);
    // minus_cloud_pub.publish(minus_msg);
    ++count;
    if (count > 10)
    {
      break;
    }
  }
  cout << "finish publish map." << endl;

  return 0;
}