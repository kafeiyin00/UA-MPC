#pragma once
#include "ros/ros.h"
#include "optimalcontroller/Float64Stamped.h"
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <deque>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

 

namespace ROC{
    struct Point3D
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d point;
        double time = 0.0;
        double intensity = 0.0;
        Point3D() = default;
    };

    struct rotorState{
        double time;
        double rotorangle;
    };

    class lidarprojector{
    public:
        lidarprojector(bool bsimulation):
            bsimulation_(bsimulation)
        {
            if (bsimulation_)
            {
                t_r_s.setZero();
                t_rf_r.setZero();
                R_r_s = Eigen::AngleAxisd(3.14/6,Eigen::Vector3d::UnitX()).toRotationMatrix();
            }
        }

        void project_sensor2rotorframe(const std::deque<rotorState>& rotorstates, const std::vector<Point3D>& sensor_points, std::vector<Point3D>& rotorframe_points);

        Eigen::Matrix3d R_r_s,R_rf_r;
        Eigen::Vector3d t_r_s,t_rf_r;
        bool bsimulation_; // without distortion
    };


    void lidarprojector::project_sensor2rotorframe(const std::deque<rotorState>& rotorstates, const std::vector<Point3D>& sensor_points, std::vector<Point3D>& rotorframe_points)
    {
        if(sensor_points[0].time<rotorstates.front().time || sensor_points.back().time>rotorstates.back().time)
        {
            ROS_ERROR("bad time assumption");
            return;
        }

        for (size_t i = 0; i < sensor_points.size(); i++)
        {
            Point3D tmpPt = sensor_points[i];
            rotorState rs_comp; rs_comp.time = tmpPt.time;
            // find nearest rotor states
            auto lower_bound_item = std::lower_bound(rotorstates.begin(),rotorstates.end(),rs_comp,
            [](rotorState a, rotorState b)->bool{
                return (a.time < b.time);
            });

            auto upper_bound_item = lower_bound_item+1;
            double current_angle = lower_bound_item->rotorangle + (rs_comp.time-lower_bound_item->time)*(upper_bound_item->rotorangle-lower_bound_item->rotorangle)/(upper_bound_item->time-lower_bound_item->time);

            R_rf_r = Eigen::AngleAxisd(current_angle,Eigen::Vector3d::UnitZ()).toRotationMatrix();

            // transform
            Point3D tmpPt_rf;
            tmpPt_rf.time = tmpPt.time;
            tmpPt_rf.point = R_rf_r*(R_r_s*tmpPt.point+t_r_s)+t_rf_r;
            rotorframe_points.push_back(tmpPt_rf);
        }
    }
}