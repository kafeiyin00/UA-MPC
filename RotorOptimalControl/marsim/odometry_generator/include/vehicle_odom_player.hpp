#pragma once

#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <sstream>

struct waypoint{
    int num;
    double t;
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    Eigen::Vector3d v;
    Eigen::Vector3d a_b;
    Eigen::Vector3d w;

};

class vechicle_odom_player{
public:
    vechicle_odom_player()
    {
        start_time = -1;
    }

    void load_trajectory_from_csv(std::string filepath);

    waypoint getState(double time);


    std::vector<waypoint> waypoints;
    double start_time;
};

void vechicle_odom_player::load_trajectory_from_csv(std::string filepath)
{
    waypoints.clear();
    std::ifstream file(filepath);
    std::string line;

    // Check if the file is open
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file " << filepath << std::endl;
        return;
    }
     // Skip the first line (header)
    std::getline(file, line);

    // Read the file line by line
    while (std::getline(file, line)) {
        waypoint data;
        // std::cout<< line <<"\n";
        
        sscanf(line.c_str(),"%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &data.num, &data.t,
        &data.p.x(),&data.p.y(),&data.p.z(),
        &data.q.x(),&data.q.y(),&data.q.z(),&data.q.w());

        if(start_time < 0)
        {
            start_time = data.t;
        }
        data.t -= start_time;
        waypoints.push_back(data);
    }

    file.close();
}

waypoint vechicle_odom_player::getState(double time)
{
    if (time > waypoints.back().t)
    {
        waypoint compare_obj; compare_obj.t = time;
        compare_obj.p = waypoints.back().p;
        compare_obj.q.setIdentity();
        return compare_obj;
    }
    waypoint compare_obj; compare_obj.t = time;
    auto lower_waypoint = std::lower_bound(waypoints.begin(), waypoints.end(), compare_obj,
        [](const waypoint& wpt1, const waypoint& wpt2)->bool
        {
            return wpt1.t < wpt2.t;
        });

    // auto upper_waypoint = std::upper_bound(waypoints.begin(), waypoints.end(), compare_obj,
    //     [](const waypoint& wpt1, const waypoint& wpt2)->bool
    //     {
    //         return wpt1.t < wpt2.t;
    //     });

    // interpolation

    Eigen::Vector3d p1 = lower_waypoint->p;
    Eigen::Quaterniond q1 = lower_waypoint->q;
    double t1 = lower_waypoint->t;

    auto upper_waypoint = (lower_waypoint+1);
    if(upper_waypoint == waypoints.end())
    {
        compare_obj.p = p1;
        compare_obj.q = q1;
        compare_obj.q.setIdentity();

        return compare_obj;
    }

    

    Eigen::Vector3d p2 = upper_waypoint->p;
    Eigen::Quaterniond q2 = upper_waypoint->q;
    double t2 = upper_waypoint->t;

    compare_obj.p = p1 + (time - t1)/(t2 - t1)*(p2-p1);
    compare_obj.q = q1.slerp((time - t1)/(t2 - t1), q2);
    compare_obj.q.setIdentity();

    return compare_obj;
}




