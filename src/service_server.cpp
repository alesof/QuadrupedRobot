#include "ros/ros.h"
#include "project/service.h"
#include <iostream>
#include <sstream>

using namespace std;

const double room1[2]={2,11};
const double room2[2]={6,11};
const double room3[2]={6,3};
const double centrar[2]={4,6};
const double start[2]={2,2};
bool service_cb(project::service::Request &req, project::service::Response &res){



    if(req.in == 1){

        res.goal_x = room1[0];
        res.goal_y = room1[1];
    }
    else if(req.in == 2){
        res.goal_x = room3[0];
        res.goal_y = room3[1];
    }
    else if(req.in ==3) {
        res.goal_x = room2[0];
        res.goal_y = room2[1];
    }
    else{
        res.goal_x = start[0];
        res.goal_y = start[1];

    }

    ROS_DEBUG_ONCE("Value in:[%i] , Server response: [%f],[%f]",req.in,res.goal_x,res.goal_y);

    return true;
}

int main(int argc, char**argv){
    ros::init(argc,argv, "service_server");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("service", service_cb);
    ros::spin();
    return 0;
}
