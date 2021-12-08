#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>

class OdometryPub{

    public:
        OdometryPub();
        void run();
        void modelStateCallback(const gazebo_msgs::ModelStates & msg);

    private:
        ros::NodeHandle _nh;
        ros::Publisher _odom_pub; 
        ros::Subscriber _model_state_sub;
        tf::TransformBroadcaster _odom_tf_broadcast;

        double _x,_y,_z;
        double _dx,_dy;
        geometry_msgs::Quaternion _quat;

        ros::Time current_time, last_time;
        std::string _model_name = "dogbot";
};


OdometryPub::OdometryPub(){
    _odom_pub = _nh.advertise<nav_msgs::Odometry>("/odom",0);
    _model_state_sub = _nh.subscribe("/gazebo/model_states", 1, &OdometryPub::modelStateCallback, this);
    
}

void OdometryPub::modelStateCallback(const gazebo_msgs::ModelStates & msg) {

    bool found = false;
    int index = 0;
    while( !found  && index < msg.name.size() ) {

        if( msg.name[index] == _model_name )
            found = true;
        else index++;
    }

    _quat.x = msg.pose[index].orientation.x;
    _quat.y = msg.pose[index].orientation.y;
    _quat.z = msg.pose[index].orientation.z;
    _quat.w = msg.pose[index].orientation.w;
    
    _x = msg.pose[index].position.x;
    _y = msg.pose[index].position.y;
    _z = msg.pose[index].position.z;

    _dx=msg.twist[index].linear.x;
    _dy=msg.twist[index].linear.y;
    

}



void OdometryPub::run(){

    ros::Rate r(1000);

    while(ros::ok()){

        ros::spinOnce();
        current_time = ros::Time::now();

        geometry_msgs::TransformStamped odom_tf;
        if(last_time!=current_time){
            odom_tf.header.stamp = current_time;
            odom_tf.header.frame_id = "odom";
            odom_tf.child_frame_id = "base_link";

            odom_tf.transform.translation.x = _x;
            odom_tf.transform.translation.y = _y;
            odom_tf.transform.translation.z = _z;
            odom_tf.transform.rotation = _quat;
            _odom_tf_broadcast.sendTransform(odom_tf);

            nav_msgs::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";

            odom.pose.pose.position.x = _x;
            odom.pose.pose.position.y = _y;
            odom.pose.pose.position.z = _z;
            odom.pose.pose.orientation = _quat;

            odom.twist.twist.linear.x=_dx;
            odom.twist.twist.linear.y=_dy;
            _odom_pub.publish(odom);
            last_time = current_time;

        }
        
        r.sleep();


    }
}


int main(int argc, char** argv){

    ros::init(argc,argv, "odometry_node");
    OdometryPub odom_publish;
    odom_publish.run();

}
