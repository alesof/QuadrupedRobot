#include <ros/ros.h>
#include "boost/thread.hpp"
#include <iostream>
#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>
#include "sensor_msgs/LaserScan.h"
#include "tf/tf.h"

#include "project/service.h"
#include "math.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"

#define PI_2 1.570796327

class GlobalPlanner{

    public:
        GlobalPlanner();

        void set_goal(double, double);
        void odom_cb(const nav_msgs::Odometry&);
        void laser_cb(const sensor_msgs::LaserScan& laser);
        void marker_cb( std_msgs::String cam);
        void srv_routine();

        void compute_attractive();
        void compute_total_force();
        double get_yaw();

        bool check_local_min();

        Eigen::Vector2d get_position();
        Eigen::Vector2d get_velocity();

        void run();
        

    private:
        ros::NodeHandle _nh;
        ros::Subscriber _sub_odom;
        ros::Subscriber _sub_scan;
        ros::Subscriber _sub_cam;
        ros::Publisher _pub_vel;
        ros::Publisher _pub_map;

        double _x_goal, _y_goal;
        nav_msgs::Odometry _pose;
        Eigen::Vector2d _e;
        Eigen::Vector2d _gradient;

        Eigen::Vector2d _attractive_for;
        Eigen::Vector2d _repulsive_for;
        Eigen::Vector2d _total_force;

        double _attractive_pot;
        double _repulsive_pot;
        double _total_pot;


        double _ka;
        double _ka_close;
        double _kr;
        bool _obs_found;
        double _obs_distance;
        void loop();

        int _roomexplored;
        double _old_yaw;
        int _n_giri;
        bool _first_jpos;
        bool inizia_ad_esplorare;

        Eigen::Vector2d Vel;
        double _distanza_ostacoli_frontale;
        double _distanza_ostacoli_destra;
        std::string _marker_desiderato;
        bool _richiedi_mappa;
};

GlobalPlanner::GlobalPlanner(){
    _sub_odom = _nh.subscribe("/odom", 1, &GlobalPlanner::odom_cb, this);
    _sub_cam = _nh.subscribe("/opencv/marker_data", 1, &GlobalPlanner::marker_cb, this);
    _sub_scan = _nh.subscribe("/laser/scan",1,&GlobalPlanner::laser_cb,this); 
    _pub_vel = _nh.advertise<geometry_msgs::Vector3>("/dogbot/cmd_pos_towr", 1); 
    _pub_map = _nh.advertise<std_msgs::String>("/salvamappa", 1); 
    _x_goal = 0;
    _y_goal = 0;
    _roomexplored = 1;

    _ka = 10000;
    _ka_close =10000;
    _kr = 2000;
    _obs_found = false;
    _first_jpos = false;
    inizia_ad_esplorare = false;
    _obs_distance=0;
    _richiedi_mappa=false;
    

    _e= Eigen::Vector2d::Zero();
    _attractive_for= Eigen::Vector2d::Zero();
    _repulsive_for= Eigen::Vector2d::Zero();
    _total_force= Eigen::Vector2d::Zero();
    Vel = Eigen::Vector2d::Zero();

    _attractive_pot= 0;
    _repulsive_pot= 0;
    _total_pot= 0;

    _pose.pose.pose.position.x=2;
    _pose.pose.pose.position.y=1; // TODO valori iniziali hardcoded
    _distanza_ostacoli_frontale=2.5;
    _old_yaw=3.11; 
    _n_giri=0;

    _marker_desiderato="ID0101"; 
    /*
        ID1234 room1 
        ID2997 room2
        ID0101 room3 
    */
}

void GlobalPlanner::set_goal(double x, double y){
    _x_goal = x;
    _y_goal = y;
}

Eigen::Vector2d GlobalPlanner::get_position(){
    Eigen::Vector2d posizione_x_y;
    posizione_x_y << _pose.pose.pose.position.x , _pose.pose.pose.position.y;
    return posizione_x_y;
}

Eigen::Vector2d GlobalPlanner::get_velocity(){
    Eigen::Vector2d velocity;
    velocity << _pose.twist.twist.linear.x , _pose.twist.twist.linear.y;
    return velocity;
}

double GlobalPlanner::get_yaw(){
    double roll, pitch, yaw;
    double yaw_periodico;
    tf::Quaternion q(_pose.pose.pose.orientation.x,_pose.pose.pose.orientation.y,_pose.pose.pose.orientation.z,_pose.pose.pose.orientation.w);
    q.normalize();
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    if( yaw *_old_yaw<0 && abs(yaw)>3){
        if(yaw<_old_yaw){
            _n_giri++;
        }else{
            _n_giri--;
        }
    }
    yaw_periodico=yaw +_n_giri*6.28;
    _old_yaw=yaw;
    
    return yaw_periodico;
}

void GlobalPlanner::odom_cb(const nav_msgs::Odometry& od){
    _pose = od;
    _first_jpos=true;
}

void GlobalPlanner::marker_cb(std_msgs::String cam){
    if(cam.data[0]=='I' && cam.data[1]=='D' && inizia_ad_esplorare ){ // accetta valori di ID solo se stai esplorando

        if(!_marker_desiderato.compare(cam.data))_roomexplored=0;
        else _roomexplored++;

        inizia_ad_esplorare=false;
    }
}

void GlobalPlanner::laser_cb(const sensor_msgs::LaserScan& msg){

    sensor_msgs::LaserScan laser=msg;
    int start_ind = 0;
    int end_ind = laser.ranges.size();
    double temp_min=2.0;
    int gamma = 2;

    double yaw;
    double temp1,temp2;
    Eigen::Vector2d obstacle_pos;
    Eigen::Vector2d pos_robot;
    double theta = 0;
    double obs_view_dist=2.2;
    tf::Quaternion quaternione(_pose.pose.pose.orientation.x,_pose.pose.pose.orientation.y,_pose.pose.pose.orientation.z,_pose.pose.pose.orientation.w);
    tf::Matrix3x3 rot(quaternione);
    rot.getRPY(temp1,temp2,yaw);

    _obs_found = false;
    _repulsive_for << 0,0;

    int init_obst=0;
    int temp=0;
    int count_ray=0;
    int start_ind_temp=0;
    int end_ind_temp=0;

    // inizia da dopo il primo ostacolo per evitare problemi di discontinuità dovuti al vettore dei laser
    while(laser.ranges[start_ind]<obs_view_dist)start_ind++;

    // check corner 
    start_ind_temp=start_ind;
    end_ind_temp=end_ind;
    for(int i=start_ind_temp;i<end_ind_temp;i++){

        if(laser.ranges[i]<obs_view_dist){
            if(!_obs_found)init_obst=i;
            _obs_found=true;

            if(laser.ranges[i]>temp && i!=start_ind_temp &&i!=end_ind_temp){
                if(laser.ranges[i]>laser.ranges[i+1] && laser.ranges[i]>laser.ranges[i-1]){
                    temp=laser.ranges[i];
                    theta = i;
                }
            }

            count_ray++;
        }
        if(_obs_found &&(i != end_ind_temp-2 &&laser.ranges[i+1]>obs_view_dist ||i == end_ind_temp-1 && start_ind_temp==0 )){

            if(count_ray*laser.angle_increment>=2.09){//corner
                laser.ranges[theta]=obs_view_dist+10;
            }
            temp=0;
            _obs_found=false;
            count_ray=0;
        }

        if(i==end_ind_temp-1 && start_ind_temp!=0){ // questo serve per caso in cui ostacolo continua tra 0 e 360
            end_ind_temp=start_ind_temp-1;
            i=0;
            start_ind_temp=0;
        }

    }
    
    _obs_found=false;
    theta=0;
    //calcola repulsive force
    for(int i=start_ind;i<end_ind;i++){
       
        if(laser.ranges[i]<obs_view_dist){
            _obs_found=true;

            if(laser.ranges[i]<temp_min){
                temp_min=laser.ranges[i];
                theta = i*laser.angle_increment;
            }

        }

        if(_obs_found &&(i != end_ind-2 &&laser.ranges[i+1]>obs_view_dist ||i == end_ind-1 && start_ind==0 )){
                _obs_distance = temp_min;
                
                double x,y;
                x= -_obs_distance*cos(theta + yaw -PI_2);
                y= -_obs_distance*sin(theta + yaw -PI_2);
                _gradient<< x/sqrt(pow(x,2)+pow(y,2)),
                            y/sqrt(pow(x,2)+pow(y,2));
                
                _repulsive_pot += _kr/gamma*pow((1/_obs_distance -1/obs_view_dist),gamma);
                _repulsive_for += (_kr/pow(_obs_distance,2))*pow((1/_obs_distance -1/obs_view_dist),gamma-1)*_gradient; //compute repulsive
               
                temp_min=obs_view_dist;
                _obs_found=false;

        }

        if(i==end_ind-1 && start_ind!=0){ // questo serve per caso in cui ostacolo continua tra 0 e 360
            end_ind=start_ind-1;
            i=0;
            start_ind=0;
        }
    }
    _distanza_ostacoli_frontale=laser.ranges[0];
    _distanza_ostacoli_destra=laser.ranges[270];
}

bool GlobalPlanner::check_local_min(){

    _total_pot=_attractive_pot+_repulsive_pot;
    return (_total_force.norm()<1080 && _e.norm() >1 && _total_pot>0); 
}

void GlobalPlanner::compute_total_force(){
    _total_force = _repulsive_for+_attractive_for; // queste sono accelerazioni 
}

void GlobalPlanner::compute_attractive(){
    if (_e.norm()<=1.0 && _e!=Eigen::Vector2d::Zero()){
        _attractive_for = _ka_close * _e ;
        _attractive_pot= 0.5*_ka_close * pow(_e.norm(),2);
    }
    else{ 
        _attractive_for = _ka * _e/ _e.norm();
        _attractive_pot = _ka * _e.norm();
    }
}

void GlobalPlanner::loop(){
    double fr=400;
	ros::Rate r(fr);

    Eigen::Vector2d current_pos;
    Eigen::Vector2d current_vel;
    Eigen::Vector2d desired_pos;
    Eigen::Vector2d desired_pos_min_loc;
    
    geometry_msgs::Vector3 pos_des;
    pos_des.x=0;
    pos_des.y=0;
    pos_des.z=0;
    

    geometry_msgs::Twist q_dot;
    ros::Time t_start_min_routine;
    double k=0;
    bool girotondo=false;
    bool min_routine=false;
    bool passetti_dietro=false;
    int pareti_esplorate=0;
    double yaw_esplorativo=0;

    q_dot.linear.x=0;
    q_dot.linear.y=0;
    q_dot.linear.z=0;
    q_dot.angular.x=0;
    q_dot.angular.y=0;
    q_dot.angular.z=0;


    Vel<<0,0;


    while( !_first_jpos  )
        usleep(0.1*1e6);


    pos_des.x=_pose.pose.pose.position.x;
    pos_des.y=_pose.pose.pose.position.y;; 
    pos_des.z=_old_yaw;
    _pub_vel.publish(pos_des);

    while(ros::ok()){

        current_pos = get_position();
        current_vel = get_velocity();


        if(check_local_min() || min_routine||passetti_dietro){
            ROS_INFO_ONCE("WEWE condizione di minimo");
            if(!passetti_dietro && !min_routine ){
                ROS_INFO_ONCE("WEWE passetti");
                t_start_min_routine = ros::Time::now(); 
                yaw_esplorativo=get_yaw();
                desired_pos_min_loc<<current_pos[0]-cos(yaw_esplorativo-PI_2),current_pos[1]-sin(yaw_esplorativo-PI_2);
                passetti_dietro=true;
                ROS_INFO_ONCE("desired_pos_min_loc : %f,%f",desired_pos_min_loc[0],desired_pos_min_loc[1]);
            }
            else if(!min_routine && (ros::Time::now()-t_start_min_routine).toSec() >10){
                ROS_INFO_ONCE("WEWE minroutnie");
                t_start_min_routine = ros::Time::now(); 
                min_routine=true;
                passetti_dietro=false;
                desired_pos_min_loc[0]= (_x_goal -current_pos[0] >0)?-rand()%3 +  current_pos[0] : +rand()%3 +  current_pos[0] ;
                desired_pos_min_loc[1]= rand() %3 + 7;
                ROS_INFO_ONCE("desired_pos_min_loc : %f,%f",desired_pos_min_loc[0],desired_pos_min_loc[1]);
            }
            else if((ros::Time::now()-t_start_min_routine).toSec() >120 || _e.norm()<0.05 ){
                min_routine=false;
            }
            _e = desired_pos_min_loc - current_pos;
        }
        else if(inizia_ad_esplorare){
            if(_distanza_ostacoli_frontale>1.6 && !girotondo &&pareti_esplorate!=2){ // vai dritto e non uscire dalla porta quando giri due volte 
                _x_goal=current_pos[0]+0.5*cos(get_yaw()-PI_2);
                _y_goal=current_pos[1]+0.5*sin(get_yaw()-PI_2);
                yaw_esplorativo=get_yaw();
            }
            else if(_distanza_ostacoli_destra>1.6 && !girotondo){ // se non puoi andare dritto vai a destra a granchio
                _x_goal=current_pos[0]+0.5*cos(get_yaw()+PI_2*2);
                _y_goal=current_pos[1]+0.5*sin(get_yaw()+PI_2*2);
                yaw_esplorativo=get_yaw();
            }
            else if(!girotondo){ //se non puoi nemmeno andare a destra 
                _x_goal=current_pos[0];
                _y_goal=current_pos[1];
                yaw_esplorativo=get_yaw()-PI_2-0.2;//per compensare gli errori di towr nello girare, meglio che guarda piu verso l'interno della stanza
                girotondo=true;
                pareti_esplorate++;
            }
            desired_pos << _x_goal, _y_goal;
            _e = desired_pos - current_pos;
        }
        else{
            pareti_esplorate=0;
            desired_pos << _x_goal, _y_goal;
            _e = desired_pos - current_pos;
        }

        //Calcola forza attrattiva + forza repulsiva = forza totale
        compute_attractive();
        compute_total_force();

        if(isnan(_total_force[0])){ 
            Vel[0] = (_total_force[0]/fr );
            Vel[1] = (_total_force[1]/fr );
        }else{
            Vel[0] = current_vel[0] + (_total_force[0]/fr );
            Vel[1] = current_vel[1] + (_total_force[1]/fr );
        }

        q_dot.linear.x = Vel[0];
        q_dot.linear.y = Vel[1];
        
        //Saturazione sullo step
        if(abs(Vel[0])>fr*0.15){ 
            if(Vel[0]>0) Vel[0]=fr*0.15;
           else Vel[0]= - fr*0.15;
        }
        if(abs(Vel[1])>fr*0.15){
            if(Vel[1]>0) Vel[1]=fr*0.15;
           else Vel[1]= - fr*0.15;
        }

        if(!inizia_ad_esplorare && !passetti_dietro)pos_des.z=atan2(_total_force[1],_total_force[0]) +1.57 ;  
        else{
            pos_des.z=yaw_esplorativo;
        }

        // gira sul posto se yaw des >>> yaw
        if(abs(pos_des.z -get_yaw()) >= 0.2){
            if(!girotondo){
                pos_des.x=current_pos[0];
                pos_des.y=current_pos[1];
                girotondo=true;
            }
            pos_des.z=(pos_des.z>get_yaw())? get_yaw()+0.15 : get_yaw()-0.15;  
            
        }else{
            pos_des.x=current_pos[0] +Vel[0]*1/fr;
            pos_des.y=current_pos[1] +Vel[1]*1/fr;
            girotondo=false;

        }
        
  
        // if(_total_force.norm()<1500){
        // std::cout<<"\n_....................";
        //     std::cout<<"\n_attractive_for:"<< _attractive_for;
        //     std::cout<<"\n_repulsive_for:"<< _repulsive_for; 
        //     std::cout<<"\n_total_force :"<< _total_force;
        //     std::cout<<"\n_attractive_pot norm:"<< _attractive_for.norm();
        //     std::cout<<"\n_repulsive_pot norm:"<< _repulsive_for.norm();
        //     std::cout<<"\n_total_force norm:"<< _total_force.norm();
        // }
        // std::cout<<"\n_e:"<< _e;
        // std::cout<<"\n_attractive_for:"<< _attractive_for;
        // std::cout<<"\n_repulsive_for:"<< _repulsive_for; 
        // std::cout<<"\n_total_force :"<< _total_force;
        // std::cout<<"\n_attractive_pot norm:"<< _attractive_for.norm();
        // std::cout<<"\n_repulsive_pot norm:"<< _repulsive_for.norm();
        // std::cout<<"\n_total_force norm:"<< _total_force.norm();
        // std::cout<<"\ncurrent_vel :"<< current_vel;
        // std::cout<<"\nincr pos x:"<< Vel[0]*1/250;
        // std::cout<<"\nincr pos y:"<< Vel[1]*1/250;
        // std::cout<<"\nGET_yaw:"<< get_yaw();
        // std::cout<<"\npos_des.z:"<< pos_des.z;
        // if(_total_force.norm()<900) std::cout<<"\n_total_force :"<< _total_force.norm();

        //Publish cmd_vel calcolate con potenziali artificiali
        _pub_vel.publish(pos_des);
    
	r.sleep();
	}
}

void GlobalPlanner::srv_routine(){

    ros::Rate srvRate(1);
    ros::ServiceClient client = _nh.serviceClient<project::service>("service");
    project::service srv;
    Eigen::Vector2d position_reached;
    Eigen::Vector2d goal_pos;
    
    while(ros::ok()){
        position_reached = get_position();
        
     
        srv.request.in = _roomexplored;
        if(client.call(srv) && !inizia_ad_esplorare ){
            _x_goal = srv.response.goal_x;
            _y_goal = srv.response.goal_y;
        }

        goal_pos << _x_goal, _y_goal;

        if((position_reached-goal_pos).norm()<0.05){

            if( _roomexplored==0 && !_richiedi_mappa){
                std_msgs::String data;
                data.data="$";
                _pub_map.publish(data);
                _richiedi_mappa=true;
            }
            else if(!_richiedi_mappa) inizia_ad_esplorare=true;
        }
        srvRate.sleep();
    }
    
}
    

void GlobalPlanner::run() {
    int rate=1;
	boost::thread ctrl_loop_t ( &GlobalPlanner::loop, this);
    boost::thread srv_routine_t (&GlobalPlanner::srv_routine, this);
    
	ros::spin();
}



int main(int argc, char** argv){
    ros::init(argc, argv, "globalplanner");
    GlobalPlanner planner;

    planner.run();
    return 0;
}
