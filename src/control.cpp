#include "iostream"
#include "opt.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/JointState.h"
#include <tf/tf.h>
#include "tf_conversions/tf_eigen.h"
#include "ros/ros.h"

#include <cstdlib>
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <ros/package.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "boost/thread.hpp"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include <std_srvs/Empty.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>
#include <towr/initialization/gait_generator.h>
#include <map>
#include <unistd.h>
#include <unordered_map>
#include "gazebo_msgs/ContactsState.h"

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/common/Plugin.hh>

#include "project/debug_msg.h"

#include "boost/thread.hpp"
#include "geometry_msgs/Vector3.h"
using namespace std;



class DOGCTRL {
    public:
        DOGCTRL();
        void jointStateCallback(const sensor_msgs::JointState & msg);
        void modelStateCallback(const gazebo_msgs::ModelStates & msg);
        void planner_cb(geometry_msgs::Vector3 msg);
        void run();
        void ctrl_loop();
        void publish_cmd(  Eigen::VectorXd tau  );
        void eebr_cb(gazebo_msgs::ContactsStateConstPtr eebr);
        void eebl_cb(gazebo_msgs::ContactsStateConstPtr eebl);
        void eefr_cb(gazebo_msgs::ContactsStateConstPtr eefr);
        void eefl_cb(gazebo_msgs::ContactsStateConstPtr eefl);
        void createrobot(std::string modelFile);
        // Compute the Jacobian
        void  computeJac();
        void  ComputeJaclinear();
        // Compute matrix transformation T needed to recompute matrices/vecotor after the coordinate transform to the CoM
        void computeTransformation(const Eigen::VectorXd &Vel_);
        void computeJacDotQDot();
        void computeJdqdCOMlinear();
        void update(Eigen::Matrix4d &eigenWorld_H_base, Eigen::Matrix<double,12,1> &eigenJointPos, Eigen::Matrix<double,12,1> &eigenJointVel, Eigen::Matrix<double,6,1> &eigenBasevel, Eigen::Vector3d &eigenGravity);			
        //Funzioni aggiunte 
        void trajectoryplanner();
        void momentum_observer();
        void W_CoM_des_calc(double t_now);
        void X_dot_dot_sw_calc(double t_now);
        void quadratic_problem_solver_stance();
        void quadratic_problem_solver_swing();
        void compute_dyn_J(Eigen::MatrixXd &Jst_com,Eigen::MatrixXd &Jst_j,Eigen::MatrixXd &Jsw_com,Eigen::MatrixXd &Jsw_j,Eigen::MatrixXd &Jstdot_qdot,Eigen::MatrixXd &Jswdot_qdot);
        void print_towr_trajectory();
        Eigen::Vector3d get_ee_pos(int id_foot);
        Eigen::Vector3d get_ee_vel(int id_foot);
        Eigen::Matrix3d  get_rot4Wcom();
        double get_twr_yaw( double actual_twr_yaw);

    private:
        ros::NodeHandle _nh;
        ros::Subscriber _joint_state_sub; 
        ros::Subscriber _model_state_sub; 
        ros::Subscriber _eebl_sub;
        ros::Subscriber _eebr_sub;
        ros::Subscriber _eefl_sub;
        ros::Subscriber _eefr_sub;
        ros::Subscriber _planner_sub;
        ros::Publisher  _joint_pub;
        ros::Publisher  _debug_pub;

        Eigen::Matrix4d _world_H_base;
        Eigen::Matrix<double,12,1> _jnt_pos; 
        Eigen::Matrix<double,12,1> _jnt_vel;
        Eigen::Matrix<double,6,1> _base_pos;
        Eigen::Matrix<double,6,1> _base_vel;

        string _model_name;
        OPT *_o; //ottimizzatore 
        OPT *_o_sw; //ottimizzatore 
        
        bool _first_wpose;
        bool _first_jpos;
        bool _first_cmd;
        unordered_map<int, string> _id2idname;  
        unordered_map<int, int> _id2index;      
        unordered_map<int, int> _index2id;      
        Eigen::VectorXd x_eigen;
       

        bool _contact_br;
        bool _contact_bl;
        bool _contact_fl;
        bool _contact_fr;


        unsigned int n; // int for DoFs number
        double robot_mass;  // Total mass of the robot
        iDynTree::KinDynComputations kinDynComp;    // KinDynComputations element
        iDynTree::Transform world_H_base;   // world to floating base transformation
        iDynTree::VectorDynSize jointPos;   // Joint position
        iDynTree::Twist         baseVel;    // Floating base velocity
        iDynTree::VectorDynSize jointVel;   // Joint velocity
        iDynTree::Vector3       gravity;    // Gravity acceleration

        iDynTree::VectorDynSize  qb;    // Position vector base+joints
        iDynTree::VectorDynSize  dqb;   // Velocity vector base+joints
        iDynTree::VectorDynSize  q;     // Position vector COM+joints
        iDynTree::VectorDynSize  dq;    // Velocity vector COM+joints
        iDynTree::VectorDynSize  qmin;  // Joints limit vector
        iDynTree::VectorDynSize  qmax;
        iDynTree::Vector6        CoM;      // Center of Mass Position
        iDynTree::Vector6        CoM_vel;  // Center of mass velocity

        iDynTree::FreeFloatingMassMatrix MassMatrix;    //Mass matrix
        iDynTree::VectorDynSize Bias;   //Bias Matrix
        iDynTree::MatrixDynSize GravMatrix; //Gravity Matrix
        iDynTree::MatrixDynSize Jac;    // Jacobian
        iDynTree::MatrixDynSize JacDot; // Jacobian derivative
        iDynTree::MatrixDynSize Jcom;   //CoM Jacobian
        iDynTree::MatrixDynSize Jdqd;   // Bias acceleration J_dot*q_dot
        iDynTree::MatrixDynSize T;      // Transformation Matrix
        iDynTree::MatrixDynSize T_inv_dot;  // Transformation matrix time derivative
        iDynTree::Model         model;  //Model
        iDynTree::ModelLoader   mdlLoader;

        iDynTree::FreeFloatingMassMatrix MassMatrixCOM; //Mass matrix in CoM representation
        iDynTree::VectorDynSize BiasCOM;                //Bias Matrix in CoM representation
        iDynTree::MatrixDynSize GravMatrixCOM;          //Gravity Matrix in CoM representation
        iDynTree::MatrixDynSize JacCOM;                 // Jacobian in CoM representation
        iDynTree::MatrixDynSize JacCOM_lin;             //Jacobian in CoM representation (only linear part)
        iDynTree::MatrixDynSize JdqdCOM;                // Bias acceleration J_dot*q_dot in CoM representation
        iDynTree::MatrixDynSize JdqdCOM_lin;    // Bias acceleration J_dot*q_dot in CoM representation

        //Towr Vars
        towr::SplineHolder solution;

        //Robot Vars
        int _nst; //desired_nst
        bool desired_ee_contact[4];
        // bool contact[4];      

        // Momentum Observer
        Eigen::Matrix<double,3,1> _fgr_bl;
        Eigen::Matrix<double,3,1> _fgr_br;
        Eigen::Matrix<double,3,1> _fgr_fl;
        Eigen::Matrix<double,3,1> _fgr_fr;
        Eigen::Matrix<double,12,1> _F_hat; 
        Eigen::Matrix<double,12,1> _f_hat; // stima dei momenti 
        Eigen::Matrix<double,12,1> _gamma_int; //

        // Control Var
        Eigen::Matrix<double,12,1> _tau;
        Eigen::Matrix<double,6,1> _W_CoM_des;
        Eigen::MatrixXd  _Xdotdot_sw_cmd;
        int _remap[4]={1,0,2,3};
        Eigen::Vector3d _planner_des;
        double _old_yaw;
        int _n_giri;
        double _yaw_periodico;
        double _old_yaw_twr;
        int _n_giri_twr;
        double _yaw_periodico_twr;

        Eigen::Matrix<double,6,1> _COM_pos_old;

        project::debug_msg demsg; // per stampare i grafici matlab

};


DOGCTRL::DOGCTRL() {
   
    _joint_state_sub = _nh.subscribe("/dogbot/joint_states", 1, &DOGCTRL::jointStateCallback, this);
    _model_state_sub = _nh.subscribe("/gazebo/model_states", 1, &DOGCTRL::modelStateCallback, this);
    _eebl_sub = _nh.subscribe("/back_left_contactsensor_state",1, &DOGCTRL::eebl_cb, this);
    _eefl_sub = _nh.subscribe("/front_left_contactsensor_state",1, &DOGCTRL::eefl_cb, this);
    _eebr_sub = _nh.subscribe("/back_right_contactsensor_state",1, &DOGCTRL::eebr_cb, this);
    _eefr_sub = _nh.subscribe("/front_right_contactsensor_state",1,&DOGCTRL::eefr_cb, this);
    _planner_sub = _nh.subscribe("/dogbot/cmd_pos_towr",1,&DOGCTRL::planner_cb, this);

    _joint_pub = _nh.advertise<std_msgs::Float64MultiArray>("/dogbot/joint_effort_controllers/command", 1);
    _debug_pub = _nh.advertise<project::debug_msg>("/dogbot/debug", 0);
    _model_name = "dogbot";

    _first_wpose = false;
    _first_jpos = false;
    _first_cmd = false;
    _contact_br = true; 
    _contact_bl = true; 
    _contact_fl = true; 
    _contact_fr = true;
    

    for(int i=0; i<4;i++)desired_ee_contact[i]=true;


    std::string path = ros::package::getPath("project");
    path += "/dogbot_description/urdf/dogbot.urdf";
    createrobot(path);

    model = kinDynComp.model();
	kinDynComp.setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);
	// Resize matrices of the class given the number of DOFs
    n = model.getNrOfDOFs();
    
    robot_mass = model.getTotalMass();  
    // robot_mass=robot_mass*9/10;//TODO
    jointPos = iDynTree::VectorDynSize(n);
    baseVel = iDynTree::Twist();
    jointVel = iDynTree::VectorDynSize(n);
	q = iDynTree::VectorDynSize(6+n);
	dq = iDynTree::VectorDynSize(6+n);
	qb = iDynTree::VectorDynSize(6+n);
	dqb=iDynTree::VectorDynSize(6+n);
	qmin= iDynTree::VectorDynSize(n);
	qmax= iDynTree::VectorDynSize(n);
	Bias=iDynTree::VectorDynSize(n+6);
	GravMatrix=iDynTree::MatrixDynSize(n+6,1);
    MassMatrix=iDynTree::FreeFloatingMassMatrix(model) ;
    Jcom=iDynTree::MatrixDynSize(3,6+n);
	Jac=iDynTree::MatrixDynSize(24,6+n);	
	JacDot=iDynTree::MatrixDynSize(24,6+n);
	Jdqd=iDynTree::MatrixDynSize(24,1);
    T=iDynTree::MatrixDynSize(6+n,6+n);
	T_inv_dot=iDynTree::MatrixDynSize(6+n,6+n);
    MassMatrixCOM=iDynTree::FreeFloatingMassMatrix(model) ;
    BiasCOM=iDynTree::VectorDynSize(n+6);
	GravMatrixCOM=iDynTree::MatrixDynSize(n+6,1);
	JacCOM=iDynTree::MatrixDynSize(24,6+n);
	JacCOM_lin=iDynTree::MatrixDynSize(12,6+n);
	JdqdCOM=iDynTree::MatrixDynSize(24,1);
	JdqdCOM_lin=iDynTree::MatrixDynSize(12,1);
	x_eigen= Eigen::VectorXd::Zero(30);
	_planner_des= Eigen::Vector3d::Zero();


    // _o = new OPT( 30,58,58); 

    // init per controllo
    _gamma_int=Eigen::Matrix<double,12,1>::Zero();
    _F_hat=Eigen::Matrix<double,12,1>::Zero();
    _f_hat=Eigen::Matrix<double,12,1>::Zero();
    _tau=Eigen::Matrix<double,12,1>::Zero();
    _fgr_bl=Eigen::Matrix<double,3,1>::Zero();
    _fgr_br=Eigen::Matrix<double,3,1>::Zero();
    _fgr_fl=Eigen::Matrix<double,3,1>::Zero();
    _fgr_fr=Eigen::Matrix<double,3,1>::Zero();
    _W_CoM_des=Eigen::Matrix<double,6,1>::Zero();
    _COM_pos_old=Eigen::Matrix<double,6,1>::Zero();
    
    _old_yaw=0; 
    _n_giri=0;
    _old_yaw_twr=0;
    _n_giri_twr=0;
    ROS_INFO("Dogcontrol creato");
}



void DOGCTRL::createrobot(std::string modelFile) {  
    
    if( !mdlLoader.loadModelFromFile(modelFile) ) {
        std::cerr << "KinDynComputationsWithEigen: impossible to load model from " << modelFile << std::endl;
        return ;
    }
    if( !kinDynComp.loadRobotModel(mdlLoader.model()) )
    {
        std::cerr << "KinDynComputationsWithEigen: impossible to load the following model in a KinDynComputations class:" << std::endl
                  << mdlLoader.model().toString() << std::endl;
        return ;
    }

    _id2idname.insert( pair< int, string > ( 0, kinDynComp.getDescriptionOfDegreeOfFreedom(0) ));
    _id2idname.insert( pair< int, string > ( 1, kinDynComp.getDescriptionOfDegreeOfFreedom(1) ));
    _id2idname.insert( pair< int, string > ( 2, kinDynComp.getDescriptionOfDegreeOfFreedom(2) ));
    _id2idname.insert( pair< int, string > ( 3, kinDynComp.getDescriptionOfDegreeOfFreedom(3) ));
    _id2idname.insert( pair< int, string > ( 4, kinDynComp.getDescriptionOfDegreeOfFreedom(4) ));
    _id2idname.insert( pair< int, string > ( 5, kinDynComp.getDescriptionOfDegreeOfFreedom(5) ));
    _id2idname.insert( pair< int, string > ( 6, kinDynComp.getDescriptionOfDegreeOfFreedom(6) ));
    _id2idname.insert( pair< int, string > ( 7, kinDynComp.getDescriptionOfDegreeOfFreedom(7) ));
    _id2idname.insert( pair< int, string > ( 8, kinDynComp.getDescriptionOfDegreeOfFreedom(8) ));
    _id2idname.insert( pair< int, string > ( 9, kinDynComp.getDescriptionOfDegreeOfFreedom(9) ));
    _id2idname.insert( pair< int, string > ( 10, kinDynComp.getDescriptionOfDegreeOfFreedom(10) ));
    _id2idname.insert( pair< int, string > ( 11, kinDynComp.getDescriptionOfDegreeOfFreedom(11) ));

}

// Compute matrix transformation T needed to recompute matrices/vector after the coordinate transform to the CoM
void DOGCTRL::computeTransformation(const Eigen::VectorXd &Vel_) {

    //Set ausiliary matrices
    iDynTree::MatrixDynSize Jb(6,6+n);
    iDynTree::MatrixDynSize Jbc(3,n);
    iDynTree::Vector3 xbc;
    iDynTree::MatrixDynSize xbc_hat(3,3);
    iDynTree::MatrixDynSize xbc_hat_dot(3,3);
    iDynTree::MatrixDynSize Jbc_dot(6,6+n);
    iDynTree::Vector3 xbo_dot;

    //Set ausiliary matrices
    iDynTree::Vector3 xbc_dot;

    // Compute T matrix
    // Get jacobians of the floating base and of the com
    kinDynComp.getFrameFreeFloatingJacobian(0,Jb);
    kinDynComp.getCenterOfMassJacobian(Jcom);

    // Compute jacobian Jbc=d(xc-xb)/dq used in matrix T
    toEigen(Jbc)<<toEigen(Jcom).block<3,12>(0,6)-toEigen(Jb).block<3,12>(0,6);

    // Get xb (floating base position) and xc ( com position)
    iDynTree::Position xb = world_H_base.getPosition();
    iDynTree::Position xc= kinDynComp.getCenterOfMassPosition();

    // Vector xcb=xc-xb
    toEigen(xbc)=toEigen(xc)-toEigen(xb);

    // Skew of xcb
    toEigen(xbc_hat)<<0, -toEigen(xbc)[2], toEigen(xbc)[1],
    toEigen(xbc)[2], 0, -toEigen(xbc)[0],                          
    -toEigen(xbc)[1], toEigen(xbc)[0], 0;

    Eigen::Matrix<double,6,6> X;
    X<<Eigen::MatrixXd::Identity(3,3), toEigen(xbc_hat).transpose(), 
    Eigen::MatrixXd::Zero(3,3), Eigen::MatrixXd::Identity(3,3);

    Eigen::MatrixXd Mb_Mj= toEigen(MassMatrix).block(0,0,6,6).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(toEigen(MassMatrix).block(0,6,6,12));
    Eigen::Matrix<double,6,12> Js=X*Mb_Mj;

    // Matrix T for the transformation
    toEigen(T)<<Eigen::MatrixXd::Identity(3,3), toEigen(xbc_hat).transpose(), Js.block(0,0,3,12),
    Eigen::MatrixXd::Zero(3,3), Eigen::MatrixXd::Identity(3,3), Js.block(3,0,3,12),
    Eigen::MatrixXd::Zero(12,3),  Eigen::MatrixXd::Zero(12,3), Eigen::MatrixXd::Identity(12,12);

    //Compute time derivative of T 
    // Compute derivative of xbc
    toEigen(xbc_dot)=toEigen(kinDynComp.getCenterOfMassVelocity())-toEigen(baseVel.getLinearVec3());
    Eigen::VectorXd  mdr=robot_mass*toEigen(xbc_dot);
    Eigen::Matrix<double,3,3> mdr_hat;
    mdr_hat<<0, -mdr[2], mdr[1],
    mdr[2], 0, -mdr[0],                          
    -mdr[1], mdr[0], 0;

    //Compute skew of xbc
    toEigen(xbc_hat_dot)<<0, -toEigen(xbc_dot)[2], toEigen(xbc_dot)[1],
    toEigen(xbc_dot)[2], 0, -toEigen(xbc_dot)[0],                          
    -toEigen(xbc_dot)[1], toEigen(xbc_dot)[0], 0;

    Eigen::Matrix<double,6,6> dX;
    dX<<Eigen::MatrixXd::Zero(3,3), toEigen(xbc_hat_dot).transpose(),
    Eigen::MatrixXd::Zero(3,6);
    // Time derivative of Jbc
    kinDynComp.getCentroidalAverageVelocityJacobian(Jbc_dot);

    Eigen::Matrix<double,6,6> dMb;
    dMb<<Eigen::MatrixXd::Zero(3,3), mdr_hat.transpose(),
    mdr_hat, Eigen::MatrixXd::Zero(3,3);

    Eigen::MatrixXd inv_dMb1=(toEigen(MassMatrix).block(0,0,6,6).transpose().bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(dMb.transpose())).transpose();
    Eigen::MatrixXd inv_dMb2=-(toEigen(MassMatrix).block(0,0,6,6).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve( inv_dMb1));

    Eigen::Matrix<double,6,12> dJs=dX*Mb_Mj+X*inv_dMb2*toEigen(MassMatrix).block(0,6,6,12);

    toEigen(T_inv_dot)<<Eigen::MatrixXd::Zero(3,3), toEigen(xbc_hat_dot), -dJs.block(0,0,3,12),
    Eigen::MatrixXd::Zero(15,18);




}

// Compute Jacobian
void  DOGCTRL::computeJac() {     

    //Set ausiliary matrices
    iDynTree::MatrixDynSize Jac1(6,6+n);
    iDynTree::MatrixDynSize Jac2(6,6+n);
    iDynTree::MatrixDynSize Jac3(6,6+n);
    iDynTree::MatrixDynSize Jac4(6,6+n);

    // Compute Jacobian for each leg

    // Jacobian for back left leg
    kinDynComp.getFrameFreeFloatingJacobian( kinDynComp.getFrameIndex("back_left_foot"),Jac1);
    // Jacobian for back right leg
    kinDynComp.getFrameFreeFloatingJacobian( kinDynComp.getFrameIndex("back_right_foot"), Jac2);
    // Jacobian for front left leg
    kinDynComp.getFrameFreeFloatingJacobian( kinDynComp.getFrameIndex("front_left_foot"), Jac3);

    // Jacobian for front right leg
    kinDynComp.getFrameFreeFloatingJacobian( kinDynComp.getFrameIndex("front_right_foot"), Jac4);

    // Full Jacobian
    toEigen(Jac)<<toEigen(Jac1), toEigen(Jac2), toEigen(Jac3), toEigen(Jac4); 
    
}

void DOGCTRL::ComputeJaclinear() {
    
  Eigen::Matrix<double,12,24> B;
  B<< Eigen::MatrixXd::Identity(3,3) , Eigen::MatrixXd::Zero(3,21),
      Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,15),
	  Eigen::MatrixXd::Zero(3,12), Eigen::MatrixXd::Identity(3,3),  Eigen::MatrixXd::Zero(3,9),
	  Eigen::MatrixXd::Zero(3,18), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3);

  toEigen(JacCOM_lin)=B*toEigen(JacCOM);
    
}

void DOGCTRL::computeJdqdCOMlinear()
{
	Eigen::Matrix<double,12,24> B;
    B<< Eigen::MatrixXd::Identity(3,3) , Eigen::MatrixXd::Zero(3,21),
      Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,15),
	  Eigen::MatrixXd::Zero(3,12), Eigen::MatrixXd::Identity(3,3),  Eigen::MatrixXd::Zero(3,9),
	  Eigen::MatrixXd::Zero(3,18), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3);


    toEigen(JdqdCOM_lin)= Eigen::MatrixXd::Zero(12,1);
    toEigen(JdqdCOM_lin)=B*toEigen(JdqdCOM);
	
}

// Compute Bias acceleration: J_dot*q_dot
void  DOGCTRL::computeJacDotQDot() {
    
    // Bias acceleration for back left leg
    iDynTree::Vector6 Jdqd1=kinDynComp.getFrameBiasAcc("back_left_foot"); 
    // Bias acceleration for back right leg
    iDynTree::Vector6 Jdqd2=kinDynComp.getFrameBiasAcc("back_right_foot"); 
    // Bias acceleration for front left leg
    iDynTree::Vector6 Jdqd3=kinDynComp.getFrameBiasAcc("front_left_foot"); 
    // Bias acceleration for front right leg
    iDynTree::Vector6 Jdqd4=kinDynComp.getFrameBiasAcc("front_right_foot"); 

    toEigen(Jdqd)<<toEigen(Jdqd1), toEigen(Jdqd2), toEigen(Jdqd3), toEigen(Jdqd4);

	
}

// Get joints position and velocity
void DOGCTRL::jointStateCallback(const sensor_msgs::JointState & msg) {

    if( _first_jpos == false ) {

        for( int i=0; i<12; i++) {
            bool found = false;
            int index = 0;
            while( !found && index <  msg.name.size() ) {
                if( msg.name[index] == _id2idname.at( i )    ) {
                    found = true;
                    _id2index.insert( pair< int, int > ( i, index ));
                    _index2id.insert( pair< int, int > ( index, i ));
                }
                else index++;
            }
        }
    }
   

    for( int i=0; i<12; i++ ) {
        _jnt_pos( i, 0) = msg.position[    _id2index.at(i)    ];
    }

    for( int i=0; i<12; i++ ) {
        _jnt_vel( i, 0) = msg.velocity[    _id2index.at(i)    ];
    }

    _first_jpos=true;

/*---unordered map 
front_right_roll_joint  id=0 index=11
front_left_roll_joint   id=1 index=8
back_right_roll_joint   id=2 index=5
back_left_roll_joint    id=3 index=2
back_left_pitch_joint   id=4 index=1
back_left_knee_joint    id=5 index=0
back_right_pitch_joint  id=6 index=4
back_right_knee_joint   id=7 index=3
front_left_pitch_joint  id=8 index=7
front_left_knee_joint   id=9 index=6
front_right_pitch_joint id=10 index=10
front_right_knee_joint  id=11 index=9
*/
}


// Get base position and velocity
void DOGCTRL::modelStateCallback(const gazebo_msgs::ModelStates & msg) {


    bool found = false;
    int index = 0;
    while( !found  && index < msg.name.size() ) {

        if( msg.name[index] == _model_name )
            found = true;
        else index++;
    }

    if( found ) {
        
        _world_H_base.setIdentity();
        
        //quaternion
        tf::Quaternion q(msg.pose[index].orientation.x, msg.pose[index].orientation.y, msg.pose[index].orientation.z,  msg.pose[index].orientation.w);
        q.normalize();
        Eigen::Matrix<double,3,3> rot;
        tf::matrixTFToEigen(tf::Matrix3x3(q),rot);

        //Roll, pitch, yaw
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        //Set base pos (position and orientation)
        _base_pos << msg.pose[index].position.x, msg.pose[index].position.y, msg.pose[index].position.z, roll, pitch, yaw;
        
        //Set transformation matrix
        _world_H_base.block(0,0,3,3)= rot;
        _world_H_base.block(0,3,3,1)= _base_pos.block(0,0,3,1);

        //Set base vel
        _base_vel << msg.twist[index].linear.x, msg.twist[index].linear.y, msg.twist[index].linear.z, msg.twist[index].angular.x, msg.twist[index].angular.y, msg.twist[index].angular.z;
        _first_wpose = true;
    }
}

void DOGCTRL::momentum_observer(){
    /*
        hp rho(0)=gamma(0)=0
        poiche fgr è 0 quando il piede non è in stance, hp di poter far il calcolo matriciale di alfa con la J intera
    */ 

    Eigen::Matrix<double,12,12> K;
    Eigen::Matrix<double,12,1> rho;
    Eigen::Matrix<double,12,1> fgr_tot;

    Eigen::MatrixXd qdot_inv = iDynTree::toEigen(dqb).completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd C =(iDynTree::toEigen(Bias) -iDynTree::toEigen(GravMatrix)) * qdot_inv;
    Eigen::MatrixXd C_sign= iDynTree::toEigen(T).transpose().inverse()*C*iDynTree::toEigen(T).inverse()+iDynTree::toEigen(T).transpose().inverse()*iDynTree::toEigen(MassMatrix)*iDynTree::toEigen(T_inv_dot); //C riportata al centro di massa
    Eigen::MatrixXd Ctqdot=C_sign.transpose()*iDynTree::toEigen(dqb);

    fgr_tot<<  iDynTree::toEigen(kinDynComp.getWorldTransform(8).getRotation())*_fgr_bl,
                iDynTree::toEigen(kinDynComp.getWorldTransform(11).getRotation())*_fgr_br,
                iDynTree::toEigen(kinDynComp.getWorldTransform(14).getRotation())*_fgr_fl,
                iDynTree::toEigen(kinDynComp.getWorldTransform(17).getRotation())*_fgr_fr;

    K= Eigen::Matrix<double,12,12>::Identity(); // Valore preso da teoria 
    rho=  iDynTree::toEigen(MassMatrixCOM).block(6,6,12,12) *iDynTree::toEigen(dqb).block(6,0,12,1); 

    Eigen::MatrixXd alpha= Ctqdot.block(6,0,12,1) + _tau + iDynTree::toEigen(JacCOM_lin).block(0,6,12,12).transpose()* fgr_tot;
    
    double freq = 0.001;
    _gamma_int += (_F_hat + alpha)*freq;
    _F_hat= K*(rho -_gamma_int);
    _f_hat= iDynTree::toEigen(JacCOM_lin).block(0,6,12,12).transpose().inverse()*_F_hat;

}

Eigen::Matrix3d DOGCTRL::get_rot4Wcom(){
    double roll = _base_pos[3];
    double pitch= _base_pos[4];
    double yaw  = _base_pos[5];
   

    if( yaw *_old_yaw<0 && abs(yaw)>1.57){
        if(yaw<_old_yaw){
            _n_giri++;
        }else{
            _n_giri--;
        }
    }
    _yaw_periodico=yaw +_n_giri*6.28;
    _old_yaw=yaw;

    tf::Quaternion myQuaternion;
    myQuaternion.setRPY( roll, pitch, _yaw_periodico );
    myQuaternion.normalize();
    Eigen::Matrix<double,3,3> rot;
    tf::matrixTFToEigen(tf::Matrix3x3(myQuaternion),rot);

    return rot;

}

double DOGCTRL::get_twr_yaw( double actual_twr_yaw){

    if( actual_twr_yaw *_old_yaw_twr<0 && abs(actual_twr_yaw)>1.57){
        if(actual_twr_yaw<_old_yaw_twr){
            _n_giri_twr++;
        }else{
            _n_giri_twr--;
        }
    }
    _yaw_periodico_twr=actual_twr_yaw +_n_giri_twr*6.28;
    _old_yaw_twr=actual_twr_yaw;
    return _yaw_periodico_twr;
}






void DOGCTRL::W_CoM_des_calc(double t_now){

    Eigen::Matrix<double,6,1> CoM_des_pos;
    Eigen::Matrix<double,6,1> CoM_des_vel;
    Eigen::Matrix<double,6,1> CoM_des_acc;
    Eigen::Matrix<double,6,1> g_temp;
    Eigen::Matrix<double,6,6> Kp =3500*Eigen::MatrixXd::Identity(6,6); 
    Eigen::Matrix<double,6,6> Kd =50*Eigen::MatrixXd::Identity(6,6);
    Eigen::Matrix<double,3,3> ROT =get_rot4Wcom();
    Eigen::Matrix<double,3,1> err_lin;
    Eigen::Matrix<double,3,1> err_ang;
    Eigen::Matrix<double,6,1> e_pos;
    Eigen::Matrix<double,6,1> e_vel;

    g_temp << 0,0,+9.81,0,0,0; 
    CoM_des_pos << solution.base_linear_->GetPoint(t_now).p() , solution.base_angular_->GetPoint(t_now).p();
    CoM_des_vel << solution.base_linear_->GetPoint(t_now).v() , solution.base_angular_->GetPoint(t_now).v();
    CoM_des_acc << solution.base_linear_->GetPoint(t_now).a() , solution.base_angular_->GetPoint(t_now).a();

    Eigen::Matrix<double,6,1> temp_com=iDynTree::toEigen(CoM);
    temp_com[5]=_yaw_periodico;
    err_ang= ROT*(CoM_des_pos.block(3,0,3,1) - temp_com.block(3,0,3,1));

    err_lin= CoM_des_pos.block(0,0,3,1) - iDynTree::toEigen(CoM).block(0,0,3,1);
    e_pos<<err_lin,err_ang;
    e_vel=CoM_des_vel - iDynTree::toEigen(CoM_vel);

    _W_CoM_des=  Kp*e_pos + Kd*e_vel+ robot_mass*g_temp + iDynTree::toEigen(MassMatrixCOM).block(0,0,6,6)*CoM_des_acc;

   // per creare grafici su matlab
    demsg.error_pos_x = e_pos[0];
    demsg.error_pos_y = e_pos[1];
    demsg.error_pos_z = e_pos[2];
    demsg.error_pos_r = e_pos[3];
    demsg.error_pos_p = e_pos[4];
    demsg.error_pos_yaw = e_pos[5];

    demsg.error_vel_x = e_vel[0];
    demsg.error_vel_y = e_vel[1];
    demsg.error_vel_z = e_vel[2];
    demsg.error_vel_r = e_vel[3];
    demsg.error_vel_p = e_vel[4];
    demsg.error_vel_yaw = e_vel[5];

    if(fabs(e_pos(0,0))>2){ //check errori dovuti ad idyntree
        cout<<"\n err_lin"<< err_lin;
        cout<<"\n CoM_des_pos.block(0,0,3,1)"<< CoM_des_pos.block(0,0,3,1);
        cout<<"\n e_pos"<< iDynTree::toEigen(CoM).block(0,0,3,1);

        abort();
    }
    
}

void DOGCTRL::X_dot_dot_sw_calc(double t_now ){
    Eigen::MatrixXd x_sw;
    Eigen::MatrixXd x_sw_dot;
    Eigen::MatrixXd x_sw_dot_dot;
    Eigen::MatrixXd e_pos;
    Eigen::MatrixXd e_vel;
    Eigen::MatrixXd Kp;
    Eigen::MatrixXd Kd;

  
    Eigen::MatrixXd Jst;
    Eigen::MatrixXd Jsw;
    Eigen::MatrixXd f_hat_sw;
    Eigen::Matrix<double,18,18> P;
    Eigen::Matrix<double,18,18> Mc;

    x_sw.resize(3*(4-_nst),1);
    x_sw_dot.resize(3*(4-_nst),1);
    x_sw_dot_dot.resize(3*(4-_nst),1);
    e_pos.resize(3*(4-_nst),1);
    e_vel.resize(3*(4-_nst),1);
    Kp.resize(3*(4-_nst),3*(4-_nst));
    Kd.resize(3*(4-_nst),3*(4-_nst));
    _Xdotdot_sw_cmd.resize(3*(4-_nst),1);

    Jst.resize(3*_nst,18);
    Jsw.resize(3*(4-_nst),18);
    f_hat_sw.resize(3*(4-_nst),1);

    x_sw=Eigen::MatrixXd::Zero(3*(4-_nst),1);
    x_sw_dot=Eigen::MatrixXd::Zero(3*(4-_nst),1);
    x_sw_dot_dot=Eigen::MatrixXd::Zero(3*(4-_nst),1);
    e_pos=Eigen::MatrixXd::Zero(3*(4-_nst),1);
    e_vel=Eigen::MatrixXd::Zero(3*(4-_nst),1);
    Kp =1000*Eigen::MatrixXd::Identity(3*(4-_nst),3*(4-_nst)); 
    Kd =100*Eigen::MatrixXd::Identity(3*(4-_nst),3*(4-_nst));
    _Xdotdot_sw_cmd =Eigen::MatrixXd::Zero(3*(4-_nst),1);

    Jst<<Eigen::MatrixXd::Zero(3*_nst,18);
    Jsw<<Eigen::MatrixXd::Zero(3*(4-_nst),18);
    f_hat_sw<<Eigen::MatrixXd::Zero(3*(4-_nst),1);
    
    //compute j 
    int riemp=0;
    int riemp_sw=0;
    int j=0;
    for (int i=0; i<4;i++){ 
        if(i==0)j=3;
        else if(i==1)j=0;
        else j=i*3;
        if(desired_ee_contact[i]){
            Jst.block(riemp*3,0,3,18)+= iDynTree::toEigen(JacCOM_lin).block(j,0,3,18);
            riemp++;
        }else{
            Jsw.block(riemp_sw*3,0,3,18)+= iDynTree::toEigen(JacCOM_lin).block(j,0,3,18);
            riemp_sw++;
        }
    }
    P=Eigen::Matrix<double,18,18>::Identity()-Jst.transpose()*(Jst*Jst.transpose()).inverse()*Jst;
    Mc=P*iDynTree::toEigen(MassMatrixCOM)+Eigen::Matrix<double,18,18>::Identity() -P; 

    int temp=0;
    for (int i = 0; i < 4; i++){ 
        if( !desired_ee_contact[i] ){
            x_sw.block(3*temp,0,3,1) = solution.ee_motion_.at(_remap[i])->GetPoint(t_now).p();
            x_sw_dot.block(3*temp,0,3,1) = solution.ee_motion_.at(_remap[i])->GetPoint(t_now).v();
            x_sw_dot_dot.block(3*temp,0,3,1) = solution.ee_motion_.at(_remap[i])->GetPoint(t_now).a();
            
            e_pos.block(3*temp,0,3,1)=x_sw.block(3*temp,0,3,1)- get_ee_pos(_remap[i]);
            e_vel.block(3*temp,0,3,1)=x_sw_dot.block(3*temp,0,3,1)- get_ee_vel(_remap[i]);

            switch(i){
                case 0:
                    demsg.ee_br_err_pos_x = e_pos(0+3*temp,0);
                    demsg.ee_br_err_pos_y = e_pos(1+3*temp,0);
                    demsg.ee_br_err_pos_z = e_pos(2+3*temp,0);
                    demsg.ee_br_err_vel_x = e_vel(0+3*temp,0);
                    demsg.ee_br_err_vel_y = e_vel(1+3*temp,0);
                    demsg.ee_br_err_vel_z = e_vel(2+3*temp,0);
                    break;
                case 1:
                    demsg.ee_bl_err_pos_x = e_pos(0+3*temp,0);
                    demsg.ee_bl_err_pos_y = e_pos(1+3*temp,0);
                    demsg.ee_bl_err_pos_z = e_pos(2+3*temp,0);
                    demsg.ee_bl_err_vel_x = e_vel(0+3*temp,0);
                    demsg.ee_bl_err_vel_y = e_vel(1+3*temp,0);
                    demsg.ee_bl_err_vel_z = e_vel(2+3*temp,0);
                    break;
                case 2:
                    demsg.ee_fl_err_pos_x = e_pos(0+3*temp,0);
                    demsg.ee_fl_err_pos_y = e_pos(1+3*temp,0);
                    demsg.ee_fl_err_pos_z = e_pos(2+3*temp,0);
                    demsg.ee_fl_err_vel_x = e_vel(0+3*temp,0);
                    demsg.ee_fl_err_vel_y = e_vel(1+3*temp,0);
                    demsg.ee_fl_err_vel_z = e_vel(2+3*temp,0);
                    break;
                case 3:
                    demsg.ee_fr_err_pos_x = e_pos(0+3*temp,0);
                    demsg.ee_fr_err_pos_y = e_pos(1+3*temp,0);
                    demsg.ee_fr_err_pos_z = e_pos(2+3*temp,0);
                    demsg.ee_fr_err_vel_x = e_vel(0+3*temp,0);
                    demsg.ee_fr_err_vel_y = e_vel(1+3*temp,0);
                    demsg.ee_fr_err_vel_z = e_vel(2+3*temp,0);
                    break;
            }
            temp++;
        }else{
            switch(i){
                case 0:
                    demsg.ee_br_err_pos_x = 0;
                    demsg.ee_br_err_pos_y = 0;
                    demsg.ee_br_err_pos_z = 0;
                    demsg.ee_br_err_vel_x = 0;
                    demsg.ee_br_err_vel_y = 0;
                    demsg.ee_br_err_vel_z = 0;
                    break;
                case 1:
                    demsg.ee_bl_err_pos_x = 0;
                    demsg.ee_bl_err_pos_y = 0;
                    demsg.ee_bl_err_pos_z = 0;
                    demsg.ee_bl_err_vel_x = 0;
                    demsg.ee_bl_err_vel_y = 0;
                    demsg.ee_bl_err_vel_z = 0;
                    break;
                case 2:
                    demsg.ee_fl_err_pos_x = 0;
                    demsg.ee_fl_err_pos_y = 0;
                    demsg.ee_fl_err_pos_z = 0;
                    demsg.ee_fl_err_vel_x = 0;
                    demsg.ee_fl_err_vel_y = 0;
                    demsg.ee_fl_err_vel_z = 0;
                    break;
                case 3:
                    demsg.ee_fr_err_pos_x = 0;
                    demsg.ee_fr_err_pos_y = 0;
                    demsg.ee_fr_err_pos_z = 0;
                    demsg.ee_fr_err_vel_x = 0;
                    demsg.ee_fr_err_vel_y = 0;
                    demsg.ee_fr_err_vel_z = 0;
                    break;
                
            }
        }
    }

    riemp=0;
    j=0;
    for( int i=0; i<4;i++){
        if(i==0)j=3;
        else if(i==1)j=0;
        else j=i*3;
        if(!desired_ee_contact[i]){
            f_hat_sw.block(3*riemp,0,3,1)=_f_hat.block(j,0,3,1);
            riemp++;
        }
    }

    _Xdotdot_sw_cmd=x_sw_dot_dot+ Kd*e_vel + Kp*e_pos-Jsw*Mc.inverse()*P*Jsw.transpose()*f_hat_sw;

}


//Update elements of the class given the new state
void DOGCTRL::update (Eigen::Matrix4d &eigenWorld_H_base, Eigen::Matrix<double,12,1> &eigenJointPos, Eigen::Matrix<double,12,1> &eigenJointVel, Eigen::Matrix<double,6,1> &eigenBasevel, Eigen::Vector3d &eigenGravity)
{   
    // Update joints, base and gravity from inputs
    iDynTree::fromEigen(world_H_base,eigenWorld_H_base);
    iDynTree::toEigen(jointPos) = eigenJointPos;
    iDynTree::fromEigen(baseVel,eigenBasevel);
    toEigen(jointVel) = eigenJointVel;
    toEigen(gravity)  = eigenGravity;

    Eigen::Vector3d worldeigen=toEigen(world_H_base.getPosition()); // caso in cui idynntree restituisca valori nulli
    kinDynComp.setRobotState(world_H_base,jointPos,baseVel,jointVel,gravity);
    Eigen::Vector2d coord=toEigen(kinDynComp.getCenterOfMassPosition()).block(0,0,2,1);

    while (worldeigen==Eigen::Vector3d::Zero()|| coord.norm()<0.1){
        iDynTree::fromEigen(world_H_base,eigenWorld_H_base);
        worldeigen=toEigen(world_H_base.getPosition());
        //Set the state for the robot 
        kinDynComp.setRobotState(world_H_base,jointPos,baseVel,jointVel,gravity);
        coord=toEigen(kinDynComp.getCenterOfMassPosition()).block(0,0,2,1);
    }

    //Compute Center of Mass
    iDynTree::Vector3 base_angle=world_H_base.getRotation().asRPY();   
    toEigen(CoM)<<  toEigen(kinDynComp.getCenterOfMassPosition()),
                    toEigen(base_angle);

	//Compute velocity of the center of mass
	toEigen(CoM_vel)<<toEigen(kinDynComp.getCenterOfMassVelocity()), eigenBasevel.block(3,0,3,1);
		   
    // Compute position base +joints
	toEigen(qb)<<toEigen(world_H_base.getPosition()), toEigen(base_angle), eigenJointPos;
    // Compute position COM+joints
	toEigen(q)<<toEigen(CoM), eigenJointPos;
   	toEigen(dq)<<toEigen(CoM_vel), eigenJointVel;
	toEigen(dqb) << eigenBasevel, eigenJointVel;
   
	// Joint limits
    toEigen(qmin)<< -1.75 , -1.75,-1.75,-1.75,-1.58, -2.62, -3.15, -0.02,  -1.58, -2.62, -3.15, -0.02;
    toEigen(qmax)<< 1.75, 1.75, 1.75, 1.75, 3.15, 0.02, 1.58, 2.62,  3.15, 0.02, 1.58, 2.62;

    // Get mass, bias (C(q,v)*v+g(q)) and gravity (g(q)) matrices
    //Initialize ausiliary vector
    iDynTree::FreeFloatingGeneralizedTorques bias_force(model);
    iDynTree::FreeFloatingGeneralizedTorques grav_force(model);
    //Compute Mass Matrix
    kinDynComp.getFreeFloatingMassMatrix(MassMatrix); 
    //Compute Coriolis + gravitational tedums (Bias)
    kinDynComp.generalizedBiasForces(bias_force);
    toEigen(Bias)<<iDynTree::toEigen(bias_force.baseWrench()),
        iDynTree::toEigen(bias_force.jointTorques());

    
    //Compute Gravitational term
    kinDynComp.generalizedGravityForces(grav_force);
    toEigen(GravMatrix)<<iDynTree::toEigen(grav_force.baseWrench()),
            iDynTree::toEigen(grav_force.jointTorques());

    computeJac();	
    // Compute Bias Acceleration -> J_dot*q_dot
    computeJacDotQDot();
    
    Eigen::Matrix<double, 18,1> q_dot;

    q_dot<< eigenBasevel,
            eigenJointVel;

    // Compute Matrix needed for transformation from floating base representation to CoM representation
    computeTransformation(q_dot);
    // Compute Mass Matrix in CoM representation 
    toEigen(MassMatrixCOM)=toEigen(T).transpose().inverse()*toEigen(MassMatrix)*toEigen(T).inverse();
    // Compute Coriolis+gravitational term in CoM representation
    toEigen(BiasCOM)=toEigen(T).transpose().inverse()*toEigen(Bias)+toEigen(T).transpose().inverse()*toEigen(MassMatrix)*toEigen(T_inv_dot)*toEigen(dq);

    // Compute gravitational term in CoM representation	
    toEigen(GravMatrixCOM)=toEigen(T).transpose().inverse()*toEigen(GravMatrix);
    // Compute Jacobian term in CoM representation
    toEigen(JacCOM)=toEigen(Jac)*toEigen(T).inverse();
    ComputeJaclinear();
    // Compute Bias Acceleration -> J_dot*q_dot  in CoM representation
    toEigen(JdqdCOM)=toEigen(Jdqd)+toEigen(Jac)*toEigen(T_inv_dot)*toEigen(dq);
    computeJdqdCOMlinear();	
}

void  DOGCTRL::publish_cmd(  Eigen::VectorXd tau  ) {
    std_msgs::Float64MultiArray tau1_msg;
    // // Fill Command message
    for(int i=11; i>=0; i--) {
        tau1_msg.data.push_back(  tau( _index2id.at(i) )    );
    }
    //Sending command
    _joint_pub.publish(tau1_msg);
    /*
    front_right_roll_joint  id=0 index=11
    front_right_pitch_joint id=10 index=10
    front_right_knee_joint  id=11 index=9
    front_left_roll_joint   id=1 index=8
    front_left_pitch_joint  id=8 index=7
    front_left_knee_joint   id=9 index=6
    back_right_roll_joint   id=2 index=5
    back_right_pitch_joint  id=6 index=4
    back_right_knee_joint   id=7 index=3
    back_left_roll_joint    id=3 index=2
    back_left_pitch_joint   id=4 index=1
    back_left_knee_joint    id=5 index=0

    
*/

}

void DOGCTRL::planner_cb(geometry_msgs::Vector3 msg){
    _planner_des<<msg.x,msg.y,msg.z;
    _first_cmd=true;
}

void DOGCTRL::eebr_cb(gazebo_msgs::ContactsStateConstPtr eebr){
	if(eebr->states.empty()){ 
        _contact_br= false;
	}
	else {
		_contact_br= true;
        _fgr_br<< eebr->states[0].total_wrench.force.x ,eebr->states[0].total_wrench.force.y ,eebr->states[0].total_wrench.force.z;
	}
}

void DOGCTRL::eefl_cb(gazebo_msgs::ContactsStateConstPtr eefl){

	if(eefl->states.empty()){ 
        _contact_fl= false;
	}
	else {
		_contact_fl= true;
        _fgr_fl<< eefl->states[0].total_wrench.force.x ,eefl->states[0].total_wrench.force.y ,eefl->states[0].total_wrench.force.z;
    }
}

void DOGCTRL::eebl_cb(gazebo_msgs::ContactsStateConstPtr eebl){

	if(eebl->states.empty()){ 
        _contact_bl= false;
        
	}
	else {
	    _contact_bl= true;
        _fgr_bl<< eebl->states[0].total_wrench.force.x ,eebl->states[0].total_wrench.force.y ,eebl->states[0].total_wrench.force.z;
    }
}

void DOGCTRL::eefr_cb(gazebo_msgs::ContactsStateConstPtr eefr){
	if(eefr->states.empty()){ 
        _contact_fr= false;
	}
	else {
		_contact_fr= true;
        _fgr_fr<< eefr->states[0].total_wrench.force.x ,eefr->states[0].total_wrench.force.y ,eefr->states[0].total_wrench.force.z;
	}
}

Eigen::Vector3d DOGCTRL::get_ee_pos(int id_foot){
    iDynTree::Transform  World_ee_tf;
    switch(id_foot){
        case 0 :
            World_ee_tf=kinDynComp.getWorldTransform("back_left_foot");
            break;
        case 1 :
            World_ee_tf=kinDynComp.getWorldTransform("back_right_foot"); 
            break;
        case 2 :
            World_ee_tf=kinDynComp.getWorldTransform("front_left_foot");
            break;
        case 3 :
            World_ee_tf=kinDynComp.getWorldTransform("front_right_foot");
            break;

    }
        
	return toEigen(World_ee_tf.getPosition());
}

Eigen::Vector3d DOGCTRL::get_ee_vel(int id_foot){
    iDynTree::Twist ee_twist;
    switch(id_foot){
        case 0 :
            ee_twist=kinDynComp.getFrameVel("back_left_foot");
            break;
        case 1 :
            ee_twist=kinDynComp.getFrameVel("back_right_foot"); 
            break;
        case 2 :
            ee_twist=kinDynComp.getFrameVel("front_left_foot");
            break;
        case 3 :
            ee_twist=kinDynComp.getFrameVel("front_right_foot");
            break;

    }
        
	return toEigen(ee_twist.getLinearVec3());
}

void DOGCTRL::trajectoryplanner(){

    towr::NlpFormulation formulation;
    ifopt::Problem nlp;

    auto gategen = towr::GaitGenerator::MakeGaitGenerator(4);
    Eigen::Matrix<double,3,1> bl, br, fl, fr;


    //Model vars
    formulation.terrain_ = std::make_shared<towr::FlatGround>(0.0); 
    formulation.model_ = towr::RobotModel(towr::RobotModel::Dogbot);

    //Foot states
    bl << get_ee_pos(0);
    br << get_ee_pos(1);
    fl << get_ee_pos(2);
    fr << get_ee_pos(3);

    // //Initial pos and vel
    formulation.initial_base_.lin.at(towr::kPos) << toEigen(CoM).block(0,0,3,1);
    Eigen::Matrix<double,3,1> temp_com_ang=toEigen(CoM).block(3,0,3,1);
    temp_com_ang[2]= get_twr_yaw(temp_com_ang[2]);
    formulation.initial_base_.ang.at(towr::kPos) << temp_com_ang; 

    formulation.initial_base_.lin.at(towr::kVel) << toEigen(CoM_vel).block(0,0,3,1);
    formulation.initial_base_.ang.at(towr::kVel) <<toEigen(CoM_vel).block(3,0,3,1);

    //Definisco size del vettore per posizione degli "end effector"/gambe e gli passo i nominali
    auto stance = formulation.model_.kinematic_model_->GetNominalStanceInBase();
    formulation.initial_ee_W_ = stance;

    //Definisco valori iniziali posizione piedi    
    formulation.initial_ee_W_.at(0)[0] = bl[0]; // in accordo alla formulazione del dogbot model di towr
    formulation.initial_ee_W_.at(0)[1] = bl[1];
    formulation.initial_ee_W_.at(0)[2] = 0.0;
    formulation.initial_ee_W_.at(1)[0] = br[0];
    formulation.initial_ee_W_.at(1)[1] = br[1];
    formulation.initial_ee_W_.at(1)[2] = 0.0;
    formulation.initial_ee_W_.at(2)[0] = fl[0];
    formulation.initial_ee_W_.at(2)[1] = fl[1];
    formulation.initial_ee_W_.at(2)[2] = 0.0;
    formulation.initial_ee_W_.at(3)[0] = fr[0];
    formulation.initial_ee_W_.at(3)[1] = fr[1];
    formulation.initial_ee_W_.at(3)[2] = 0.0;
    
    if(!_first_cmd ){
        _planner_des[0]=formulation.initial_base_.lin.at(towr::kPos)[0];
        _planner_des[1]=formulation.initial_base_.lin.at(towr::kPos)[1];
        _planner_des[2]=formulation.initial_base_.ang.at(towr::kPos)[2];
    }

    std::cout<<"\n-----------------------";
    cout<<"\nformulation.initial_base_.lin.at(towr::kPos)[0]"<<formulation.initial_base_.lin.at(towr::kPos)[0];
    cout<<"\nformulation.initial_base_.lin.at(towr::kPos)[1]"<<formulation.initial_base_.lin.at(towr::kPos)[1];
    cout<<"\nformulation.initial_base_.lin.at(towr::kPos)[2]"<<formulation.initial_base_.lin.at(towr::kPos)[2];
    cout<<"\nformulation.initial_base_.ang.at(towr::kPos)[2]"<<formulation.initial_base_.ang.at(towr::kPos)[2];
    cout<<"\nformulation.final_base_x"<<_planner_des[0];
    cout<<"\nformulation.final_base_y"<<_planner_des[1];
    cout<<"\nformulation.final_base_yaw"<<_planner_des[2];

    // Set della posa finale da raggiungere
    formulation.final_base_.lin.at(towr::kPos) << _planner_des[0], _planner_des[1],0.40229; 
    formulation.final_base_.ang.at(towr::kPos) << 0.0 , 0.0 , _planner_des[2];

    // auto gatetype = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C6);// Gait implementati in quadruped_gait_generator.cc libreria di towr
    auto gatetype = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C1);// Gait implementati in quadruped_gait_generator.cc libreria di towr
    gategen->SetCombo(gatetype);
    
    double time= 1; //1sec
    formulation.params_.ee_phase_durations_.clear();
    for(int i=0;i<4;++i){
    formulation.params_.ee_phase_durations_.push_back(gategen->GetPhaseDurations(time,i)); 
    formulation.params_.ee_in_contact_at_start_.push_back(gategen->IsInContactAtStart(i));
    }

    //Init nonlinear programming with vars, constraints and costs
    for(auto c: formulation.GetVariableSets(solution))
        nlp.AddVariableSet(c);
    for(auto c: formulation.GetConstraints(solution))
        nlp.AddConstraintSet(c);
    for(auto c: formulation.GetCosts())
        nlp.AddCostSet(c);

    auto solver = std::make_shared<ifopt::IpoptSolver>();
    solver -> SetOption("jacobian_approximation","exact"); // "finite difference-values"
    solver -> SetOption("max_cpu_time",20.0);
    solver -> Solve(nlp);
}
 
void DOGCTRL::print_towr_trajectory(){

    std::cout<< "\n------------\nTraiettoria:\n------------\n";

    double temp=0.0;

    while(temp<=solution.base_linear_->GetTotalTime()+1e-5){
        std::cout<<"t=" <<temp <<"\n";
        std::cout<<"Posizione lineare della base x,y,z : \t";
        std::cout<<solution.base_linear_->GetPoint(temp).p().transpose() <<"\t[m] \n";

        std::cout<<"Roll Pitch Yaw Base: \t";
        Eigen::Vector3d rad = solution.base_angular_->GetPoint(temp).p();
        std::cout<< (rad/M_PI*180).transpose() << "\t[deg]\n";

        bool contact_bl = solution.phase_durations_.at(0)->IsContactPhase(temp);
        bool contact_br = solution.phase_durations_.at(1)->IsContactPhase(temp);
        bool contact_fl = solution.phase_durations_.at(2)->IsContactPhase(temp);
        bool contact_fr = solution.phase_durations_.at(3)->IsContactPhase(temp);

        std::string bl_foot_in_contact = contact_bl ? "yes" : "no" ;
        std::string br_foot_in_contact = contact_br ? "yes" : "no" ;
        std::string fl_foot_in_contact = contact_fl ? "yes" : "no" ;
        std::string fr_foot_in_contact = contact_fr ? "yes" : "no" ;
        std::cout<<" BL \t BR \t FL \t FR \n\n ";
        std::cout<< "Feet in contact: \t" +bl_foot_in_contact <<"\t" +br_foot_in_contact <<"\t" +fl_foot_in_contact <<"\t" +fr_foot_in_contact <<"\n\n";
        temp+=0.1;
    }

}

void DOGCTRL::quadratic_problem_solver_stance(){
    
    _o=new OPT(30,58,58);
    // --- Setting Q e c di Alglib ---
    int riemp_fhat=0;
    Eigen::Matrix<double, 12, 6> Jst_com= iDynTree::toEigen(JacCOM_lin).block(0,0,12,6);
    Eigen::Matrix<double, 12, 12> Jst_j= iDynTree::toEigen(JacCOM_lin).block(0,6,12,12);
    Eigen::Matrix<double, 12, 30> Sigma = Eigen::Matrix<double,12,30>::Zero();
    Eigen::Matrix<double,6,30>  T_al;
    Eigen::Matrix<double,30,30>  Q_al;
    Eigen::Matrix<double,6,6>  W_al;
    Eigen::Matrix<double,30,1>  c_al;
    Eigen::Matrix<double,12,1>  f_hat_remap;
    Eigen::Matrix<double,30,30>  R;

    Sigma.block(0,18,12,12) = Eigen::Matrix<double,12,12>::Identity();
    T_al = Jst_com.transpose()*Sigma; 
    W_al = 50*Eigen::Matrix<double,6,6>::Identity(); 
    R=Eigen::Matrix<double,30,30>::Identity();
    Q_al = T_al.transpose()*W_al*T_al+ R; 

    f_hat_remap.block(0,0,3,1)=_f_hat.block(3,0,3,1);
    f_hat_remap.block(3,0,3,1)=_f_hat.block(0,0,3,1);
    f_hat_remap.block(6,0,3,1)=_f_hat.block(6,0,3,1);
    f_hat_remap.block(9,0,3,1)=_f_hat.block(9,0,3,1);

    c_al << -T_al.transpose()*W_al.transpose()*( _W_CoM_des - Jst_com.transpose()*f_hat_remap); 

    _o->setQ(Q_al);
    _o->setc(c_al);

    //------ Costruzione matrici L e Lt
    double mi = 0.5; // Cono di frizione cambiato per diminuire forza tangenziale
    Eigen::Matrix<double,58, 31> L_stance= Eigen::Matrix<double,58,31>::Zero();
    Eigen::Matrix<double,18, 30> A_stance= Eigen::Matrix<double,18,30>::Zero();
    Eigen::Matrix<double,18, 1 > b_stance= Eigen::Matrix<double,18,1>::Zero();
    Eigen::Matrix<double,40, 30> D_stance= Eigen::Matrix<double,40,30>::Zero();
    Eigen::Matrix<double,40, 1 > c_stance= Eigen::Matrix<double,40,1>::Zero();
    Eigen::Matrix<double,16, 12 > Dfr= Eigen::Matrix<double,16,12>::Zero();
    Eigen::Matrix<double,4,3> cone;
    Eigen::Vector3d n, t1, t2;
    Eigen::Matrix<double,12,1> tau_max =  60*Eigen::Matrix<double,12,1>::Ones(); 
    Eigen::Matrix<double,12,1> tau_min = -60*Eigen::Matrix<double,12,1>::Ones(); 

    A_stance<<  iDynTree::toEigen(MassMatrixCOM).block(0,0,6,6), Eigen::Matrix<double,6,12>::Zero(),-Jst_com.transpose(),
                Jst_com, Jst_j, Eigen::Matrix<double,12,12>::Zero();

    b_stance<<  -iDynTree::toEigen(BiasCOM).block(0,0,6,1),
                -iDynTree::toEigen(JdqdCOM_lin);

    n<<0,0,1;
    t1<<0,1,0;
    t2<<1,0,0;
    cone<<  (t1 - mi*n).transpose(),
            -(t1 + mi*n).transpose(),
            (t2 - mi*n).transpose(),
            -(t2 + mi*n).transpose();
    for( int i=0; i<4;i++)Dfr.block(4*i,3*i,4,3)=cone;

    D_stance<<  Eigen::Matrix<double,16,18>::Zero(),Dfr,
                Eigen::Matrix<double,12,6>::Zero(),iDynTree::toEigen(MassMatrixCOM).block(6,6,12,12),-Jst_j.transpose(),
                Eigen::Matrix<double,12,6>::Zero(),-iDynTree::toEigen(MassMatrixCOM).block(6,6,12,12),Jst_j.transpose();
    
    c_stance<<  Eigen::Matrix<double,16,1>::Zero(),
                tau_max -iDynTree::toEigen(BiasCOM).block(6,0,12,1),
                -(tau_min -iDynTree::toEigen(BiasCOM).block(6,0,12,1));

    L_stance<<  A_stance,b_stance,
                D_stance,c_stance;
    
    _o->setL_stance(L_stance); // all'interno della funzione assegna anche Lt automaticamente
    _o->opt_stance(x_eigen);


    _tau =iDynTree::toEigen(MassMatrixCOM).block(6,6,12,12)*x_eigen.block(6,0,12,1) + iDynTree::toEigen(BiasCOM).block(6,0,12,1) - Jst_j.transpose()*x_eigen.block(18,0,12,1);

    if(desired_ee_contact[0]){
        demsg.fgr_star1 = x_eigen(21,0);
        demsg.fgr_star2 = x_eigen(22,0);
        demsg.fgr_star3 = x_eigen(23,0);
        demsg.fgr_star4 = _fgr_br(0,0);
        demsg.fgr_star5 = _fgr_br(1,0);
        demsg.fgr_star6 = _fgr_br(2,0);
    }else{
        demsg.fgr_star1 = 0;
        demsg.fgr_star2 = 0;
        demsg.fgr_star3 = 0;
        demsg.fgr_star4 = _fgr_br(0,0);
        demsg.fgr_star5 = _fgr_br(1,0);
        demsg.fgr_star6 = _fgr_br(2,0);
    }
    demsg.stance = int(desired_ee_contact[0]);




    delete _o;
}

void DOGCTRL::compute_dyn_J(Eigen::MatrixXd &Jst_com,Eigen::MatrixXd &Jst_j,Eigen::MatrixXd &Jsw_com,Eigen::MatrixXd &Jsw_j,Eigen::MatrixXd &Jstdot_qdot,Eigen::MatrixXd &Jswdot_qdot){
    int riemp=0;
    int riemp_sw=0;
    int j=0; 
    for (int i=0; i<4;i++){ 
        if(i==0)j=3;
        else if(i==1)j=0;
        else j=i*3;
        if(desired_ee_contact[i]){
            Jst_com.block(riemp*3,0,3,6)+= iDynTree::toEigen(JacCOM_lin).block(j,0,3,6);
            Jst_j.block(riemp*3,0,3,12)  += iDynTree::toEigen(JacCOM_lin).block(j,6,3,12);
            Jstdot_qdot.block(riemp*3,0,3,1) +=iDynTree::toEigen(JdqdCOM_lin).block(j,0,3,1);
            riemp++;
        }else{
            Jsw_com.block(riemp_sw*3,0,3,6)+= iDynTree::toEigen(JacCOM_lin).block(j,0,3,6);
            Jsw_j.block(riemp_sw*3,0,3,12)  += iDynTree::toEigen(JacCOM_lin).block(j,6,3,12);
            Jswdot_qdot.block(riemp_sw*3,0,3,1) +=iDynTree::toEigen(JdqdCOM_lin).block(j,0,3,1);
            riemp_sw++;
        }
    }

}   

void DOGCTRL::quadratic_problem_solver_swing(){ 

    // --- Setting Q e c di Alglib ---
    int riemp_fhat=0;
    Eigen::Matrix<double,6,30>  T_al;
    Eigen::Matrix<double,30,30>  Q_al;
    Eigen::Matrix<double,6,6>  W_al;
    Eigen::Matrix<double,30,1>  c_al;
    Eigen::Matrix<double,30,30>  R;

    Eigen::MatrixXd Sigma;
    Eigen::MatrixXd f_hat_st;
    Eigen::MatrixXd Jst_com;
    Eigen::MatrixXd Jst_j;
    Eigen::MatrixXd Jsw_com;
    Eigen::MatrixXd Jsw_j;
    Eigen::MatrixXd Jstdot_qdot;
    Eigen::MatrixXd Jswdot_qdot;

    Sigma.resize(3*_nst,30);
    f_hat_st.resize(3*_nst,1);
    Jst_com.resize(3*_nst,6);
    Jst_j.resize(3*_nst,12);
    Jsw_com.resize(3*(4-_nst),6);
    Jsw_j.resize(3*(4-_nst),12);
    Jstdot_qdot.resize(3*_nst,1);
    Jswdot_qdot.resize(3*(4-_nst),1);

    Sigma<<Eigen::MatrixXd::Zero(3*_nst,30);
    f_hat_st<<Eigen::MatrixXd::Zero(3*_nst,1);
    Jst_com<<Eigen::MatrixXd::Zero(3*_nst,6);
    Jst_j<<Eigen::MatrixXd::Zero(3*_nst,12);
    Jsw_com<<Eigen::MatrixXd::Zero(3*(4-_nst),6);
    Jsw_j<<Eigen::MatrixXd::Zero(3*(4-_nst),12);
    Jstdot_qdot<<Eigen::MatrixXd::Zero(3*_nst,1);
    Jswdot_qdot<<Eigen::MatrixXd::Zero(3*(4-_nst),1);

    _o_sw = new OPT( 30,6+3*_nst+4*_nst+24+3*2*(4-_nst),6+3*_nst+4*_nst+24+3*2*(4-_nst)); 
    
    int j=0;
    for( int i=0; i<4;i++){
        if(i==0)j=3;
        else if(i==1)j=0;
        else j=i*3;
        if(desired_ee_contact[i]){ 
            f_hat_st.block(3*riemp_fhat,0,3,1)=_f_hat.block(j,0,3,1);
            riemp_fhat++;
        }
    }

    compute_dyn_J(Jst_com,Jst_j,Jsw_com,Jsw_j,Jstdot_qdot,Jswdot_qdot);

    Sigma.block(0,18,3*_nst,3*_nst) = Eigen::MatrixXd::Identity(3*_nst,3*_nst);
    T_al = Jst_com.transpose()*Sigma; 
    W_al = 50*Eigen::Matrix<double,6,6>::Identity();
    R=Eigen::Matrix<double,30,30>::Identity();
    R.block(24,24,6,6)=10000*Eigen::Matrix<double,6,6>::Identity(); 
    Q_al = T_al.transpose()*W_al*T_al+ R; 
    c_al << -T_al.transpose()*W_al.transpose()*( _W_CoM_des - Jst_com.transpose()*f_hat_st); 

    _o_sw->setQ(Q_al);
    _o_sw->setc(c_al);

    //------ Costruzione matrici L e Lt

    double mi = 1.0;
    Eigen::Matrix<double,4,3> cone= Eigen::Matrix<double,4,3>::Zero();
    Eigen::Vector3d n;
    Eigen::Vector3d t1;
    Eigen::Vector3d t2;
    Eigen::Matrix<double,12,1> tau_max =  60*Eigen::Matrix<double,12,1>::Ones(); 
    Eigen::Matrix<double,12,1> tau_min = -60*Eigen::Matrix<double,12,1>::Ones(); 
    Eigen::MatrixXd A_swing;
    Eigen::MatrixXd b_swing;
    Eigen::MatrixXd D_swing;
    Eigen::MatrixXd c_swing;
    Eigen::MatrixXd Dfr;
    Eigen::MatrixXd L_swing;


    A_swing.resize(6+3*_nst,30);
    b_swing.resize(6+3*_nst,1);
    D_swing.resize(4*_nst+24+3*2*(4-_nst),30);
    c_swing.resize( 4*_nst+ 24 + 2*3*(4-_nst), 1);
    Dfr.resize(4*_nst,3*_nst);
    L_swing.resize(6+3*_nst+4*_nst+24+3*2*(4-_nst), 31);

    A_swing<<Eigen::MatrixXd::Zero(6+3*_nst,30);
    b_swing<<Eigen::MatrixXd::Zero(6+3*_nst,1);
    D_swing<<Eigen::MatrixXd::Zero(4*_nst+24+3*2*(4-_nst),30);
    c_swing<<Eigen::MatrixXd::Zero( 4*_nst+ 24 + 2*3*(4-_nst), 1);
    Dfr<<Eigen::MatrixXd::Zero(4*_nst,3*_nst);
    L_swing<<Eigen::MatrixXd::Zero(6+3*_nst+4*_nst+24+3*2*(4-_nst), 31);

    A_swing<<  iDynTree::toEigen(MassMatrixCOM).block(0,0,6,6), Eigen::Matrix<double,6,12>::Zero(),-Jst_com.transpose(),Eigen::MatrixXd::Zero(6,3*(4-_nst)),
                Jst_com, Jst_j, Eigen::MatrixXd::Zero(3*_nst,3*_nst),Eigen::MatrixXd::Zero(3*_nst,3*(4-_nst));


    b_swing<<  -iDynTree::toEigen(BiasCOM).block(0,0,6,1),
            -Jstdot_qdot;

    n<<0,0,1;
    t2<<0,1,0;
    t1<<1,0,0;
    cone<<  (t1 - mi*n).transpose(),
            -(t1 + mi*n).transpose(),
            (t2 - mi*n).transpose(),
            -(t2 + mi*n).transpose();
    for( int i=0; i<_nst;i++)Dfr.block(4*i,3*i,4,3)=cone;

    D_swing <<  Eigen::MatrixXd::Zero(4*_nst,18), Dfr,Eigen::MatrixXd::Zero(4*_nst,3*(4-_nst)),
                Eigen::Matrix<double,12,6>::Zero(), iDynTree::toEigen(MassMatrixCOM).block(6,6,12,12), -Jst_j.transpose(),Eigen::MatrixXd::Zero(12,3*(4-_nst)),
                Eigen::Matrix<double,12,6>::Zero(), - iDynTree::toEigen(MassMatrixCOM).block(6,6,12,12), Jst_j.transpose(),Eigen::MatrixXd::Zero(12,3*(4-_nst)),
                Jsw_com, Jsw_j, Eigen::MatrixXd::Zero(3*(4-_nst),3*_nst),-Eigen::MatrixXd::Identity(3*(4-_nst),3*(4-_nst)),
                -Jsw_com, -Jsw_j, Eigen::MatrixXd::Zero(3*(4-_nst),3*_nst),-Eigen::MatrixXd::Identity(3*(4-_nst),3*(4-_nst));

    c_swing<<   Eigen::MatrixXd::Zero(4*_nst,1),
                tau_max -iDynTree::toEigen(BiasCOM).block(6,0,12,1),
                -(tau_min -iDynTree::toEigen(BiasCOM).block(6,0,12,1)),
                _Xdotdot_sw_cmd -Jswdot_qdot , 
                -_Xdotdot_sw_cmd +Jswdot_qdot; 

    L_swing<<   A_swing,b_swing,
                D_swing,c_swing;


    if(_nst==2){
        _o_sw->setL_swing(L_swing,12); 
        _o_sw->opt_swing(x_eigen);
        _tau =iDynTree::toEigen(MassMatrixCOM).block(6,6,12,12)*x_eigen.block(6,0,12,1) + iDynTree::toEigen(BiasCOM).block(6,0,12,1) - Jst_j.transpose()*x_eigen.block(18,0,3*_nst,1);


    }else ROS_INFO("[ERROR] problema di ottimo swing con nst!=2");

    if(desired_ee_contact[0]){
        demsg.fgr_star1 = x_eigen(18,0);
        demsg.fgr_star2 = x_eigen(19,0);
        demsg.fgr_star3 = x_eigen(20,0);
        demsg.fgr_star4 = _fgr_br(0,0);
        demsg.fgr_star5 = _fgr_br(1,0);
        demsg.fgr_star6 = _fgr_br(2,0);
    }else{
        demsg.fgr_star1 = 0;
        demsg.fgr_star2 = 0;
        demsg.fgr_star3 = 0;
        demsg.fgr_star4 = _fgr_br(0,0);
        demsg.fgr_star5 = _fgr_br(1,0);
        demsg.fgr_star6 = _fgr_br(2,0);
    }
    demsg.stance = int(desired_ee_contact[0]);


    delete _o_sw;
}

void DOGCTRL::ctrl_loop() {
    ROS_INFO("CTRL LOOP STARTED");
    bool first_cycle=true;
    bool is_fine=true;
    double soglia_tolleranza=0.5;
    gazebo::msgs::WorldControl step;
    gazebo::transport::PublisherPtr gazebo_pub; 
    gazebo::transport::NodePtr node(new gazebo::transport::Node());


    node->Init();  
    gazebo_pub = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");  
    gazebo_pub->WaitForConnection();
    ROS_INFO("gazebo pub connesso");
    step.set_step(1);
    
    ros::ServiceClient pauseGazebo = _nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    ros::ServiceClient unpauseGazebo = _nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    std_srvs::Empty pauseSrv;
    //wait for first data...
    while( !_first_wpose  )
        usleep(0.1*1e6);

    while( !_first_jpos  )
        usleep(0.1*1e6);

    //Update robot state using the update function
    Eigen::Vector3d gravity;
    gravity << 0, 0, -9.8;
    update(_world_H_base, _jnt_pos, _jnt_vel, _base_vel, gravity);
    _COM_pos_old<<toEigen(CoM);
    

    //If towr takes too much time, you can stop the simulation, use the trajectory generator function and restart the simulation
    ros::Rate r(1000);
    while( ros::ok() ) {

        if(pauseGazebo.call(pauseSrv))
            ROS_INFO("Simulation paused.");
        else
            ROS_INFO("Failed to pause simulation.");
        do{
            update(_world_H_base, _jnt_pos, _jnt_vel, _base_vel, gravity);  
            //Mi assicuro che la posizione iniziale sia coerente e non soffra di sbalzi dovuti ad errori runtime di gazebo
            is_fine=true;
            for(int i=0;i<5;i++){
                if(fabs(_COM_pos_old[i]-toEigen(CoM)[i]) > soglia_tolleranza){
                    is_fine=false;
                    ROS_ERROR("COM bug");
                    if(!is_fine){
                        cout<<"\n _COM_pos_old: "<<_COM_pos_old;
                        cout<<"\n toEigen(CoM): "<<toEigen(CoM);
                    }
                }
            }
        }while(!is_fine);
        _COM_pos_old<<toEigen(CoM);
        trajectoryplanner();

        print_towr_trajectory();
        ROS_INFO("STARTING CTRL");
        ros::Time t0 = ros::Time::now();
        ros::Time t = ros::Time::now();
        while ((t-t0).toSec() < solution.base_linear_->GetTotalTime() -1e-5){ //0.5 è la durata impostata su towr
            
            pauseGazebo.call(pauseSrv);
            update(_world_H_base, _jnt_pos, _jnt_vel, _base_vel, gravity);
            _nst=0;
            for(int i=0;i<4;i++){
                desired_ee_contact[i]=solution.phase_durations_.at(_remap[i]) ->IsContactPhase((t-t0).toSec());
                _nst +=int(desired_ee_contact[i]);
            }
            momentum_observer(); 
            W_CoM_des_calc((t-t0).toSec());
            if(_nst==4 || first_cycle){
                quadratic_problem_solver_stance();
                first_cycle=false;
            }
            else{
                X_dot_dot_sw_calc((t-t0).toSec());
                quadratic_problem_solver_swing();
            }
            unpauseGazebo.call(pauseSrv);   
            publish_cmd(_tau);

            _debug_pub.publish(demsg);
            t = ros::Time::now();
            r.sleep();
        }
        
        
        r.sleep();        
    }
}

void DOGCTRL::run() {
    boost::thread ctrl_loop_t ( &DOGCTRL::ctrl_loop, this);
    ros::spin();
}

int main(int argc, char** argv) {
    ros::init( argc, argv, "popt");
    DOGCTRL dc;
    gazebo::client::setup(argc,argv); 
    dc.run();
}