#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "ros/ros.h"

#include <ignition/math.hh>
namespace gazebo{

    class DogbotExtForce : public ModelPlugin{

    
        public: 
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){    
                this->model=_parent;
                
                 this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&DogbotExtForce::OnUpdate, this));
            }

            void OnUpdate(){
                _base_link=model->GetLink("base_link");
                ignition::math::Pose3d base_pose=_base_link->WorldCoGPose();
                ignition::math::Vector3 base_position=base_pose.Pos();
                ignition::math::Vector3d force( 0,0,0); // insert the force in the world coordinate
                _base_link->AddForceAtWorldPosition(force,base_position);

               
            }

        private: 
            physics::ModelPtr model;
            physics::LinkPtr  _base_link;
            event::ConnectionPtr updateConnection;


    };

    GZ_REGISTER_MODEL_PLUGIN(DogbotExtForce)
}


