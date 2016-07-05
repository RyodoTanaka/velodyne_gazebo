#include "velodyne_plugin.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include <cmath>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)

  VelodynePlugin::VelodynePlugin() 
  {}

  /// \brief The load function is called by Gazebo when the plugin is
  /// inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is
  /// attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
  void VelodynePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
	// Safety check
	if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
        return;
      }
	
	// Store the model pointer for convenience.
	this->model = _model;
	
	// Get the first joint. We are making an assumption about the model
	// having one joint that is the rotational joint.
	this->joint = _model->GetJoints()[0];

	// Get the rotation Link.
	this->link = this->joint->GetChild();
	
	// Default to zero hertz.
	double hz = 0;
	
	// Check that the velocity element exists, then read the value
	if (_sdf->HasElement("hz"))
	  hz = _sdf->Get<double>("hz");
	
	this->SetVelocity(hz);
	
	// Create the node
	this->node = transport::NodePtr(new transport::Node());
	this->node->Init(this->model->GetWorld()->GetName());
	
	// Create a topic name
	std::string sub_TopicName = "~/" + this->model->GetName() + "/hz";
	
	// Subscribe to the topic, and register a callback
	this->sub = this->node->Subscribe(sub_TopicName,
									  &VelodynePlugin::OnMsg, this);
	// Initialize ros, if it has not already bee initialized.
	if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
				  ros::init_options::NoSigintHandler);
      }
	
	// Create our ROS node. This acts in a similar manner to
	// the Gazebo node
	this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
	
	// Create a named topic, and subscribe to it.
	ros::SubscribeOptions so =
      ros::SubscribeOptions::create<std_msgs::Float32>(
	  "/" + this->model->GetName() + "/hz",
      1,
      boost::bind(&VelodynePlugin::OnRosMsg, this, _1),
      ros::VoidPtr(), &this->rosQueue);
	this->rosSub = this->rosNode->subscribe(so);
	
	// Spin up the queue helper thread.
	this->rosQueueThread =
      std::thread(std::bind(&VelodynePlugin::QueueThread, this));
  }

  /// \brief Set the velocity of the Velodyne
  /// \param[in] _vel New target velocity
  void VelodynePlugin::SetVelocity(const double &_hz)
  {
	math::Vector3 linear_vel;
	math::Vector3 angular_vel;
    double vel = M_PI*(double)_hz;
	linear_vel = math::Vector3::Zero;
	angular_vel.Set(0, 0, vel);
	link->SetLinearVel(linear_vel);
	link->SetAngularVel(angular_vel);
  }

  /// \brief Handle an incoming message from ROS
  /// \param[in] _msg A float value that is used to set the velocity
  /// of the Velodyne.
  void VelodynePlugin::OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
  {
	this->SetVelocity(_msg->data);
  }

  /// \brief ROS helper function that processes messages
  void VelodynePlugin::QueueThread()
  {
	static const double timeout = 0.01;
	while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
  }

  /// \brief Handle incoming message
  /// \param[in] _msg Repurpose a vector3 message. This function will
  /// only use the x component.
  void VelodynePlugin::OnMsg(ConstVector3dPtr &_msg)
  {
	this->SetVelocity(_msg->x());
  }
}
