#include <velodyne_gazebo_plugin/velodyne_plugin.h>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/JointState.hh>
#include <memory>
#include <cmath>
#include <ros/subscribe_options.h>

namespace gazebo
{
  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)

  VelodynePlugin::VelodynePlugin() 
  {
	this->point_cloud_connect_count = 0;
  }

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
	

	// Get parameters
	std::cout << std::endl;
	std::cout << "VELODYNE PLUGIN PARAMETER" << std::endl;
	
	std::string sensor_name;
	if(_sdf->HasElement("sensor_name"))
	  sensor_name = _sdf->Get<std::string>("sensor_name");
	else
	  sensor_name =  this->model->GetName();
	std::cout << " * sensor_name : " << sensor_name.c_str() << std::endl;
	
	std::string frame_name;
	if(_sdf->HasElement("frame_name"))
	  frame_name = _sdf->Get<std::string>("frame_name");
	else
	  frame_name = sensor_name;
	std::cout << " * frame_name : " << frame_name.c_str() << std::endl;
	
	std::string PCtopic_name;
	if(_sdf->HasElement("topic_name"))
	  PCtopic_name = _sdf->Get<std::string>("topic_name");
	else
	  PCtopic_name = " * point_cloud2";

	// Create a topic name
	std::string sub_TopicName = sensor_name + "/rate";
	std::string pub_TopicName = sensor_name + "/" + PCtopic_name;

	std::cout << " * rate topic_name : " << sub_TopicName.c_str() << std::endl; 
	std::cout << " * point cloud topic_name : " << pub_TopicName.c_str() << std::endl;
	
	unsigned int point_number;
	if(_sdf->HasElement("point_number"))
	  point_number = _sdf->Get<unsigned int>("point_number");
	else
	  point_number = 700000;
	std::cout << " * point_number : " << point_number << std::endl;

	if (_sdf->HasElement("rate"))
	  rate = _sdf->Get<double>("rate");
	else
	  this-> rate = 0;
	std::cout << " * rate : " << rate << std::endl;
	std::cout << std::endl;
	
	this->SetVelocity(this->rate);
		

	// Initialize ros, if it has not already bee initialized.
	if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
				  ros::init_options::NoSigintHandler);
      }
	
	// Create our ROS node. This acts in a similar manner to
	// the Gazebo nodeubscribe options
	this->rosNode.reset(new ros::NodeHandle);
	
	// Create a named topic, and subscribe to it.ubscribe options
	ros::SubscribeOptions so =
      ros::SubscribeOptions::create<std_msgs::Float32>(
	  sub_TopicName,
      1,
      boost::bind(&VelodynePlugin::OnRosMsg, this, _1),
      ros::VoidPtr(), &this->SubrosQueue);
	this->rosSub = this->rosNode->subscribe(so);

	ros::AdvertiseOptions ao =
	  ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
	  pub_TopicName,
	  1,
	  boost::bind(&VelodynePlugin::PC2connectCB, this),
	  boost::bind(&VelodynePlugin::PC2disconnectCB, this),
	  ros::VoidPtr(), &this->PubrosQueue);
	this->rosPub = this->rosNode->advertise(ao);
	
	// Spin up the queue helper thread.
	this->SubrosQueueThread =
	  std::thread(std::bind(&VelodynePlugin::SubQueueThread, this));
	this->PubrosQueueThread =
	  std::thread(std::bind(&VelodynePlugin::PubQueueThread, this));

	this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&VelodynePlugin::PointCloudUpdate, this));
  }

  /// \brief Set the velocity of the Velodyne
  /// \param[in] _vel New target velocity
  void VelodynePlugin::SetVelocity(const double &_rate)
  {
	math::Vector3 linear_vel;
	math::Vector3 angular_vel;
    double vel = M_PI*(double)_rate;
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
	this->rate = _msg->data;
	this->SetVelocity(_msg->data);
  }

  /// \brief ROS helper function that processes messages
  void VelodynePlugin::SubQueueThread()
  {
	static const double timeout = 0.01;
	while (this->rosNode->ok())
      {
		this->SubrosQueue.callAvailable(ros::WallDuration(timeout));
      }
  }
  
  void VelodynePlugin::PointCloudUpdate()
  {
	// ROS_INFO("here");
	// math::Angle now;
    now = this->joint->GetAngle(0);
	ROS_INFO("%10.10lf", now.Degree());

  }

  void VelodynePlugin::PC2connectCB(){
	this->point_cloud_connect_count++;
  }

  void VelodynePlugin::PC2disconnectCB(){
	this->point_cloud_connect_count--;
  }

  /// \brief ROS helper function that processes messages
  void VelodynePlugin::PubQueueThread()
  {
	static const double timeout = 0.01;
	while (this->rosNode->ok())
      {
		this->PubrosQueue.callAvailable(ros::WallDuration(timeout));
      }
  }
}
