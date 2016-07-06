#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class VelodynePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: VelodynePlugin();

	/// \brief Destructor
    public: ~VelodynePlugin();
    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Set the velocity of the Velodyne
    /// \param[in] _vel New target velocity
    public: void SetVelocity(const double &_rate);

    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the Velodyne.
    public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg);

    /// \brief ROS helper function that processes messages
    private: void SubQueueThread();

	/// \brief Update Point Cloud datas
    protected: void PointCloudUpdate();

	/// \brief connection counter
    private: void PC2connectCB();

	/// \brief disconnection counter
    private: void PC2disconnectCB();

	/// \brief ROS helper function that processes messages
    private: void PubQueueThread();
	
	/// member
    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

	/// \brief Pointer to the link.
    private: physics::LinkPtr link;

	/// \brief Velodyne Turning rate;
    private : double rate;
	
	/// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;

	/// \breaf A ROS publisher
    private: ros::Publisher rosPub;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue SubrosQueue;

    /// \brief A thread the keeps running the SubrosQueue
    private: std::thread SubrosQueueThread;

	/// \brief A ROS callback that counts the connection
    private: ros::CallbackQueue PubrosQueue;

	/// \brief A thread the keeps runnning the PubrosQueue
    private: std::thread PubrosQueueThread;

	/// \brief A counter for connection
    private : int point_cloud_connect_count;
	
	/// \brief Pointer to the update event connection
    private: event::ConnectionPtr update_connection_;

	/// \brief Point Cloud data
    private: sensor_msgs::PointCloud2ConstPtr pc2_data;
  };

}

#endif
