#include <iostream>
#include <string.h>
#include "Leap.h"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
//#include "geometry_msgs/PoseStamped.h"
//#include "hiqp_msgs/PoseWithName.h"
#include "std_msgs/Float32.h"

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Eigen>
#include <sstream>


using namespace Leap;
using namespace std;

class HandsListener : public Listener {
  public:
  ros::NodeHandle _node;
  ros::Publisher _pub_marker_array;
  
  ros::Publisher _pub_hand_grasp_left;
  ros::Publisher _pub_hand_grasp_right;
 
  tf::TransformListener listener;
  tf::TransformBroadcaster br; 

  unsigned int seq;
  virtual void onInit(const Controller&);
  virtual void onConnect(const Controller&);
  virtual void onDisconnect(const Controller&);
  virtual void onExit(const Controller&);
  virtual void onFrame(const Controller&);
  virtual void onFocusGained(const Controller&);
  virtual void onFocusLost(const Controller&);
  virtual void onDeviceChange(const Controller&);
  virtual void onServiceConnect(const Controller&);
  virtual void onServiceDisconnect(const Controller&);  
  private:
  double min_hand_confidence, ws_scale;
  Eigen::Affine3d static_transform;
  std::string frame_name, publish_frame_name, left_hand_frame, right_hand_frame;

};

void HandsListener::onInit(const Controller& controller) {
  _node = ros::NodeHandle("~");
  _pub_marker_array = _node.advertise<visualization_msgs::MarkerArray>("hands", 1);
  
  _pub_hand_grasp_left = _node.advertise<std_msgs::Float32>("grasp_left", 1);
  _pub_hand_grasp_right = _node.advertise<std_msgs::Float32>("grasp_right", 1);

  _node.param("min_hand_confidence", min_hand_confidence, 0.1);
  _node.param<std::string>("frame_name", frame_name, "/leap_optical_frame");
  _node.param<std::string>("publish_frame_name", publish_frame_name, "/world");
  _node.param<std::string>("left_frame_name", left_hand_frame, "/leap_left_hand");
  _node.param<std::string>("right_frame_name", right_hand_frame, "/leap_right_hand");
  _node.param("workspace_scale", ws_scale, 1.0);
  
  ROS_INFO("Initialized node, waiting for transform");

  tf::StampedTransform transform;

  try{
      listener.waitForTransform(publish_frame_name, frame_name, ros::Time(0), ros::Duration(10.0) );
      listener.lookupTransform(publish_frame_name, frame_name, ros::Time(0), transform);
      ROS_INFO("Got transform to publish frame");
  }
  catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
  }
  
  tf::transformTFToEigen(transform, static_transform);
}


void HandsListener::onConnect(const Controller& controller) {
  std::cout << "Connected" << std::endl;
  //screw the Gestures
//  controller.enableGesture(Gesture::TYPE_CIRCLE);
//  controller.enableGesture(Gesture::TYPE_KEY_TAP);
//  controller.enableGesture(Gesture::TYPE_SCREEN_TAP);
//  controller.enableGesture(Gesture::TYPE_SWIPE);
}

void HandsListener::onDisconnect(const Controller& controller) {
  // Note: not dispatched when running in a debugger.
  std::cout << "Disconnected" << std::endl;
}

void HandsListener::onExit(const Controller& controller) {
  std::cout << "Exited" << std::endl;
}

void HandsListener::onFrame(const Controller& controller) {
  // Get the most recent frame and report some basic information
  const Frame frame = controller.frame();
  //ROS_INFO("flags = %i", (int) controller.policyFlags());
  visualization_msgs::Marker marker_msg, joint_msg;
  visualization_msgs::MarkerArray marker_array_msg;
  marker_msg.header.frame_id=joint_msg.header.frame_id="leap_optical_frame";
  marker_msg.header.stamp=joint_msg.header.stamp=ros::Time::now();
  marker_msg.ns="leap_marker";
  joint_msg.ns="joint";
  marker_msg.id = 0; 
  joint_msg.id = 0;
  marker_msg.type = visualization_msgs::Marker::LINE_LIST;
  joint_msg.type = visualization_msgs::Marker::SPHERE;
  marker_msg.scale.x = 0.005;
  joint_msg.scale.x = joint_msg.scale.y = joint_msg.scale.z = 0.015;
  joint_msg.color.r = .0f;
  joint_msg.color.g = 1.0f;
  joint_msg.color.b = 1.0f;
  joint_msg.color.a = 0.7f;
  marker_msg.action = joint_msg.action = visualization_msgs::Marker::ADD;
  marker_msg.lifetime = joint_msg.lifetime = ros::Duration(0.1);
    
  geometry_msgs::PoseStamped hand_pose;
  hand_pose.header.frame_id=frame_name;
  hand_pose.header.stamp=ros::Time::now();

  HandList hands = frame.hands();
  for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
    //Marker stuff for visualization
    // Get the first hand
    const Hand hand = *hl;
    if(!hand.isValid()) continue;
    if(hand.confidence() < min_hand_confidence) continue;
    // Get the Arm bone
    // Get fingers
    const FingerList fingers = hand.fingers();
    for (FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) {
      const Finger finger = *fl;
      // Get finger bones
      for (int b = 0; b < 4; ++b) {
        Bone::Type boneType = static_cast<Bone::Type>(b);
        Bone bone = finger.bone(boneType);
	geometry_msgs::Point point;
	point.y = -bone.prevJoint().x/1000;
	point.x = -bone.prevJoint().z/1000;
	point.z = bone.prevJoint().y/1000;
	marker_msg.points.push_back(point);
	point.y = joint_msg.pose.position.y =  -bone.nextJoint().x/1000;
	point.x = joint_msg.pose.position.x = -bone.nextJoint().z/1000;
	point.z = joint_msg.pose.position.z = bone.nextJoint().y/1000;
	marker_msg.points.push_back(point);
	joint_msg.id = joint_msg.id+1;
	marker_array_msg.markers.push_back(joint_msg);
	std_msgs::ColorRGBA color;
	color.r = 1.0f; color.g=.0f; color.b=.0f, color.a=1.0f;
	marker_msg.colors.push_back(color);
	marker_msg.colors.push_back(color);
      }
    }
    marker_array_msg.markers.push_back(marker_msg);

    //serious pose stuff
    Eigen::Affine3d m;
    m = Eigen::AngleAxisd(-hand.direction().yaw(), Eigen::Vector3d::UnitZ())
	* Eigen::AngleAxisd(-hand.direction().pitch(), Eigen::Vector3d::UnitY())
	* Eigen::AngleAxisd(-hand.palmNormal().roll(), Eigen::Vector3d::UnitX());
    //rotate to expected hand coordinate frame
    m = m*Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY());
    //add translation
    m.translation() <<  -hand.palmPosition().z/1e3,-hand.palmPosition().x/1e3,hand.palmPosition().y/1e3;

    m.translation() = ws_scale*m.translation();

    m = static_transform*m;

    tf::Transform transform;
    tf::poseEigenToTF(m, transform);

    std_msgs::Float32 grasp;
    grasp.data = hand.grabStrength();
    if(hand.isLeft()) {
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), publish_frame_name, left_hand_frame));
	_pub_hand_grasp_left.publish(grasp);
    }
    if(hand.isRight()) {
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), publish_frame_name, right_hand_frame));
	_pub_hand_grasp_right.publish(grasp);
    }
  }

  _pub_marker_array.publish(marker_array_msg);
}

void HandsListener::onFocusGained(const Controller& controller) {
  std::cout << "Focus Gained" << std::endl;
}

void HandsListener::onFocusLost(const Controller& controller) {
  std::cout << "Focus Lost" << std::endl;
}

void HandsListener::onDeviceChange(const Controller& controller) {
  std::cout << "Device Changed" << std::endl;
  const DeviceList devices = controller.devices();

  for (int i = 0; i < devices.count(); ++i) {
    std::cout << "id: " << devices[i].toString() << std::endl;
    std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
  }
}

void HandsListener::onServiceConnect(const Controller& controller) {
  std::cout << "Service Connected" << std::endl;
}

void HandsListener::onServiceDisconnect(const Controller& controller) {
  std::cout << "Service Disconnected" << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "leap_sender");
  // Create a sample listener and controller
  HandsListener listener;
  Controller controller;
  
  // Have the sample listener receive events from the controller
  controller.addListener(listener);

  controller.setPolicyFlags(static_cast<Leap::Controller::PolicyFlag> (Leap::Controller::POLICY_IMAGES));
  ros::spin();
  // Remove the sample listener when done
  controller.removeListener(listener);

  return 0;
}
