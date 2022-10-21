// Copyright 2020 National Institute of Advanced Industrial Science and Technology, Japan
// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Floris Erich <floris.erich@aist.go.jp>,
//         David Vargas Frutos <david.vargas@urjc.es>
//
// Also includes code fragments from Kumar Robotics ROS 1 Qualisys driver

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <utility>
#include <chrono>
#include <iostream>
#include <functional>
#include <ratio>
#include <tgmath.h>
#include "qualisys_driver/qualisys_driver.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;

std_msgs::msg::Header_<std::allocator<void> >::_stamp_type stampRt2Ros(long long unsigned int rtStamp){
	int sec = static_cast<int>(rtStamp/1000000);
	int nsec = static_cast<int>(rtStamp%1000000);
	std_msgs::msg::Header_<std::allocator<void>>::_stamp_type rosStamp;
	rosStamp.sec = sec;
	rosStamp.nanosec = nsec;
	return rosStamp;
};


inline float SIGN(float x) {
	return (x >= 0.0f) ? +1.0f : -1.0f;
}

inline float NORM(float a, float b, float c, float d) {
	return sqrt(a * a + b * b + c * c + d * d);
}

// quaternion = [w, x, y, z]'
float* mRot2Quat(const float* m) {
	float* res = new float[4];
	//std::cout << ("Calculating quaternions\n");
	float r11 = m[0];
	float r12 = m[1];
	float r13 = m[2];
	float r21 = m[3];
	float r22 = m[4];
	float r23 = m[5];
	float r31 = m[6];
	float r32 = m[7];
	float r33 = m[8];
	//std::cout << r11 << " " << r12 << " "<< r13 << " "<< r21 << " "<< r22 << " "<< r23 << " "<< r31 << " "<< r32 << " "<< r33;
	//std::cout << "\n\n";

	float q0 = (r11 + r22 + r33 + 1.0f) / 4.0f;
	float q1 = (r11 - r22 - r33 + 1.0f) / 4.0f;
	float q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
	float q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
	if (q0 < 0.0f) {
		q0 = 0.0f;
	}
	if (q1 < 0.0f) {
		q1 = 0.0f;
	}
	if (q2 < 0.0f) {
		q2 = 0.0f;
	}
	if (q3 < 0.0f) {
		q3 = 0.0f;
	}
	q0 = sqrt(q0);
	q1 = sqrt(q1);
	q2 = sqrt(q2);
	q3 = sqrt(q3);
	if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
		q0 *= +1.0f;
		q1 *= SIGN(r32 - r23);
		q2 *= SIGN(r13 - r31);
		q3 *= SIGN(r21 - r12);
	}
	else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
		q0 *= SIGN(r32 - r23);
		q1 *= +1.0f;
		q2 *= SIGN(r21 + r12);
		q3 *= SIGN(r13 + r31);
	}
	else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
		q0 *= SIGN(r13 - r31);
		q1 *= SIGN(r21 + r12);
		q2 *= +1.0f;
		q3 *= SIGN(r32 + r23);
	}
	else if (q3 >= q0 && q3 >= q1 && q3 >= q2) {
		q0 *= SIGN(r21 - r12);
		q1 *= SIGN(r31 + r13);
		q2 *= SIGN(r32 + r23);
		q3 *= +1.0f;
	}
	else {
	        res[0] = -999999; res[1] =  -999999; res[2] = -999999; res[3] = -999999;
		return res;
		printf("coding error\n");
	}
	float r = NORM(q0, q1, q2, q3);
	q0 /= r;
	q1 /= r;
	q2 /= r;
	q3 /= r;

	res[0] = q0; res[1] =  q1; res[2] = q2; res[3] = q3;
	return res;
}


void QualisysDriver::set_settings_qualisys()
{
}

void QualisysDriver::loop()
{
  CRTPacket * prt_packet = port_protocol_.GetRTPacket();
  CRTPacket::EPacketType e_type;
  port_protocol_.GetCurrentFrame(CRTProtocol::cComponent6dRes);//CRTProtocol::cComponent3dNoLabels
  if (port_protocol_.ReceiveRTPacket(e_type, true)) {
    switch (e_type) {
      case CRTPacket::PacketError:
        {
          std::string s = "Error when streaming frames: ";
          s += port_protocol_.GetRTPacket()->GetErrorString();
          RCLCPP_ERROR(get_logger(), s.c_str());
          break;
        }
      case CRTPacket::PacketNoMoreData:
        RCLCPP_WARN(get_logger(), "No more data");
        break;
      case CRTPacket::PacketData:
        process_packet(prt_packet);
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown CRTPacket");
    }
  }
}

void QualisysDriver::process_packet(CRTPacket * const packet)
{
  unsigned int marker_count = packet->Get3DNoLabelsMarkerCount();
  unsigned int body_count = packet->Get6DOFResidualBodyCount();
  int frame_number = packet->GetFrameNumber();

  int frame_diff = 0;
  if (last_frame_number_ != 0) {
    frame_diff = frame_number - last_frame_number_;
    frame_count_ += frame_diff;

    if (frame_diff > 1) {
      dropped_frame_count_ += frame_diff;
      double dropped_frame_pct = static_cast<double>(dropped_frame_count_ / frame_count_ * 100);

      RCLCPP_DEBUG(
        get_logger(),
        "%d more (total %d / %d, %f %%) frame(s) dropped. Consider adjusting rates",
        frame_diff, dropped_frame_count_, frame_count_, dropped_frame_pct
      );
    }
  }
  last_frame_number_ = frame_number;

  geometry_msgs::msg::PoseWithCovarianceStamped body_msg;
  if (use_markers_with_id_) {
    if (!marker_with_id_pub_->is_activated()) {
      return;
    }

    mocap_msgs::msg::Markers markers_msg;
    markers_msg.header.stamp = rclcpp::Clock().now();
    markers_msg.frame_number = frame_number;

    for (unsigned int i = 0; i < marker_count; ++i) {
      float x, y, z;
      unsigned int id;
      packet->Get3DNoLabelsMarker(i, x, y, z, id);
      mocap_msgs::msg::Marker this_marker;
      this_marker.marker_index = id;
      this_marker.translation.x = x / 1000;
      this_marker.translation.y = y / 1000;
      this_marker.translation.z = z / 1000;
      markers_msg.markers.push_back(this_marker);
    }

    marker_with_id_pub_->publish(markers_msg);
  }
  else {
    if (!marker_pub_->is_activated()) {
      return;
    }

    mocap_msgs::msg::Markers markers_msg;
    markers_msg.header.stamp = rclcpp::Clock().now();
    markers_msg.frame_number = frame_number;

    for (unsigned int i = 0; i < marker_count; ++i) {
      float x, y, z;
      unsigned int id;
      packet->Get3DNoLabelsMarker(i, x, y, z, id);
      mocap_msgs::msg::Marker this_marker;
      this_marker.translation.x = x / 1000;
      this_marker.translation.y = y / 1000;
      this_marker.translation.z = z / 1000;
      markers_msg.markers.push_back(this_marker);
    }

    marker_pub_->publish(markers_msg);
  }

  //RCLCPP_INFO(get_logger(), std::to_string(body_count).c_str());
  const bool printOutput = false;
  float fX, fY, fZ, fRes;
  float* rotationMatrix = new float[9];
  body_msg.header.stamp = stampRt2Ros(packet->GetTimeStamp());
 //RCLCPP_INFO(get_logger(), std::to_string(packet->GetTimeStamp()).c_str());
  for (unsigned int i = 0; i < body_count; i++){
	  if (packet->Get6DOFResidualBody(i, fX, fY, fZ, rotationMatrix, fRes))
	  {
		// Read the 6DOF rigid body name
		body_msg.header.frame_id = port_protocol_.Get6DOFBodyName(i);
		// Output 6DOF data
		body_msg.pose.pose.position.x = fX;
		body_msg.pose.pose.position.y = fY;
		body_msg.pose.pose.position.z = fZ;
		float* temp = mRot2Quat(rotationMatrix);
		body_msg.pose.pose.orientation.x = temp[1];
		body_msg.pose.pose.orientation.y = temp[2];
		body_msg.pose.pose.orientation.z = temp[3];
		body_msg.pose.pose.orientation.w = temp[0];
		body_msg.pose.covariance = fRes;
		if(printOutput){
			RCLCPP_INFO(get_logger(),"Pos: %9.3f %9.3f %9.3f    Rot: %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n",
			fX, fY, fZ, rotationMatrix[0], rotationMatrix[1], rotationMatrix[2],
			rotationMatrix[3], rotationMatrix[4], rotationMatrix[5], rotationMatrix[6], rotationMatrix[7], rotationMatrix[8]);
		}
		rigid_body_pub_->publish(body_msg);
		delete[] temp;
	  }
	  if(printOutput)
		  RCLCPP_INFO(get_logger(), "\n");
  }
}

bool QualisysDriver::stop_qualisys()
{
  RCLCPP_INFO(get_logger(), "Stopping the Qualisys motion capture");
  port_protocol_.StreamFramesStop();
  port_protocol_.Disconnect();

  return true;
}

QualisysDriver::QualisysDriver(const rclcpp::NodeOptions node_options)
: rclcpp_lifecycle::LifecycleNode("qualisys_driver_node", node_options)
{
  initParameters();
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT QualisysDriver::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());

  auto rmw_qos_history_policy = name_to_history_policy_map.find(qos_history_policy_);
  auto rmw_qos_reliability_policy = name_to_reliability_policy_map.find(qos_reliability_policy_);
  auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(
      // The history policy determines how messages are saved until taken by
      // the reader.
      // KEEP_ALL saves all messages until they are taken.
      // KEEP_LAST enforces a limit on the number of messages that are saved,
      // specified by the "depth" parameter.
      rmw_qos_history_policy->second,
      // Depth represents how many messages to store in history when the
      // history policy is KEEP_LAST.
      qos_depth_
  ));
  // The reliability policy can be reliable, meaning that the underlying transport layer will try
  // ensure that every message gets received in order, or best effort, meaning that the transport
  // makes no guarantees about the order or reliability of delivery.
  qos.reliability(rmw_qos_reliability_policy->second);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
    "/qualisys_driver/change_state");

  marker_pub_ = create_publisher<mocap_msgs::msg::Markers>(
    "/markers", 100);

  marker_with_id_pub_ = create_publisher<mocap_msgs::msg::Markers>(
    "/markers_with_id", 100);

  rigid_body_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/rigid_bodies",100);

  update_pub_ = create_publisher<std_msgs::msg::Empty>(
    "/qualisys_driver/update_notify", qos);

  set_settings_qualisys();

  RCLCPP_INFO(get_logger(), "Configured!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT QualisysDriver::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  update_pub_->on_activate();
  marker_pub_->on_activate();
  rigid_body_pub_->on_activate();
  marker_with_id_pub_->on_activate();
  bool success = connect_qualisys();

  if (success) {
    timer_ = this->create_wall_timer(100ms, std::bind(&QualisysDriver::loop, this));

    RCLCPP_INFO(get_logger(), "Activated!\n");

    return CallbackReturnT::SUCCESS;
  } else {
    RCLCPP_INFO(get_logger(), "Unable to activate!\n");

    return CallbackReturnT::FAILURE;
  }
}

CallbackReturnT QualisysDriver::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  timer_->reset();
  update_pub_->on_deactivate();
  marker_pub_->on_deactivate();
  marker_with_id_pub_->on_deactivate();
  rigid_body_pub_->on_deactivate();
  stop_qualisys();
  RCLCPP_INFO(get_logger(), "Deactivated!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT QualisysDriver::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  update_pub_.reset();
  marker_pub_.reset();
  marker_with_id_pub_.reset();
  rigid_body_pub_.reset();
  timer_->reset();
  RCLCPP_INFO(get_logger(), "Cleaned up!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT QualisysDriver::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  /* Shut down stuff */
  RCLCPP_INFO(get_logger(), "Shutted down!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT QualisysDriver::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());

  return CallbackReturnT::SUCCESS;
}

bool QualisysDriver::connect_qualisys()
{
  RCLCPP_WARN(
    get_logger(),
    "Trying to connect to Qualisys host at %s:%d", host_name_.c_str(), port_);

  if (!port_protocol_.Connect(
      reinterpret_cast<const char *>(host_name_.data()), port_, 0, 1, 7))
  {
    RCLCPP_FATAL(get_logger(), "Connection error");
    return false;
  }
  RCLCPP_INFO(get_logger(), "Connected");

  bool settings_read;
  port_protocol_.Read6DOFSettings(settings_read);

  return settings_read;
}

void QualisysDriver::initParameters()
{
  declare_parameter<std::string>("host_name", "mocap");
  declare_parameter<int>("port", 22222);
  declare_parameter<int>("last_frame_number", 0);
  declare_parameter<int>("frame_count", 0);
  declare_parameter<int>("dropped_frame_count", 0);
  declare_parameter<std::string>("qos_history_policy", "keep_all");
  declare_parameter<std::string>("qos_reliability_policy", "best_effort");
  declare_parameter<int>("qos_depth", 10);
  declare_parameter<bool>("use_markers_with_id", true);

  get_parameter<std::string>("host_name", host_name_);
  get_parameter<int>("port", port_);
  get_parameter<int>("last_frame_number", last_frame_number_);
  get_parameter<int>("frame_count", frame_count_);
  get_parameter<int>("dropped_frame_count", dropped_frame_count_);
  get_parameter<std::string>("qos_history_policy", qos_history_policy_);
  get_parameter<std::string>("qos_reliability_policy", qos_reliability_policy_);
  get_parameter<int>("qos_depth", qos_depth_);
  get_parameter<bool>("use_markers_with_id", use_markers_with_id_);

  RCLCPP_INFO(
    get_logger(),
    "Param host_name: %s", host_name_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Param port: %d", port_);
  RCLCPP_INFO(
    get_logger(),
    "Param last_frame_number: %d", last_frame_number_);
  RCLCPP_INFO(
    get_logger(),
    "Param frame_count: %d", frame_count_);
  RCLCPP_INFO(
    get_logger(),
    "Param dropped_frame_count: %d", dropped_frame_count_);
  RCLCPP_INFO(
    get_logger(),
    "Param qos_history_policy: %s", qos_history_policy_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Param qos_reliability_policy: %s", qos_reliability_policy_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Param qos_depth: %d", qos_depth_);
  RCLCPP_INFO(
    get_logger(),
    "Param use_markers_with_id: %s", use_markers_with_id_ ? "true" : "false");
}
