// Copyright 2023 CMP Engineers Pty Ltd
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

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "ublox_nav_sat_fix_hp_node/visibility_control.h"
#include "ublox_ubx_msgs/msg/gps_fix.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_hp_pos_llh.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_status.hpp"

using std::placeholders::_1;

// size of position covariance array
static const size_t POS_COV_ARR_SIZE = 9;

namespace ublox_nav_sat_fix_hp
{

class UbloxNavSatHpFixNode : public rclcpp::Node
{
public:
  UBLOX_NAV_SAT_FIX_HP_NODE_PUBLIC
  explicit UbloxNavSatHpFixNode(const rclcpp::NodeOptions & options)
  : Node("ublox_nav_sat_fix_hp",
      rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
  {
    RCLCPP_INFO(this->get_logger(), "starting %s", get_name());

    auto sub_qos = rclcpp::SensorDataQoS();
    auto pub_qos = rclcpp::QoS(10).reliable();
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

    // Create publishers
    nav_sat_fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("fix", pub_qos, pub_options);

    // Create subscribers
    ubx_nav_hp_pos_llh_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavHPPosLLH>(
      "ubx_nav_hp_pos_llh", sub_qos,
      std::bind(&UbloxNavSatHpFixNode::nav_hp_pos_llh_callback, this, std::placeholders::_1));

    ubx_nav_status_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavStatus>(
      "ubx_nav_status", sub_qos,
      std::bind(&UbloxNavSatHpFixNode::nav_sta_callback, this, std::placeholders::_1));
  }

  UBLOX_NAV_SAT_FIX_HP_NODE_LOCAL
  ~UbloxNavSatHpFixNode() {RCLCPP_INFO(this->get_logger(), "finished");}

private:
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_pub_;

  rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavHPPosLLH>::SharedPtr ubx_nav_hp_pos_llh_sub_;
  rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavStatus>::SharedPtr ubx_nav_status_sub_;

  sensor_msgs::msg::NavSatStatus nav_sat_stat_;

  UBLOX_NAV_SAT_FIX_HP_NODE_LOCAL
  void nav_hp_pos_llh_callback(
    const ublox_ubx_msgs::msg::UBXNavHPPosLLH::SharedPtr ubx_hppos_llh_msg)
  {
    // Create the NavSatFix message
    sensor_msgs::msg::NavSatFix nav_sat_fix_msg;
    // header - copy from Pos message
    nav_sat_fix_msg.header = ubx_hppos_llh_msg->header;
    // copy status from previous nav_sat_stat message
    nav_sat_fix_msg.status = nav_sat_stat_;

    // Extract the LLH and high-precision components
    double lat = ubx_hppos_llh_msg->lat * 1e-7 + ubx_hppos_llh_msg->lat_hp * 1e-9;
    double lon = ubx_hppos_llh_msg->lon * 1e-7 + ubx_hppos_llh_msg->lon_hp * 1e-9;
    double alt = ubx_hppos_llh_msg->height * 1e-3 + ubx_hppos_llh_msg->height_hp * 1e-4;

    // Convert the LLH position and covariance values to NavSatFix message format
    nav_sat_fix_msg.latitude = lat;   // Degrees
    nav_sat_fix_msg.longitude = lon;  // Degrees
    nav_sat_fix_msg.altitude = alt;   // meters

    // Calucuate cov from HPPOSLLH
    double h_acc_m = ubx_hppos_llh_msg->h_acc * 1e-4;
    double v_acc_m = ubx_hppos_llh_msg->v_acc * 1e-4;

    double h_var = h_acc_m * h_acc_m;
    double v_var = v_acc_m * v_acc_m;

    nav_sat_fix_msg.position_covariance.fill(0.0);
    nav_sat_fix_msg.position_covariance[0] = h_var; // East
    nav_sat_fix_msg.position_covariance[4] = h_var; // North
    nav_sat_fix_msg.position_covariance[8] = v_var; // Up

    nav_sat_fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    // Publish NavSatFix message
    nav_sat_fix_pub_->publish(nav_sat_fix_msg);

    RCLCPP_DEBUG(
      this->get_logger(), "Published NavSatFix with lat %4f lon %4f alt %4f", lat, lon, alt);
  }

  UBLOX_NAV_SAT_FIX_HP_NODE_LOCAL
  void nav_sta_callback(const ublox_ubx_msgs::msg::UBXNavStatus::SharedPtr ubx_sta_msg)
  {
    switch (ubx_sta_msg->gps_fix.fix_type) {
      case ublox_ubx_msgs::msg::GpsFix::GPS_NO_FIX:
      case ublox_ubx_msgs::msg::GpsFix::GPS_TIME_ONLY:
      case ublox_ubx_msgs::msg::GpsFix::GPS_DEAD_RECKONING_ONLY:
        nav_sat_stat_.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        break;
      case ublox_ubx_msgs::msg::GpsFix::GPS_FIX_2D:
      case ublox_ubx_msgs::msg::GpsFix::GPS_FIX_3D:
      case ublox_ubx_msgs::msg::GpsFix::GPS_PLUS_DEAD_RECKONING:
        if (true == ubx_sta_msg->diff_soln) {  // diff corrections were applied
          nav_sat_stat_.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        } else {
          nav_sat_stat_.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        }
        break;
      default:
        nav_sat_stat_.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        break;
    }
    nav_sat_stat_.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  }
};

}  // namespace ublox_nav_sat_fix_hp

RCLCPP_COMPONENTS_REGISTER_NODE(ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode)
