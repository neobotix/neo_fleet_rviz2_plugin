/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef NEOFLEETRVIZ2PLUGIN_HPP_
#define NEOFLEETRVIZ2PLUGIN_HPP_

#include <QtWidgets>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_default_plugins/tools/goal_pose/goal_tool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "vector"
#include "map"
#include "memory"
#include "string"


class QLineEdit;
namespace neo_fleet
{

class RosHelper
{
private:
  rclcpp::Node::SharedPtr node_;

public:
  geometry_msgs::msg::PoseWithCovariance odom_pose_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr local_pos_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pos_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;

  bool is_localized_ = false;
  bool is_goal_sent_ = false;
  std::string robot_name_;

  RosHelper() {}

  RosHelper(rclcpp::Node::SharedPtr node, std::string robot_name)
  {
    node_ = node;
    robot_name_ = robot_name;
    local_pos_pub_ = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/" + robot_name_ + "/initialpose", 10);
    goal_pos_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/" + robot_name_ + "/goal_pose", 10);
    odom_subscriber_ = node->create_subscription<nav_msgs::msg::Odometry>(
      "/" + robot_name_ +
      "/odom", 1, std::bind(&RosHelper::odom_pose_cb, this, std::placeholders::_1));
    navigation_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      node_, "/" + robot_name_ + "/navigate_to_pose");
  }

  void odom_pose_cb(const nav_msgs::msg::Odometry::SharedPtr pose)
  {
    odom_pose_ = pose->pose;
  }
};


class Worker : public QObject
{
  Q_OBJECT

public:
  Worker();
  ~Worker();
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pos_sub_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initial_pose_;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_;

  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose);
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
  void checkAndStoreRobot(std::string & robot_name);

  rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("neo_fleet_thread");
  std::vector<std::shared_ptr<RosHelper>> robots_;
  std::map<std::string, std::shared_ptr<RosHelper>> robot_identity_map_;
  std::vector<std::string> robot_namespaces_;

  // Hardcoding the robot list - This later needs to be automized
  std::vector<std::string> available_robots_ = {"mpo_700", "mpo_500", "mp_400"};

public slots:
  void process();

signals:
  void finished();
  void data_recieved();
  void error(QString err);
};

class NeoFleetRViz2Plugin : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit NeoFleetRViz2Plugin(QWidget * parent = 0);
  ~NeoFleetRViz2Plugin();
  virtual void load(const rviz_common::Config & config);
  virtual void save(rviz_common::Config config) const;

  QThread * thread = new QThread;
  Worker * worker = new Worker();
  std::shared_ptr<RosHelper> robot_;

public slots:
  void update();

protected Q_SLOTS:
  void setRobotName();
  void launchRViz();

protected:
  QLineEdit * output_status_editor_{nullptr};
  QLabel * robot_location_{nullptr};
  QLabel * selected_robot_{nullptr};
  QComboBox * robot_container_{nullptr};
  QString * output_status_{nullptr};
  QHBoxLayout * topic_layout_{nullptr};
  QVBoxLayout * main_layout_{nullptr};
  QVBoxLayout * side_layout_{nullptr};
  QPushButton * start_rviz_{nullptr};
  QStringList robot_list_;

  bool process_combo_ = false;
  std::string robot_name_;
  std::thread thread_func;
};

}  // namespace neo_fleet
#endif  // NEOFLEETRVIZ2PLUGIN_HPP_
