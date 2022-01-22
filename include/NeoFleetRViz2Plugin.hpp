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

#include "rviz_common/display_context.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "rviz_common/config.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_default_plugins/tools/goal_pose/goal_tool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "QPushButton"
#include "QThread"
#include "QVBoxLayout"
#include "QStringList"
#include "QComboBox"
#include "QLabel"
#include "vector"
#include "memory"
#include "string"


class QLineEdit;
namespace neo_fleet
{

class RosHelper
{
private:
  rclcpp::Node::SharedPtr node;

public:
  geometry_msgs::msg::PoseWithCovariance m_pose;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_pub_loc_pose;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pub_goal_pose;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;

  bool m_robotLocalization = false;
  bool m_goalSent = false;

  std::string robot_name;

  // Default Constructor
  RosHelper() {}

  RosHelper(rclcpp::Node::SharedPtr node_, std::string robot_name_)
  {
    node = node_;
    robot_name = robot_name_;
    m_pub_loc_pose = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/" + robot_name + "/initialpose", 10);
    m_pub_goal_pose = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/" + robot_name + "/goal_pose", 10);
    odom_subscriber = node->create_subscription<nav_msgs::msg::Odometry>(
      "/" + robot_name +
      "/odom", 1, std::bind(&RosHelper::map_pose_callback, this, std::placeholders::_1));
    navigation_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      node, "/" + robot_name + "/navigate_to_pose");
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
  void map_pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose)
  {
    m_pose = pose->pose;
  }
};


class Worker : public QObject
{
  Q_OBJECT

public:
  Worker();
  ~Worker();
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_initial_pose;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_goal_pose;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m_pose;
  geometry_msgs::msg::PoseStamped::SharedPtr m_goal;

  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose);
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("neo_fleet_thread");

  std::vector<std::shared_ptr<RosHelper>> m_robots;

  std::map<std::string, std::shared_ptr<RosHelper>> m_named_robot; 

  // std::shared_ptr<RosHelper> Robot1 = std::make_shared<RosHelper>(node, "mpo_7000");
  // std::shared_ptr<RosHelper> Robot2 = std::make_shared<RosHelper>(node, "mpo_7001");
  std::vector<std::string> robot_namespaces;

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

  std::shared_ptr<RosHelper> robot_tmp;

public slots:
  void update();

protected Q_SLOTS:
  void setRobotName();

  void setX();

  void setY();

  void setTheta();

  void handleButton1();

  void handleButton2();

  void subscribe_topics();

  // Here we declare some internal slots.

protected:
  // One-line text editor for displaying the name of the robot.
  QLineEdit * output_status_editor_;
  QLabel * X_loc_value = new QLabel;
  QLabel * selected_robot = new QLabel;
  bool mLocalization = false;
  bool m_localization_done = false;
  QVBoxLayout * layout1 = new QVBoxLayout;


  // The current name of the output topic.
  QString output_status_;

  // List of robots available
  QStringList robot_list;

  QComboBox * combo;

  std::string robot_name;

  std::thread thread_func;

  bool process_combo = false;
};

}  // namespace neo_fleet
#endif  // NEOFLEETRVIZ2PLUGIN_HPP_
