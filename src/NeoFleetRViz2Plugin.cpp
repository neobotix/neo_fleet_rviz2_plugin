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

#include "../include/NeoFleetRViz2Plugin.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.h>

#include "string"
#include "map"
#include "vector"

#include "QPainter"
#include "QLineEdit"
#include "QVBoxLayout"
#include "QHBoxLayout"
#include "QLabel"
#include "QTimer"
#include "QComboBox"

namespace neo_fleet
{

// --- CONSTRUCTOR ---
Worker::Worker()
{
}

// --- DECONSTRUCTOR ---
Worker::~Worker()
{
}

void Worker::checkAndStoreRobot(std::string & robot)
{
  // Storing the robot name in another tmp variable
  std::string tmp = robot;
  tmp.pop_back();
  for (int i = 0; i < available_robots_.size(); i++) {
    if (tmp == available_robots_[i]) {
      robot_namespaces_.push_back(robot);
    }
  }
}

// --- PROCESS ---
// Start processing data.
void Worker::process()
{
  std::map<std::string, std::vector<std::string>> get_topic = node_->get_topic_names_and_types();
  std::string robots = "";
  std::string tmp = "";

  // Store the robots for the drop down list
  for (auto it = get_topic.begin(); it != get_topic.end(); it++) {
    int dslash = 0;
    for (int i = 0; i < (it->first).size(); i++) {
      if (it->first[i] != '/') {
        robots += it->first[i];
      }
      if (it->first[i] == '/' && dslash != 2) {
        dslash++;
        if (dslash == 2 && tmp != robots) {
          checkAndStoreRobot(robots);
          tmp = robots;
        }
      }
    }
    robots = "";
  }

  // Allocating ros helpers depending on the number of robots available.
  if (robot_namespaces_.size() == 0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "There are no robots available"
    );
    return;
  }

  robots_.resize(robot_namespaces_.size());

  for (int i = 0; i < robots_.size(); i++) {
    robots_[i] = std::make_shared<RosHelper>(node_, robot_namespaces_[i]);
    robot_identity_map_.insert(
      std::pair<std::string, std::shared_ptr<RosHelper>>(
        robot_namespaces_[i],
        robots_[i]));
  }

  initial_pos_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 1, std::bind(&Worker::pose_callback, this, std::placeholders::_1));
  goal_pos_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 1, std::bind(&Worker::goal_callback, this, std::placeholders::_1));

  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok()) {
    loop_rate.sleep();
    rclcpp::spin_some(node_);
    emit finished();
  }
}

NeoFleetRViz2Plugin::NeoFleetRViz2Plugin(QWidget * parent)
: rviz_common::Panel(parent)
{
  topic_layout_ = new QHBoxLayout;
  output_status_editor_ = new QLineEdit;
  QTimer * output_timer = new QTimer(this);
  start_rviz_ = new QPushButton("RViz", this);
  robot_container_ = new QComboBox(this);

  topic_layout_->addWidget(new QLabel("Select the target robot:"));
  topic_layout_->addWidget(robot_container_);
  topic_layout_->addWidget(start_rviz_);

  // Lay out the topic field above the control widget.
  connect(
    robot_container_, QOverload<int>::of(&QComboBox::activated), this,
    &NeoFleetRViz2Plugin::setRobotName);
  connect(start_rviz_, &QPushButton::released, this, &NeoFleetRViz2Plugin::launchRViz);

  // Start the timer.
  output_timer->start(100);

  main_layout_->addLayout(topic_layout_);
  side_layout_->addWidget(robot_location_);
  side_layout_->addWidget(selected_robot_);
  main_layout_->addLayout(side_layout_);

  setLayout(main_layout_);

  worker->moveToThread(thread);
  // connect(worker, SIGNAL(error(QString)), this, SLOT(errorString(QString)));
  connect(thread, SIGNAL(started()), worker, SLOT(process()));
  connect(worker, &Worker::finished, this, &NeoFleetRViz2Plugin::update);
  connect(worker, SIGNAL(finished()), thread, SLOT(quit()));
  connect(worker, SIGNAL(finished()), worker, SLOT(deleteLater()));
  connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
  thread->start();
}

NeoFleetRViz2Plugin::~NeoFleetRViz2Plugin()
{
}

void Worker::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose)
{
  initial_pose_ = pose;
}

void Worker::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  goal_pose_ = pose;
}

void NeoFleetRViz2Plugin::setRobotName()
{
  robot_name_ = robot_container_->currentText().toStdString();

  // Searching and assigning the corresponding pointers for selected robot
  auto search = worker->robot_identity_map_.find(robot_name_);

  if (search != worker->robot_identity_map_.end()) {
    robot_ = search->second;
  } else {
    RCLCPP_ERROR(
      worker->node_->get_logger(),
      "Robot not found in the drop down"
    );
  }
}

void NeoFleetRViz2Plugin::update()
{
  if (!process_combo_) {
    for (int i = 0; i < worker->robot_namespaces_.size(); i++) {
      robot_list_.push_back(QString::fromStdString(worker->robot_namespaces_[i]));
    }
    robot_container_->addItems(robot_list_);
    process_combo_ = true;
  }

  if (robot_ == NULL) {
    return;
  }

  if (!robot_->is_localized_) {
    if (!worker->initial_pose_) {
      robot_location_->setText(
        "X: " + QString::number(0) + ", Y: " + QString::number(
          0) + ", Theta: " + QString::number(0));
      selected_robot_->setText(
        "Selected Robot: " + QString::fromStdString(
          robot_->robot_name_));
    } else {
      geometry_msgs::msg::PoseWithCovarianceStamped pub_pose;
      selected_robot_->setText(
        "Selected Robot: " +
        QString::fromStdString(robot_->robot_name_));
      robot_location_->setText(
        "X: " + QString::number(worker->initial_pose_->pose.pose.position.x) +
        ", Y: " + QString::number(worker->initial_pose_->pose.pose.position.y) +
        ", Theta: " + QString::number(worker->initial_pose_->pose.pose.orientation.z));
      pub_pose = *worker->initial_pose_;
      robot_->local_pos_pub_->publish(pub_pose);
      robot_->is_localized_ = true;
      worker->initial_pose_ = NULL;
    }
  }

  if (robot_->is_localized_) {
    geometry_msgs::msg::PoseStamped pub_goal_pose;
    selected_robot_->setText(
      "Selected Robot: " +
      QString::fromStdString(robot_->robot_name_));
    robot_location_->setText(
      "X: " + QString::number(robot_->odom_pose_.pose.position.x) +
      ", Y: " + QString::number(robot_->odom_pose_.pose.position.y) +
      ", Theta: " + QString::number(robot_->odom_pose_.pose.orientation.z));

    if (worker->goal_pose_ && robot_->is_goal_sent_ == false) {
      pub_goal_pose = *worker->goal_pose_;
      auto check_action_server_ready =
        robot_->navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
      if (!check_action_server_ready) {
        RCLCPP_ERROR(
          worker->node_->get_logger(),
          "navigate_to_pose action server is not available."
        );
        return;
      }

      robot_->navigation_goal_.pose = pub_goal_pose;

      auto send_goal_options =
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
      send_goal_options.result_callback = [this](auto) {
          robot_->is_goal_sent_ = false;
        };

      auto future_goal_handle =
        robot_->navigation_action_client_->async_send_goal(
        robot_->navigation_goal_,
        send_goal_options);
      robot_->is_goal_sent_ = true;
      worker->goal_pose_ = NULL;
    }
  }
}

void NeoFleetRViz2Plugin::launchRViz()
{
  std::string command =
    "ros2 launch neo_nav2_bringup rviz_launch.py use_namespace:=True namespace:=";
  command.append(robot_->robot_name_);
  command.append("&");

  system(command.c_str() );
}

void NeoFleetRViz2Plugin::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

// Load all configuration data for this panel from the given Config object.
void NeoFleetRViz2Plugin::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

}  // namespace neo_fleet

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(neo_fleet::NeoFleetRViz2Plugin, rviz_common::Panel)
