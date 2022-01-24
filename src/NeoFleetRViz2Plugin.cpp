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

// --- PROCESS ---
// Start processing data.
void Worker::process()
{
  std::map<std::string, std::vector<std::string>> get_topic = node->get_topic_names_and_types();
  std::string robots = "";
  std::string tmp = "";

  // Store the robots for the drop down list
  for (auto it = get_topic.begin(); it != get_topic.end(); it++) {
    int dslash = 0;
    for (long unsigned int i = 0; i < (it->first).size(); i++) {
      if (it->first[i] != '/') {
        robots += it->first[i];
      }
      if (it->first[i] == '/' && dslash != 2) {
        dslash++;
        if (dslash == 2 && tmp != robots) {
          robot_namespaces.push_back(robots);
          tmp = robots;
        }
      }
    }
    robots = "";
  }

  // Allocating ros helpers depending on the number of robots available.
  if(robot_namespaces.size() == 0) {
    RCLCPP_ERROR(
      node->get_logger(),
      "There are no robots available"
      );
    return;
  }

  m_robots.resize(robot_namespaces.size());

  for (long unsigned int i = 0; i < m_robots.size(); i++) {
    m_robots[i] = std::make_shared<RosHelper>(node, robot_namespaces[i]);
    m_named_robot.insert(std::pair<std::string, std::shared_ptr<RosHelper>> (robot_namespaces[i], m_robots[i]));
  }

  m_initial_pose = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 1, std::bind(&Worker::pose_callback, this, std::placeholders::_1));
  m_goal_pose = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 1, std::bind(&Worker::goal_callback, this, std::placeholders::_1));

  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok()) {
    loop_rate.sleep();
    rclcpp::spin_some(node);
    emit finished();
  }
}

NeoFleetRViz2Plugin::NeoFleetRViz2Plugin(QWidget * parent)
: rviz_common::Panel(parent)
{
  QHBoxLayout * topic_layout = new QHBoxLayout;
  output_status_editor_ = new QLineEdit;
  QTimer * output_timer = new QTimer(this);

  QPushButton * m_button1 = new QPushButton("RViz", this);

  combo = new QComboBox(this);
  // robot_selected = worker->m_robots[0];

  topic_layout->addWidget(new QLabel("Select the target robot:"));
  topic_layout->addWidget(combo);
  topic_layout->addWidget(m_button1);

  // Lay out the topic field above the control widget.
  connect(
    combo, QOverload<int>::of(&QComboBox::activated), this,
    &NeoFleetRViz2Plugin::setRobotName);
  connect(m_button1, &QPushButton::released, this, &NeoFleetRViz2Plugin::handleButton1);

  // Start the timer.
  output_timer->start(100);

  QVBoxLayout * layout = new QVBoxLayout;

  layout->addLayout(topic_layout);
  layout1->addWidget(X_loc_value);
  layout1->addWidget(selected_robot);
  layout->addLayout(layout1);

  setLayout(layout);

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
  m_pose = pose;
}

void Worker::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  m_goal = pose;
}

void NeoFleetRViz2Plugin::setRobotName()
{
  robot_name = combo->currentText().toStdString();

  // Searching and assigning the corresponding pointers for selected robot
  auto search = worker->m_named_robot.find(robot_name);

  if (search != worker->m_named_robot.end()) {
    robot_selected = search->second;
  } else {
    RCLCPP_ERROR(
      worker->node->get_logger(),
      "Robot not found in the drop down"
      );
  }
}

void NeoFleetRViz2Plugin::update()
{
  if (!process_combo) {
    for (long unsigned int i = 0; i < worker->robot_namespaces.size(); i++) {
      robot_list.push_back(QString::fromStdString(worker->robot_namespaces[i]));
    }
    combo->addItems(robot_list);
    process_combo = true;
  }

  if(robot_selected == NULL) {
    return;
  }

  if (!robot_selected->m_robotLocalization) {
    if (!worker->m_pose) {
      X_loc_value->setText(
        "X: " + QString::number(0) + ", Y: " + QString::number(
          0) + ", Theta: " + QString::number(0));
      selected_robot->setText("Selected Robot: " + QString::fromStdString(robot_selected->robot_name));
    } else {
      geometry_msgs::msg::PoseWithCovarianceStamped pub_pose;
      selected_robot->setText("Selected Robot: " + QString::fromStdString(robot_selected->robot_name));
      X_loc_value->setText(
        "X: " + QString::number(worker->m_pose->pose.pose.position.x) +
        ", Y: " + QString::number(worker->m_pose->pose.pose.position.y) +
        ", Theta: " + QString::number(worker->m_pose->pose.pose.orientation.z));
      pub_pose = *worker->m_pose;
      robot_selected->m_pub_loc_pose->publish(pub_pose);
      m_localization_done = true;
      robot_selected->m_robotLocalization = true;
      worker->m_pose = NULL;
    }
  }

  if (robot_selected->m_robotLocalization) {
    geometry_msgs::msg::PoseStamped pub_goal_pose;
    selected_robot->setText("Selected Robot: " + QString::fromStdString(robot_selected->robot_name));
    X_loc_value->setText(
      "X: " + QString::number(robot_selected->m_pose.pose.position.x) +
      ", Y: " + QString::number(robot_selected->m_pose.pose.position.y) +
      ", Theta: " + QString::number(robot_selected->m_pose.pose.orientation.z));

    if (worker->m_goal && robot_selected->m_goalSent == false) {
      pub_goal_pose = *worker->m_goal;
      auto check_action_server_ready =
        robot_selected->navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
      if (!check_action_server_ready) {
        RCLCPP_ERROR(
          worker->node->get_logger(),
          "navigate_to_pose action server is not available."
        );
        return;
      }

      robot_selected->navigation_goal_.pose = pub_goal_pose;

      auto send_goal_options =
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
      send_goal_options.result_callback = [this](auto) {
          robot_selected->m_goalSent = false;
        };

      auto future_goal_handle =
        robot_selected->navigation_action_client_->async_send_goal(
        robot_selected->navigation_goal_,
        send_goal_options);
      robot_selected->m_goalSent = true;
      worker->m_goal = NULL;
    }
  }
}

void NeoFleetRViz2Plugin::handleButton1()
{
  std::string command =
    "ros2 launch neo_simulation2 rviz_launch.py use_namespace:=true namespace:=";
  command.append(robot_selected->robot_name);
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
