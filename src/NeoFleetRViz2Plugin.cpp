/*
 * neo_localization_node.cpp
 *
 *  Created on: Sep 23, 2021
 *      Author: Pradheep Padmanabhan
 */
#include "../include/NeoFleetRViz2Plugin.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QComboBox>

namespace neo_fleet
{

// --- CONSTRUCTOR ---
Worker::Worker() {
    // you could copy data from constructor arguments to internal variables here.
}
 
// --- DECONSTRUCTOR ---
Worker::~Worker() {
    // free resources
}
 
// --- PROCESS ---
// Start processing data.
void Worker::process() {
    // allocate resources using new here
    std::map<std::string, std::vector<std::string> > get_topic = node->get_topic_names_and_types(); 
    std::string robots = "";
    std::string tmp = "";

    // Store the robots for the drop down list
    for (auto it = get_topic.begin(); it != get_topic.end(); it++) {
      int dslash = 0;
      for (long unsigned int i = 0; i < (it->first).size(); i++) {
        robots += it->first[i];
        if (it->first[i] == '/' && dslash != 2) {
            dslash++;
            if(dslash == 2 and tmp != robots) {
              robot_namespaces.push_back(robots);
              tmp = robots;
            }
        }
      }
      robots = "";
    }

    m_initial_pose = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1, std::bind(&Worker::pose_callback, this, std::placeholders::_1));
    rclcpp::Rate loop_rate(10);
    while(rclcpp::ok()) {
      loop_rate.sleep();   
      rclcpp::spin_some(node);
      emit finished();
    }
}

NeoFleetRViz2Plugin::NeoFleetRViz2Plugin( QWidget* parent )
  : rviz_common::Panel( parent )

{
  QHBoxLayout* topic_layout = new QHBoxLayout;
  output_status_editor_ = new QLineEdit;
  QTimer* output_timer = new QTimer( this );

  combo = new QComboBox(this);
  robot_tmp = worker->Robot1;

  // Creating buttons
  QPushButton* m_button1 = new QPushButton("Localization Mode", this);
  QPushButton* m_button2 = new QPushButton("Goal Mode", this);
  QPushButton* m_button3 = new QPushButton("Select Goal Pose", this);

  topic_layout->addWidget( new QLabel( "Select the target robot:" ));
  topic_layout->addWidget( combo );
  topic_layout->addWidget( m_button1 );
  topic_layout->addWidget( m_button2 );
  // Connect button signal to appropriate slot
  connect(m_button1, &QPushButton::released, this, &NeoFleetRViz2Plugin::handleButton1);
  connect(m_button2, &QPushButton::released, this, &NeoFleetRViz2Plugin::handleButton2);

  // Lay out the topic field above the control widget.
  connect( combo, QOverload<int>::of(&QComboBox::activated), this, &NeoFleetRViz2Plugin::setRobotName );
  // connect( output_timer, SIGNAL( timeout() ), this, SLOT( setRobotName(std::string) ));

  // Start the timer.
  output_timer->start( 100 );

  QVBoxLayout* layout = new QVBoxLayout;

  layout->addLayout( topic_layout );
  layout1->addWidget( m_button3 );
  layout1->addWidget( loc_status );
  loc_status->setText("Localization: Not done");
  layout1->addWidget( X_loc_value );
  layout1->addWidget(selected_robot);
  goal_status->setText("Goal: Waiting for Localization to be complete");
  layout1->addWidget( goal_status ); 
  layout->addLayout( layout1 );

  QHBoxLayout* status_layout = new QHBoxLayout;
  status_layout->addWidget( new QLabel( "Robot Status:" ));
  QLineEdit* output_status_editor_status = new QLineEdit;
  status_layout->addWidget( output_status_editor_status );
  layout->addLayout( status_layout );

  setLayout( layout );

  worker->moveToThread(thread);
  connect(worker, SIGNAL(error(QString)), this, SLOT(errorString(QString)));
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

void Worker::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose ) {
  m_pose = pose;
}

void NeoFleetRViz2Plugin::setRobotName() {
   robot_name = combo->currentText().toStdString();
   if( robot_name == "/mp_4001/") 
   {
    robot_tmp = worker->Robot2;
   } else {
       robot_tmp = worker->Robot1;
   }

}

void NeoFleetRViz2Plugin::update() {

  if(!process_combo) {
    for (long unsigned int i = 0; i < worker->robot_namespaces.size(); i++)
    {
        robot_list.push_back(QString::fromStdString(worker->robot_namespaces[i]));
    }
    combo->addItems(robot_list);
    process_combo = true;
  }

  if(mLocalization) {
    if(!worker->m_pose)
    {
      X_loc_value->setText("X: " + QString::number(0) + ", Y: " + QString::number(0) + ", Theta: " + QString::number(0));
    }
    else {
      geometry_msgs::msg::PoseWithCovarianceStamped pub_pose;
      loc_status->setText("Localization: Robot Localized");
      goal_status->setText("Goal: Ready to send goals");
      X_loc_value->setText("X: " + QString::number(worker->m_pose->pose.pose.position.x) 
        + ", Y: " + QString::number(worker->m_pose->pose.pose.position.y)
        + ", Theta: " + QString::number(worker->m_pose->pose.pose.orientation.z));
      pub_pose = *worker->m_pose;
      robot_tmp->m_pub_loc_pose->publish(pub_pose);
      m_localization_done = true;
      robot_tmp->m_robotLocalization = true;
      mLocalization = false;
    }
  }

  if (robot_tmp->m_robotLocalization) {
    selected_robot->setText(QString::fromStdString(robot_tmp->robot_name));
    loc_status->setText("Localization: Robot Localized");
      goal_status->setText("Goal: Ready to send goals");
      X_loc_value->setText("X: " + QString::number(robot_tmp->m_pose.pose.position.x) 
        + ", Y: " + QString::number(robot_tmp->m_pose.pose.position.y)
        + ", Theta: " + QString::number(robot_tmp->m_pose.pose.orientation.z));
  }
}

void NeoFleetRViz2Plugin::handleButton1()
{
  if(robot_name.size()==0) {
    loc_status->setText("Localization: Robot not selected");
  }

  else {
    mLocalization = true;
    loc_status->setText("Localization: Robot ready for Localization");
  }

}

void NeoFleetRViz2Plugin::handleButton2()
{
  if(!m_localization_done) {
    goal_status->setText("Goal: Waiting for Localization to be complete");
  } else {
    goal_status->setText("Goal: Ready to send goals");
  }

}

void NeoFleetRViz2Plugin::save( rviz_common::Config config ) const
{
  rviz_common::Panel::save( config );
}

// Load all configuration data for this panel from the given Config object.
void NeoFleetRViz2Plugin::load( const rviz_common::Config& config )
{
  rviz_common::Panel::load( config );
}

} // namespace neo_fleet


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(neo_fleet::NeoFleetRViz2Plugin, rviz_common::Panel)