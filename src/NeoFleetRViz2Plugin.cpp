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

  // Creating buttons
  QPushButton* m_button1 = new QPushButton("Localization Mode", this);
  QPushButton* m_button2 = new QPushButton("Goal Mode", this);
  QPushButton* m_button3 = new QPushButton("Select Goal Pose", this);

  topic_layout->addWidget( new QLabel( "Select the target robot:" ));
  topic_layout->addWidget( output_status_editor_ );
  topic_layout->addWidget( m_button1 );
  topic_layout->addWidget( m_button2 );
  // Connect button signal to appropriate slot
  connect(m_button1, &QPushButton::released, this, &NeoFleetRViz2Plugin::handleButton1);
  connect(m_button2, &QPushButton::released, this, &NeoFleetRViz2Plugin::handleButton2);

  // Lay out the topic field above the control widget.
  connect( output_status_editor_, SIGNAL( editingFinished() ), this, SLOT( setRobotName() ));
  // connect( output_timer, SIGNAL( timeout() ), this, SLOT( setRobotName(std::string) ));

  // Start the timer.
  output_timer->start( 100 );

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( topic_layout );

  layout1->addWidget( m_button3 );

  layout1->addWidget( loc_status );

  loc_status->setText("Localization: Not done");

  layout1->addWidget( X_loc_value );

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
	ProcessRobot(output_status_editor_->text());
}

void NeoFleetRViz2Plugin::update() {

  if(mLocalization) {
    if(!worker->m_pose)
    {
      X_loc_value->setText("X: " + QString::number(0) + ", Y: " + QString::number(0) + ", Theta: " + QString::number(0));
    }
    else {
      loc_status->setText("Localization: Robot Localized");
      goal_status->setText("Goal: Ready to send goals");
      X_loc_value->setText("X: " + QString::number(worker->m_pose->pose.pose.position.x) 
        + ", Y: " + QString::number(worker->m_pose->pose.pose.position.y)
        + ", Theta: " + QString::number(worker->m_pose->pose.pose.orientation.z));
      m_localization_done = true;
    }
  }
}

void NeoFleetRViz2Plugin::ProcessRobot(QString text) {

  robot_name = text.toStdString(); 

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

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
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