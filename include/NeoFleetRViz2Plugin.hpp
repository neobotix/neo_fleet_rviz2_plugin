#ifndef INCLUDE_NEOFLEETRVIZ2PLUGIN_HPP_
#define INCLUDE_NEOFLEETRVIZ2PLUGIN_HPP_

#include "rviz_common/display_context.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include "rviz_default_plugins/tools/goal_pose/goal_tool.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <QPushButton>
#include <QThread>
#include <QVBoxLayout>
#include <QLabel>


class QLineEdit;
namespace neo_fleet
{

class RosHelper {
private:
  rclcpp::Node::SharedPtr node;

  std::string robot_name;

public:
    RosHelper(rclcpp::Node::SharedPtr node1, std::string robot_name_ ){
        node = node1;
        robot_name = robot_name_;
        map_pose = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>
        (robot_name + "/map_pose", 1, std::bind(&RosHelper::map_pose_callback, this, std::placeholders::_1));
    }
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr map_pose;
    void map_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr /*pose*/) {
        
    }
};


class Worker : public QObject {
    Q_OBJECT
 
public:
    Worker();
    ~Worker();
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_initial_pose;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m_pose;
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose);
    
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("neo_fleet_thread");

    RosHelper Robot1{node, "robot1"};
    RosHelper Robot2{node, "robot2"};


 
public slots:
    void process();
 
signals:
    void finished();
    void data_recieved();
    void error(QString err);
 
private:
    // add your variables here
};

class NeoFleetRViz2Plugin: public rviz_common::Panel
{
Q_OBJECT

public:
  NeoFleetRViz2Plugin( QWidget* parent = 0 );

  ~NeoFleetRViz2Plugin();

  virtual void load( const rviz_common::Config& config );
  virtual void save( rviz_common::Config config ) const;

  QThread* thread = new QThread;
  Worker* worker = new Worker();

public slots:
    void update();

protected Q_SLOTS:
  void setRobotName();

  void setX();

  void setY();

  void setTheta();  

  void handleButton1();

  void handleButton2();

  void ProcessRobot(QString text);

  void subscribe_topics();
    
  // Here we declare some internal slots.
protected:
  // One-line text editor for displaying the name of the robot.
  QLineEdit* output_status_editor_;

  QLabel* X_loc_value = new QLabel;
  QLabel* loc_status = new QLabel;
  QLabel* goal_status = new QLabel;

  bool mLocalization = false;

  bool m_localization_done = false;


  QVBoxLayout* layout1 = new QVBoxLayout;


  // The current name of the output topic.
  QString output_status_;

  std::string robot_name;

  std::thread thread_func;
};

} // neo_fleet_rviz2_plugin namespace

#endif // MISSION_PANEL_H