#ifndef INCLUDE_NEOFLEETRVIZ2PLUGIN_HPP_
#define INCLUDE_NEOFLEETRVIZ2PLUGIN_HPP_

#include "rviz_common/display_context.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>

class QLineEdit;
namespace neo_fleet
{

class NeoFleetRViz2Plugin: public rviz_common::Panel
{
Q_OBJECT

public:
  NeoFleetRViz2Plugin( QWidget* parent = 0 );

  virtual void load( const rviz_common::Config& config );
  virtual void save( rviz_common::Config config ) const;


  void setRobotName(const std::string text);

  // Here we declare some internal slots.
protected:
  // One-line text editor for displaying the name of the robot.
  QLineEdit* output_status_editor_;

  // The current name of the output topic.
  QString output_status_;
};

} // neo_fleet_rviz2_plugin namespace

#endif // MISSION_PANEL_H