/*
 * neo_localization_node.cpp
 *
 *  Created on: Sep 23, 2021
 *      Author: Pradheep Padmanabhan
 */
#include "../include/NeoFleetRViz2Plugin.hpp"

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

namespace neo_fleet
{
NeoFleetRViz2Plugin::NeoFleetRViz2Plugin( QWidget* parent )
  : rviz_common::Panel( parent )

{
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Select the target robot:" ));
  output_status_editor_ = new QLineEdit;
  topic_layout->addWidget( output_status_editor_ );

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( topic_layout );
  setLayout( layout );
}


void NeoFleetRViz2Plugin::setRobotName(const std::string text) {
	QString qmsgtext(text.c_str());
	output_status_editor_->setText(qmsgtext);
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