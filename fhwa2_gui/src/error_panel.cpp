#include <stdio.h>

#include "error_panel.h"
#include "plot_widget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QComboBox>

namespace fhwa2_gui {

ErrorPanel::ErrorPanel( QWidget* parent )
    : rviz::Panel( parent )
{
    // Menu
    QVBoxLayout* menu_layout = new QVBoxLayout;
    menu_layout->addWidget( new QLabel( "Target Sensor:" ) );
    target_topic_combobox_ = new QComboBox;
    // Do more combobox stuff
    menu_layout->addWidget( target_topic_combobox_ );

    // plot_widget_ = new PlotWidget;

    // WHole thing
    QHBoxLayout* layout = new QHBoxLayout;
    layout->addWidget( plot_widget_ );
    layout->addLayout( menu_layout );

    setLayout( layout );

    // Connect stuff;
}

void ErrorPanel::setTargetTopic( int cb_index ) {
    // decide what the error topic should be based on the index
    // call the set error topic signal
    return;
}

// void saveToConfig( const std::string& key_prefix,
//                    const boost::shared_ptr<rviz::Config>& config ) {
//     return;
// }

// void loadFromConfig( const std::string& key_prefix,
//                      const boost::shared_ptr<rviz::Config>& config ) {
//     return;
// }

} //end namespace fhwa2_gui

// Notify pluginlib about this class
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( fhwa2_gui, Error, fhwa2_gui::ErrorPanel, rviz::Panel )