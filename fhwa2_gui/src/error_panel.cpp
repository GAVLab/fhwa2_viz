#include <stdio.h>

#include "error_panel.h"
// #include "error_plot.h"

#include <QVBoxLayout>
#include <QLabel>
#include <QComboBox>

#include <qwt/qwt_plot.h>

namespace fhwa2_gui {

ErrorPanel::ErrorPanel( QWidget* parent )
    : rviz::Panel( parent )
{
    QVBoxLayout* layout = new QVBoxLayout;
    layout->addWidget( new QLabel( "Target Sensor:" ) );
    target_topic_combobox_ = new QComboBox;
    // Do more combobox stuff
    layout->addWidget( target_topic_combobox_ );
    setLayout( layout );

    // Connect stuff;
}

void ErrorPanel::setTargetTopic( int cb_index ) {
    // decide what the error topic should be based on the index
    // call the set error topic signal
    return;
}

void saveToConfig( const std::string& key_prefix,
                   const boost::shared_ptr<rviz::Config>& config ) {
    return;
}

void loadFromConfig( const std::string& key_prefix,
                     const boost::shared_ptr<rviz::Config>& config ) {
    return;
}

} //end namespace fhwa2_gui

// Notify pluginlib about this class
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( fhwa2_gui, Menu, fhwa2_gui::ErrorPanel, rviz::Panel )