#include <stdio.h>

#include "menu_panel.h"
// #include "error_plot.h"

#include <QVBoxLayout>
#include <QLabel>
#include <QComboBox>

namespace fhwa2_gui {

MenuPanel::MenuPanel( QWidget* parent )
    : rviz::Panel( parent )
{
    QVBoxLayout* topic_layout = new QVBoxLayout;
    topic_layout->addWidget( new QLabel( "Target Sensor:" ) );
    target_topic_combobox_ = new QComboBox;
    // Do more combobox stuff
    topic_layout->addWidget( target_topic_combobox_ );
}

void MenuPanel::setTargetTopic( int cb_index ) {
    // decide what the error topic should be based on the index
    // call the set error topic signal
    return;
}

void MenuPanel::setErrorTopic( const std::string& topic ) {
    //
    return;
}

void saveToConfig( const std::string& key_prefix,
                   const boost::shared_ptr<rviz::Config>& config ); {
    return;
}

void loadFromConfig( const std::string& key_prefix,
                     const boost::shared_ptr<rviz::Config>& config ); {
    return;
}

} //end namespace fhwa2_gui

// Notify pluginlib about this class
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( fhwa2_gui, Menu, fhwa2_gui::MenuPanel, rviz::Panel )