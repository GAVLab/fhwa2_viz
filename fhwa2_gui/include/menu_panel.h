/*
    Panel to choose which reference and target topics to use
*/
#ifndef MENU_PANEL_H
#define MENU_PANEL_H

#include <string>
#include <ros/ros.h>
#include <rviz/panel.h>

class QComboBox;

namespace fhwa2_gui {

class MenuPanel: public rviz::Panel {
Q_OBJECT
public:
    // Constructor
    MenuPanel( QWidget* parent = 0);

    // Overrides of rviz::Panel for topic subscriptions
    virtual void saveToConfig( const std::string& key_prefix,
                               const boost::shared_ptr<rviz::Config>& config );
    virtual void loadFromConfig( const std::string& key_prefix,
                                 const boost::shared_ptr<rviz::Config>& config );

public Q_SLOTS:
    // Receives from the combobox
    void setTargetTopic( int cb_index );

Q_SIGNALS:
    // Sends a the attribute error topic to the error plotter, which will then
    // subscribe to it
    void setErrorTopic( const std::string& topic );

protected:
    QComboBox* target_topic_combobox_;
    // resulting topic to send to the error plotter for subscription
    std::string error_topic; 
};

}; // end fhwa2_gui namespace
#endif // MENU_PANEL_H