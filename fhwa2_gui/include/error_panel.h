/*
    Panel to choose which reference and target topics to use
*/
#ifndef ERROR_PANEL_H
#define ERROR_PANEL_H

#include <string>
#include <ros/ros.h>
#include <rviz/panel.h>

class QComboBox;

namespace fhwa2_gui {

class PlotWidget;

class ErrorPanel: public rviz::Panel {
Q_OBJECT
public:
    // Constructor
    ErrorPanel( QWidget* parent = 0);

    // Overrides of rviz::Panel for topic subscriptions
    // virtual void saveToConfig( const std::string& key_prefix,
    //                            const boost::shared_ptr<rviz::Config>& config );
    // virtual void loadFromConfig( const std::string& key_prefix,
    //                              const boost::shared_ptr<rviz::Config>& config );

public Q_SLOTS:
    // Receives from the combobox
    void setTargetTopic( int cb_index );

Q_SIGNALS:
    // send new datapoint to the plotter
    void updateValue( float t, float v );

protected:
    PlotWidget* plot_widget_;
    QComboBox* target_topic_combobox_;
       
    // resulting topic to send to the error plotter for subscription
    std::string error_topic; 
};

}; // end fhwa2_gui namespace
#endif // ERROR_PANEL_H