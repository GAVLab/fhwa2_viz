/*
Error Panel

Robert Cofield
*/
#ifndef ERROR_PANEL_H
#define ERROR_PANEL_H

#include <string>
#include <ros/ros.h>
#include <rviz/panel.h>

namespace fhwa2_gui {

// the actual plot of error over time
class ErrorPlot; 

// holds the plot of the error over time
class ErrorPanel: public rviz::Panel {
// needs this macro for slots and is QObject subclass
Q_OBJECT

public:
    // Constructor
    ErrorPanel( QWidget* parent = 0 );

    // Overrides of rviz::Panel for topic subscriptions
    virtual void saveToConfig( const std::string& key_prefix,
                               const boost::shared_ptr<rviz::Config>& config );
    virtual void loadFromConfig( const std::string& key_prefix,
                                 const boost::shared_ptr<rviz::Config>& config );

public Q_SLOTS:
    // Receives the topic from the menu
    void setTopic( const std::string& topic );
};

}; // end fhwa2_gui namespace

#endif // ERROR_PANEL_H