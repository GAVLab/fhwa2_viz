#ifndef FHWA2_MENU_PANEL_H
#define FHWA2_MENU_PANEL_H

#include <string>

#include <ros/ros.h>

#include <rviz/panel.h>

namespace fhwa2_gui {

class FHWA2MenuWidget;

class FHWA2MenuPanel : public rviz::Panel {
Q_OBJECT

public:
    FHWA2MenuPanel( QWidget* parent = 0);

    virtual void saveToConfig( const std::string& key_prefix, const boost::shared_ptr<rviz::Config>& config );
    virtual void loadFromConfig( const std::string& key_prefix, const boost::shared_ptr<rviz::Config>& config );

protected:
    FHWA2MenuWidget* menu_widget_;

    std::string target_sensor_;
};

}; // end namespace
#endif // FHWA2_MENU_PANEL_H