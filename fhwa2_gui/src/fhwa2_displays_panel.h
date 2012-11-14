#ifndef FHWA2_MENU_PANEL_H
#define FHWA2_MENU_PANEL_H

#include <string>

#include <ros/ros.h>

#include <rviz/panel.h>
#include <rviz/displays_panel.h>

namespace fhwa2_gui {

class QComboBox;

class FHWA2DisplaysPanel : public rviz::Panel {
Q_OBJECT

public:
    FHWA2DisplaysPanel( QWidget* parent = 0);
    virtual ~FHWA2DisplaysPanel();

protected Q_SLOTS:
    // receives from 
    void updateTargetSensor(QString sensor);


protected:
    rviz::DisplaysPanel displays_panel_;
    std::string target_sensor_;
    QComboBox* target_selector_;
};

}; // end namespace
#endif // FHWA2_MENU_PANEL_H