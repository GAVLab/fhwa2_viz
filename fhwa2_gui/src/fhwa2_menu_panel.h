#ifndef FHWA2_MENU_PANEL_H
#define FHWA2_MENU_PANEL_H

#include <string>

#include <ros/ros.h>

#include <rviz/panel.h>

class QComboBox;

namespace fhwa2_gui {

class FHWA2MenuPanel : public rviz::Panel {
Q_OBJECT

public:
    FHWA2MenuPanel( QWidget* parent = 0);

protected Q_SLOTS:
    // receives from 
    void updateTargetSensor(QString sensor);


protected:
    std::string target_sensor_;

    QComboBox* target_selector_;
};

}; // end namespace
#endif // FHWA2_MENU_PANEL_H