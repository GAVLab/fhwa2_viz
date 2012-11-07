/*
Error Panel

Robert Cofield
*/
#ifndef ERROR_WIDGET_H
#define ERROR_WIDGET_H

#include <string>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <QWidget>

namespace fhwa2_gui {

class ErrorWidget: public QWidget {
// needs this macro for slots and is QObject subclass
Q_OBJECT

public:
    // Constructor
    ErrorWidget( QWidget* parent = 0 );
    
};

}; // end fhwa2_gui namespace

#endif // ERROR_PANEL_H