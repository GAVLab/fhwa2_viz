#include <stdio.h>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>

#include "fhwa2_menu_panel.h"

namespace fhwa2_gui {

    FHWA2MenuPanel::FHWA2MenuPanel( QWidget* parent )
        : rviz::Panel( parent )
        , target_sensor_( "gps" )
}