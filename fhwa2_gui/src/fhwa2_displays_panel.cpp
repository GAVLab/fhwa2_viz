#include <stdio.h>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QString>

#include "fhwa2_displays_panel.h"

namespace fhwa2_gui {

    FHWA2DisplaysPanel::FHWA2DisplaysPanel( QWidget* parent )
        : rviz::Panel( parent )
        , target_sensor_( "GPS" )
    {
        QVBoxLayout* layout = new QVBoxLayout;

        target_selector_ = new QComboBox;
        target_selector_->addItem("GPS");
        target_selector_->addItem("FHWA2 Combined");

        displays_panel_ = new rviz::DisplaysPanel;
        layout->addWidget(new QLabel("Show Target Sensor:"));
        layout->addWidget(target_selector_);
        setLayout(layout);

        connect(target_selector_, SIGNAL( currentIndexChanged(QString) ),
                this, SLOT( updateTargetSensor(QString) ));
    }

void FHWA2DisplaysPanel::updateTargetSensor(QString sensor) {
    if (sensor == "FHWA2 Comnbined")
        // do FHWA2 stuff
        return;
    else
        // do GPS stuff
        return;
}

}; // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( fhwa2_gui, FHWA2Displays, fhwa2_gui::FHWA2DisplaysPanel, rviz::Panel)