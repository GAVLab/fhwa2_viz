#ifndef PLOT_WIDGET_H
#define PLOT_WIDGET_H

#include <QWidget>
#include <queue>


namespace fhwa2_gui {

class PlotWidget: public QWidget {
    Q_OBJECT
public:
    // Constructor
    PlotWidget( QWidget* parent = 0 );

public Q_SLOTS:
    // receive new info from whatever is subscribing, then pops and pushes
    void updateData( float t, float e );

protected:
    // Puts new data on the screen
    void updatePlot();

    // The datapoints currently being displayed 
    // at each step data[step][0] = time, data[step][1] = value
    std::deque< std::vector<float> > data; 
    // The length of vs and ts
    uint datapoint_window;

    // Error value (y axis) limits in meters
    float vmax;
    float vmin;

    // Plot object
    // QwtPlot *errorPlot;
};

}; //end fhwa2_gui namespace

#endif // PLOT_WIDGET_H