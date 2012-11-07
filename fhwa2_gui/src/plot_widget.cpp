// #include <qwt/qwt_plot.h>
#include <stdio.h>
#include <math.h>

#include <qwt/qwt_plot.h>
#include <qwt/qwt_plot_curve.h>
#include <qwt/qwt_series_data.h>

#include "plot_widget.h"

namespace fhwa2_gui {

// TODO make these instance attributes
// Plotting Objects
// QwtPlot *errorPlot;
// QwtPlotCurve *errorCurve;
// QwtPointSeriesData* errorData;

PlotWidget::PlotWidget (QWidget* parent)
    : QWidget( parent )
    , data ( datapoint_window )
    , datapoint_window( 100 )
    , vmax( 10 )
    , vmin( 0 )
{
    // Initialize Qwt plot
    // errorPlot = new QwtPlot(this);
    // errorCurve = new QwtPlotCurve("Error Curve"); 
    // errorData = new QwtPointSeriesData;
}

void PlotWidget::updateData( float t, float e ) {
    // TODO Check for number values being right
    std::vector<float> point;
    point[0] = t;
    point[1] = e;
    data.push_back( point );
    data.pop_front();

    updatePlot();
}

void PlotWidget::updatePlot() {
    // Populate the plot data
    QVector<QPointF>* points = new QVector<QPointF>;    
    uint p;
    for (p=0; p<datapoint_window; p++)
    {
        std::vector<float> step = data[p];
        float t = step[0];
        float e = step[1];
        points->push_back( QPointF( t, e ) );
    }

    // errorData->setSamples( *points );
    // errorCurve->setData( errorData );

    // errorCurve->attach( errorPlot );
    // errorPlot->replot();

    // deallocate
    delete points;

    update(); // QWidget function that schedules repainting, may not be necessary?
}

}; // end fhwa2_gui namespace