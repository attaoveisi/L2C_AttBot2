#ifndef FLEXIPIECHART_H
#define FLEXIPIECHART_H

#include "FlexiPlotBase.h"
#include "FlexiPieSeries.h"

class FlexiPieChart : public FlexiPlotBase
{
public:
    FlexiPieChart(const char* id = "P0");
    
    FlexiPieSeries* addSeries(const char * name);
    FlexiPieSeries* series(uint8_t index);
    FlexiPieSeries* seriesByName(const char * name);

    void plot();
};

#endif
