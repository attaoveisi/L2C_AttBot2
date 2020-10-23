#ifndef FLEXIPIESERIES_H
#define FLEXIPIESERIES_H

#include "FlexiAbstractSeries.h"

class FlexiPieSeries : public FlexiAbstractSeries
{
  
public:
    FlexiPieSeries(const char * series_name);

    void setValue(long value);
    void setValue(int value);
    void setValue(float value);
    void setValue(double value);

    double value() { return m_value; }
    
    void clear();
    
private:
    double m_value;
};

#endif
