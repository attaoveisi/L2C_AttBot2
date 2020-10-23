#ifndef FLEXIBARSERIES_H
#define FLEXIBARSERIES_H

#include "FlexiAbstractSeries.h"

typedef struct flexi_bar_data_t flexi_bar_data_t;
struct flexi_bar_data_t
{
    double value;
    struct flexi_bar_data_t * next;
};

class FlexiBarSeries : public FlexiAbstractSeries
{
  
public:
    FlexiBarSeries(const char * series_name);

    FlexiBarSeries* setValue(uint8_t col_index, long value);
    FlexiBarSeries* setValue(uint8_t col_index, int value);
    FlexiBarSeries* setValue(uint8_t col_index, float value);
    FlexiBarSeries* setValue(uint8_t col_index, double value);

    flexi_bar_data_t* data() { return m_data; }

    uint8_t count();
    
    void clear();

    //These should not be used by the end user only by FlexiBarPlot class.
    void addColumn();
    void removeColumn(uint8_t index);


private:
    void addDataToLinkedList(flexi_bar_data_t * el);

    
private:
    flexi_bar_data_t* m_data;
    uint8_t dataCount;
};

#endif
