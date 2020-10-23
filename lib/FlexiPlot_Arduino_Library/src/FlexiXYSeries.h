#ifndef FLEXIXYSERIES_H
#define FLEXIXYSERIES_H

#include "FlexiAbstractSeries.h"

typedef struct flexi_xy_data_t flexi_xy_data_t;
struct flexi_xy_data_t
{
    double x;
    double y;
    struct flexi_xy_data_t * next;
};

class FlexiXYSeries : public FlexiAbstractSeries
{
public:
    FlexiXYSeries(const char * series_name);
    
    FlexiXYSeries* addData(int y);
    FlexiXYSeries* addData(long y);
    FlexiXYSeries* addData(double y);
    FlexiXYSeries* addData(float y);
    FlexiXYSeries* addData(int x, int y);    
    FlexiXYSeries* addData(long x, long y);
    FlexiXYSeries* addData(float x, float y);
    FlexiXYSeries* addData(double x, double y);

    flexi_xy_data_t* data() { return m_data; }

    uint8_t count();
    
    void clear();

private:
    void addDataToLinkedList(flexi_xy_data_t * el);

private:
    flexi_xy_data_t* m_data;
    uint8_t dataCount;
};

#endif
