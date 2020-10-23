#ifndef FLEXIPLOTBASE_H
#define FLEXIPLOTBASE_H

#include <Arduino.h>
#include "FlexiAbstractSeries.h"

#define ID_STR_SIZE     10

typedef struct series_t series_t;
struct series_t
{
    FlexiAbstractSeries* series;
    struct series_t * next;
};

class FlexiPlotBase
{
public:
    FlexiPlotBase(const char* id = "P0");

    void setID(const char* id);

    FlexiAbstractSeries* addSeries(FlexiAbstractSeries* series);
    uint8_t count(); //Returns number of series added to the list

    FlexiAbstractSeries* series(uint8_t index);
    FlexiAbstractSeries* seriesByName(const char * name);

    void removeAllSeries();
    void removeSeries(uint8_t index);
    void removeSeriesByName(const char * name);
    
    void printID();
    void printSeries(); //Prints the names of the series
    //void printColor();

    virtual void plot() = 0;

protected:
    char id[ID_STR_SIZE];
    series_t* seriesList;

private:
    void addSeriesToLinkedList(series_t * el);
    
private:
    uint8_t seriesCount;
};

#endif //FLEXIPLOTBASE_H
