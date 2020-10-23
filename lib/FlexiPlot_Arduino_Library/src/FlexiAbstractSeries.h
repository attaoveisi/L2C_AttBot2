#ifndef FLEXIABSTRACTSERIES_H
#define FLEXIABSTRACTSERIES_H

#include <Arduino.h>

#define COLOR_STR_SIZE                    11 //255,255,255 this is the largest color string you can have
#define SERIES_NAME_STR_SIZE              32

class FlexiAbstractSeries
{
public:
    FlexiAbstractSeries(const char * series_name);
    virtual ~FlexiAbstractSeries();

    char * getName();
  
    void setColor(uint8_t red, uint8_t green, uint8_t blue);
    void setColor(const char * htmlColor);
    char* getColor();
    bool hasColor();

    virtual void clear() = 0;

protected:
    char series_name[SERIES_NAME_STR_SIZE];
    char color[COLOR_STR_SIZE]; 

   
};

#endif
