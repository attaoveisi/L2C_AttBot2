#ifndef FLEXIBARCOLUMN_H
#define FLEXIBARCOLUMN_H

#include <Arduino.h>

#define BAR_NAME_STR_SIZE     32

class FlexiBarColumn
{
public:
    FlexiBarColumn(const char * column_name);
    ~FlexiBarColumn();
    char * getName();
  
protected:
    char column_name[BAR_NAME_STR_SIZE];

   
};

#endif
