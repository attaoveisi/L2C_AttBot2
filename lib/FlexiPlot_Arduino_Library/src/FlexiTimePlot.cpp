#include "FlexiTimePlot.h"

FlexiTimePlot::FlexiTimePlot(const char* id)
  : FlexiXYPlot(id)
{
  
}


void FlexiTimePlot::plot()
{
  Serial.print('{');
  Serial.print(id);
  Serial.print('|');

  bool firstSeriesElement = true;
  series_t* s;
  s = seriesList;
  while (s != NULL)
  {
    
      FlexiXYSeries * series = static_cast<FlexiXYSeries*>( s->series );
      if(series->count() > 0)
      {
        if(firstSeriesElement == true)
          firstSeriesElement = false;
        else
          Serial.print('|');

        Serial.print(series->getName());
        Serial.print('|');
        
        if(series->hasColor())
        {
          Serial.print( series->getColor() );
          Serial.print( '|' );
        }

        
  
        bool firstDataElement = true;
        flexi_xy_data_t* d;
        d = series->data();
        while(d != NULL)
        {
          if(firstDataElement == true)
            firstDataElement = false;
          else
            Serial.print(' ');
  
          Serial.print(d->y);
  
          d = d->next;
        }

        series->clear();
        
      }

      s = s->next;
  }
  Serial.println( '}' );
}

