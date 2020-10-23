#include "FlexiXYPlot.h"

FlexiXYPlot::FlexiXYPlot(const char* id)
  : FlexiPlotBase(id)
{
  
}


FlexiXYSeries* FlexiXYPlot::addSeries(const char * name)
{
  FlexiXYSeries* newSeries = new FlexiXYSeries(name);
  FlexiPlotBase::addSeries( newSeries );

  return newSeries;
}

FlexiXYSeries* FlexiXYPlot::series(uint8_t index)
{
  
  FlexiAbstractSeries* abstractSeries = FlexiPlotBase::series(index);
  if(abstractSeries == NULL)
    return NULL;

  FlexiXYSeries* series = static_cast<FlexiXYSeries*>(abstractSeries);
  
  return series;
}

FlexiXYSeries* FlexiXYPlot::seriesByName(const char * name)
{
  FlexiAbstractSeries* abstractSeries = FlexiPlotBase::seriesByName(name);
  if(abstractSeries == NULL)
    return NULL;

  FlexiXYSeries* series = static_cast<FlexiXYSeries*>(abstractSeries);

  return series;
}

void FlexiXYPlot::plot()
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

          Serial.print(d->x);
          Serial.print(" ");
          Serial.print(d->y);
  
          d = d->next;
        }

        series->clear();
        
      }

      s = s->next;
  }
  Serial.println( '}' );
}

