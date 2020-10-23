#include "FlexiPieChart.h"

FlexiPieChart::FlexiPieChart(const char* id)
  : FlexiPlotBase(id)
{
  
}


FlexiPieSeries* FlexiPieChart::addSeries(const char * name)
{
  FlexiPieSeries* newSeries = new FlexiPieSeries(name);
  FlexiPlotBase::addSeries( newSeries );

  return newSeries;
}

FlexiPieSeries* FlexiPieChart::series(uint8_t index)
{
  FlexiAbstractSeries* abstractSeries = FlexiPlotBase::series(index);
  if(abstractSeries == NULL)
    return NULL;

  FlexiPieSeries* series = static_cast<FlexiPieSeries*>(abstractSeries);
  
  return series;
}

FlexiPieSeries* FlexiPieChart::seriesByName(const char * name)
{
  FlexiAbstractSeries* abstractSeries = FlexiPlotBase::seriesByName(name);
  if(abstractSeries == NULL)
    return NULL;

  FlexiPieSeries* series = static_cast<FlexiPieSeries*>(abstractSeries);

  return series;
}

void FlexiPieChart::plot()
{
  Serial.print('{');
  Serial.print(id);
  Serial.print('|');

  bool firstSeriesElement = true;
  series_t* s;
  s = seriesList;
  while (s != NULL)
  {
     
      FlexiPieSeries * series = static_cast<FlexiPieSeries*>( s->series );
      
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

      Serial.print(series->value());
        
        

      s = s->next;
  }
  Serial.println( '}' );
}

