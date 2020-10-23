#include "FlexiPlotBase.h"

FlexiPlotBase::FlexiPlotBase(const char * id)
{
  strcpy(this->id, id);
  seriesList = NULL;
  seriesCount = 0;
}

void FlexiPlotBase::setID(const char * id)
{
  memset(this->id, 0, ID_STR_SIZE);
  strcpy(this->id, id);
}


FlexiAbstractSeries* FlexiPlotBase::addSeries(FlexiAbstractSeries* series)
{
  series_t* lElement = new series_t;/* malloc(sizeof *lElement);*/
  
  lElement->series =  series;
  lElement->next = NULL;
  addSeriesToLinkedList(lElement);

  return series;
}

void FlexiPlotBase::addSeriesToLinkedList( series_t * el )
{
  if(seriesList == NULL)
  {
    seriesList = el;
  }
  else
  {
    series_t* p;
    p = seriesList;
    while (p != NULL)
    {
        if(p->next == NULL)
        {
          p->next = el;
          break;
        }
        
        p = p->next;
    }
  }

  seriesCount++;
  
}

void FlexiPlotBase::printSeries()
{
    uint8_t sId = 0;
    series_t* p;
    p = seriesList;
    while (p != NULL)
    {
        Serial.print(sId++);
        Serial.print(": ");
        Serial.println( p->series->getName() );
        
        p = p->next;
    }
}

FlexiAbstractSeries* FlexiPlotBase::series(uint8_t index)
{
  if(index > seriesCount-1)
    return NULL;
    
  uint8_t sId = 0;
  series_t* p;
  p = seriesList;
  while (p != NULL)
  {      
      if(sId == index)
        return p->series;

      sId++;
      p = p->next;
  }
  
  return NULL;
}

FlexiAbstractSeries* FlexiPlotBase::seriesByName(const char * name)
{
  series_t* p;
  p = seriesList;
  while (p != NULL)
  {      
      if( strcmp(p->series->getName(), name) == 0)
      {
        return p->series;
      }

      p = p->next;
  }

  return NULL;
}

void FlexiPlotBase::removeAllSeries()
{
  if(seriesCount == 0)
    return;
    
  series_t* s;
  s = seriesList;
  while (s != NULL)
  {
    s->series->clear();
    delete s->series;

    series_t* prevSeries = s;
    s = s->next;

    delete prevSeries;
  }

  seriesCount = 0;
}

void FlexiPlotBase::removeSeries(uint8_t index)
{
  if(index > seriesCount-1 || seriesCount == 0)
    return;
    
  uint8_t sId = 0;
  series_t* p;
  p = seriesList;

  series_t *parent = NULL;
  while (p != NULL)
  {      
      if(sId == index)
      {
        p->series->clear();
        delete p->series;
        seriesCount--;
        if(seriesCount == 0)
        {
          seriesList = NULL;
        }
        else if(parent == NULL)
        {
          seriesList = p->next;
        }
        else
        {
          parent->next = p->next;
        }

        delete p;
        return;
      }

      sId++;
      parent = p;
      p = p->next;
  }
}

void FlexiPlotBase::removeSeriesByName(const char * name)
{
  series_t* p;
  p = seriesList;

  series_t *parent = NULL;
  while (p != NULL)
  {      
      if( strcmp(p->series->getName(), name) == 0)
      {
        p->series->clear();
        delete p->series;
        seriesCount--;
        if(seriesCount == 0)
        {
          seriesList = NULL;
        }
        else if(parent == NULL)
        {
          seriesList = p->next;
        }
        else
        {
          parent->next = p->next;
        }

        delete p;
        return;
      }
      parent = p;
      p = p->next;
  }
}

uint8_t FlexiPlotBase::count()
{
  return seriesCount;
}


void FlexiPlotBase::printID()
{
  Serial.println(id);
}

//void FlexiPlotBase::printColor()
//{
//  Serial.println(color);
//}

