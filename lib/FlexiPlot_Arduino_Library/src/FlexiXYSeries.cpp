#include "FlexiXYSeries.h"

FlexiXYSeries::FlexiXYSeries(const char * series_name)
  : FlexiAbstractSeries(series_name)
{  
  this->dataCount = 0;
  m_data = NULL;
}

uint8_t FlexiXYSeries::count()
{
  return this->dataCount;
}

FlexiXYSeries* FlexiXYSeries::addData(int y)
{
  return addData((double)0.0, (double)y);
}
FlexiXYSeries* FlexiXYSeries::addData(long y)
{
  return addData((double)0.0, (double)y);
}
FlexiXYSeries* FlexiXYSeries::addData(double y)
{
  return addData((double)0.0, (double)y);
}
FlexiXYSeries* FlexiXYSeries::addData(float y)
{
  return addData((double)0.0, (double)y);
}
FlexiXYSeries* FlexiXYSeries::addData(int x, int y)
{
  return addData((double)x, (double)y);
}
FlexiXYSeries* FlexiXYSeries::addData(long x, long y)
{
  return addData((double)x, (double)y);
}
FlexiXYSeries* FlexiXYSeries::addData(float x, float y)
{
  return addData((double)x, (double)y);
}

FlexiXYSeries* FlexiXYSeries::addData(double x, double y)
{  
  flexi_xy_data_t* lElement = new flexi_xy_data_t;/*malloc(sizeof *lElement);*/
  
  lElement->x =  x;
  lElement->y =  y;
  lElement->next = NULL;
  addDataToLinkedList(lElement);

  return this;
}

void FlexiXYSeries::addDataToLinkedList( flexi_xy_data_t * el )
{
  if(m_data == NULL)
  {
    m_data = el;
  }
  else
  {
    flexi_xy_data_t* p;
    p = m_data;
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

  dataCount++;
  
}

void FlexiXYSeries::clear()
{
  if(m_data == NULL)
    return;
    
  dataCount = 0;
  flexi_xy_data_t* p;
  p = m_data;
  while (p != NULL)
  {
      flexi_xy_data_t* currentP = p;
      p = p->next;

      delete currentP;
  }

  m_data = NULL;
}

