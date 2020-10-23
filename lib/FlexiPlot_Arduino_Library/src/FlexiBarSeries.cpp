#include "FlexiBarSeries.h"

FlexiBarSeries::FlexiBarSeries(const char * series_name)
  : FlexiAbstractSeries(series_name)
{  
  this->dataCount = 0;
  m_data = NULL;
}

uint8_t FlexiBarSeries::count()
{
  return this->dataCount;
}

FlexiBarSeries* FlexiBarSeries::setValue(uint8_t col_index, long value)
{
  return setValue(col_index, (double)value);
}

FlexiBarSeries* FlexiBarSeries::setValue(uint8_t col_index, int value)
{
  return setValue(col_index, (double)value);
}

FlexiBarSeries* FlexiBarSeries::setValue(uint8_t col_index, float value)
{
  return setValue(col_index, (double)value);
}

FlexiBarSeries* FlexiBarSeries::setValue(uint8_t col_index, double value)
{
  uint8_t cId = 0;
  flexi_bar_data_t* p;
  p = m_data;
  while (p != NULL)
  {      
      if(cId == col_index)
      {
        p->value = value;
        break;
      }
      cId++;
      p = p->next;
  }
  
  return this;
}

void FlexiBarSeries::addColumn()
{  
  flexi_bar_data_t* lElement = new flexi_bar_data_t;/*malloc(sizeof *lElement);*/
  
  lElement->value = 0.0;
  lElement->next = NULL;
  addDataToLinkedList(lElement);

}

void FlexiBarSeries::removeColumn(uint8_t index)
{
  if(index > dataCount-1 || dataCount == 0)
    return;
    
  uint8_t cId = 0;
  flexi_bar_data_t* p;
  p = m_data;

  flexi_bar_data_t *parent = NULL;
  while (p != NULL)
  {      
      if(cId == index)
      {
        dataCount--;
        if(dataCount == 0)
        {
          m_data = NULL;
        }
        else if(parent == NULL)
        {
          m_data = p->next;
        }
        else
        {
          parent->next = p->next;
        }

        delete p;
        return;
      }

      cId++;
      parent = p;
      p = p->next;
  }
}

void FlexiBarSeries::addDataToLinkedList( flexi_bar_data_t * el )
{
  if(m_data == NULL)
  {
    m_data = el;
  }
  else
  {
    flexi_bar_data_t* p;
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

void FlexiBarSeries::clear()
{
  if(m_data == NULL)
    return;
    
  dataCount = 0;
  flexi_bar_data_t* p;
  p = m_data;
  while (p != NULL)
  {
      flexi_bar_data_t* currentP = p;
      p = p->next;

      delete currentP;
  }

  m_data = NULL;
}

