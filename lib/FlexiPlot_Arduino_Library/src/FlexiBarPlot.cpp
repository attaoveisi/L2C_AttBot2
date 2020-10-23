#include "FlexiBarPlot.h"

FlexiBarPlot::FlexiBarPlot(const char* id)
  : FlexiPlotBase(id)
{
  m_columns = NULL;
  columnCount = 0;
}

FlexiBarPlot* FlexiBarPlot::addColumn(const char * col_name)
{  
  flexi_bar_column_t* lElement = new flexi_bar_column_t;/* malloc(sizeof *lElement);*/

  FlexiBarColumn * newColumn = new FlexiBarColumn( col_name );

  lElement->column = newColumn;
  lElement->next = NULL;
  addColumnToLinkedList(lElement);

  addColumnToAllSeries();

  return this;
}

void FlexiBarPlot::addColumnToLinkedList( flexi_bar_column_t * el )
{
  if(m_columns == NULL)
  {
    m_columns = el;
  }
  else
  {
    flexi_bar_column_t* p;
    p = m_columns;
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

  columnCount++;
  
}

void FlexiBarPlot::addColumnToAllSeries()
{
    series_t* p;
    p = seriesList;
    while (p != NULL)
    {        
        FlexiBarSeries* series = static_cast<FlexiBarSeries*>(p->series);
        series->addColumn();
        p = p->next;
    }
}

void FlexiBarPlot::removeColumn(uint8_t index)
{
  if(index > columnCount-1 || columnCount == 0)
    return;

  removeColumnFromAllSeries(index);
  
  uint8_t cId = 0;
  flexi_bar_column_t* p;
  p = m_columns;

  flexi_bar_column_t *parent = NULL;
  while (p != NULL)
  {      
      if(cId == index)
      {
        delete p->column;
        columnCount--;
        if(columnCount == 0)
        {
          m_columns = NULL;
        }
        else if(parent == NULL)
        {
          m_columns = p->next;
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

void FlexiBarPlot::removeColumnFromAllSeries(uint8_t index)
{
    series_t* p;
    p = seriesList;
    while (p != NULL)
    {        
        FlexiBarSeries* series = static_cast<FlexiBarSeries*>(p->series);
        series->removeColumn(index);
        p = p->next;
    }
}

void FlexiBarPlot::printColumns()
{
  if(m_columns == NULL)
    return;

  flexi_bar_column_t* p;
  p = m_columns;
  bool appendSemicolon = false;
  while (p != NULL)
  {
      if(appendSemicolon == false)
        appendSemicolon = true;
      else
        Serial.print(';');
    
      Serial.print(p->column->getName());
      
      p = p->next;
  }
}

uint8_t FlexiBarPlot::columns()
{
  return this->columnCount;
}

int8_t FlexiBarPlot::columnIndex(const char * col_name)
{
  int8_t cId = 0;
  flexi_bar_column_t* p;
  p = m_columns;

  while (p != NULL)
  {      
      if( strcmp(p->column->getName(), col_name) == 0)
      {
        return cId;
      }
      cId++;
      p = p->next;
  }

  return -1;
}


FlexiBarSeries* FlexiBarPlot::addSeries(const char * name)
{
  FlexiBarSeries* newSeries = new FlexiBarSeries(name);
  FlexiPlotBase::addSeries( newSeries );

  for(uint8_t col = 0; col < columnCount; col++)
    newSeries->addColumn();

  return newSeries;
}

FlexiBarSeries* FlexiBarPlot::series(uint8_t index)
{
  
  FlexiAbstractSeries* abstractSeries = FlexiPlotBase::series(index);
  if(abstractSeries == NULL)
    return NULL;

  FlexiBarSeries* series = static_cast<FlexiBarSeries*>(abstractSeries);
  
  return series;
}

FlexiBarSeries* FlexiBarPlot::seriesByName(const char * name)
{
  FlexiAbstractSeries* abstractSeries = FlexiPlotBase::seriesByName(name);
  if(abstractSeries == NULL)
    return NULL;

  FlexiBarSeries* series = static_cast<FlexiBarSeries*>(abstractSeries);

  return series;
}

void FlexiBarPlot::plot()
{
  Serial.print('{');
  Serial.print(id);
  Serial.print('|');

  printColumns();
  Serial.print('|');

  bool firstSeriesElement = true;
  series_t* s;
  s = seriesList;
  while (s != NULL)
  {
     
      FlexiBarSeries * series = static_cast<FlexiBarSeries*>( s->series );
      
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
        flexi_bar_data_t* d;
        d = series->data();
        while(d != NULL)
        {
          if(firstDataElement == true)
            firstDataElement = false;
          else
            Serial.print(' ');

          Serial.print(d->value);
  
          d = d->next;
        }
        
      }

      s = s->next;
  }
  Serial.println( '}' );
}

