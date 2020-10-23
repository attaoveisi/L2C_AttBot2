#include "FlexiPieSeries.h"

FlexiPieSeries::FlexiPieSeries(const char * series_name)
  : FlexiAbstractSeries(series_name)
{    
  m_value = 0.0;
}


void FlexiPieSeries::setValue(long value)
{
  m_value = (double)value;
}

void FlexiPieSeries::setValue(int value)
{
  m_value = (double)value;
}

void FlexiPieSeries::setValue( float value)
{
  m_value = (double)value;
}

void FlexiPieSeries::setValue(double value)
{
  m_value = (double)value;
}


void FlexiPieSeries::clear()
{
  m_value = 0.0;
}

