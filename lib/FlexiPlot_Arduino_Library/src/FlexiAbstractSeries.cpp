#include "FlexiAbstractSeries.h"

FlexiAbstractSeries::FlexiAbstractSeries(const char * series_name)
{
  memset(this->color, 0, COLOR_STR_SIZE);
  strcpy(this->series_name, series_name);
}

char * FlexiAbstractSeries::getName()
{
  return series_name;
}

char * FlexiAbstractSeries::getColor()
{
  return color;
}

void FlexiAbstractSeries::setColor(uint8_t red, uint8_t green, uint8_t blue)
{
  char numBuffer[3];
  memset(this->color, 0, COLOR_STR_SIZE);
  
  itoa(red, numBuffer, 10);
  strcpy(this->color, numBuffer);
  strcat(this->color, ",");
  
  itoa(green, numBuffer, 10);
  strcat(this->color, numBuffer);
  strcat(this->color, ",");
  
  itoa(blue, numBuffer, 10);
  strcat(this->color, numBuffer);
}

void FlexiAbstractSeries::setColor(const char * htmlColor)
{
  if(strlen(htmlColor) != 7)
  {
    Serial.println(F("Invalid Color"));
    return;
  }

  char numBuffer[2];

  numBuffer[0] = htmlColor[1]; numBuffer[1] = htmlColor[2];
  uint8_t red = strtol(numBuffer, NULL, 16);

  numBuffer[0] = htmlColor[3]; numBuffer[1] = htmlColor[4];
  uint8_t green = strtol(numBuffer, NULL, 16);

  numBuffer[0] = htmlColor[5]; numBuffer[1] = htmlColor[6];
  uint8_t blue = strtol(numBuffer, NULL, 16);

  this->setColor(red, green, blue);
}

bool FlexiAbstractSeries::hasColor()
{
  return (color[0] != '\0');
}

FlexiAbstractSeries::~FlexiAbstractSeries()
{
  free( color );
  free( series_name );
}

