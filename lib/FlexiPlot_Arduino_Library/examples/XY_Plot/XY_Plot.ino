#include "FlexiPlot.h"

FlexiXYPlot* xyPlot;

uint8_t k = 0;

void setup() {
  Serial.begin(115200);

  xyPlot = new FlexiXYPlot("P0");
  xyPlot->addSeries("Test");

}

void loop() {

  xyPlotExample(k++%7);

  delay(1000);


}


void xyPlotExample(uint8_t k)
{
  
    double xD;
    for(int xI = -50; xI < 50; xI++)
    {
      xD = xI / 10.0;
      double r = cos(k*xD);
      double x = r * cos(xD);
      double y = r * sin(xD);
      xyPlot->series(0)->addData( x, y );
    }
    
    xyPlot->plot();
}
