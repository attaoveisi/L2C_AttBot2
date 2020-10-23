#include "FlexiPlot.h"

FlexiXYPlot* scatterPlot;

#define K_CONST     4

void setup() {
  Serial.begin(115200);

  scatterPlot = new FlexiXYPlot("P0");
  scatterPlot->addSeries("Test");

}

void loop() {

  scatterPlotExample(K_CONST);

  delay(1000);


}


void scatterPlotExample(uint8_t k)
{
    double xD;
    for(int xI = -1000; xI < 1000; xI++)
    {
      xD = xI / 10.0;
      double r = cos(k*xD);
      double x = r * cos(xD);
      double y = r * sin(xD);
      scatterPlot->series(0)->addData( x, y );
      if(xI % 10 == 0)
      {
        scatterPlot->plot();
        delay(30);
      }
    }
    
    scatterPlot->plot();
}
