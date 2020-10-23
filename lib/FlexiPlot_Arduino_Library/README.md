# FlexiPlot Arduino Library
Arduino Library for interfacing with FlexiPlot plotting tool

Provides a relativly easier way to interface arduino with FlexiPlot

## Time Plot example:
Example how to plot a real time line chart with arduino and flexiplot.

It consists of 4 steps:
1. Add a plot:            *FlexiTimePlot myPlot;*
2. Add a series:          *myPlot.addSeries("Random");*
3. Add data to series:    *myPlot.seriesByName("Random");*
4. Send it to FlexiPlot:  *myPlot.plot();*

Then repeat steps 3 and 4. 
Once you send the data to FlexiPlot, the data points in arduino are automatically cleared.

Here is full example of real time plotting:
```
#include "FlexiPlot.h"

FlexiTimePlot myPlot;
FlexiXYSeries* seriesSin;

double xVal;

void setup() {
  Serial.begin(115200);

  myPlot.setID("P0");
  seriesSin = myPlot.addSeries("Sin");
  myPlot.addSeries("Cos");
  myPlot.addSeries("Random");

  xVal = 0;
}

void loop() {
  xVal += 2;
  
  seriesSin->addData(sin(xVal/10.0))->addData(sin((xVal+1)/10.0));
  myPlot.series(1)->addData(cos(xVal/10.0))->addData(cos((xVal+1)/10.0));
  myPlot.seriesByName("Random")->addData( random(-30,30)/30.0 )->addData( random(-30,30)/30.0 );
  delay(60);

  myPlot.plot();

}

```
