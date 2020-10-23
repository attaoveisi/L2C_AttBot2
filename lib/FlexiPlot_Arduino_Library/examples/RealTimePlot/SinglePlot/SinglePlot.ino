#include "FlexiPlot.h"

FlexiTimePlot myPlot;
FlexiXYSeries* seriesSin;

double xVal;

void setup() {
  Serial.begin(115200);

  myPlot.setID("P0");
  seriesSin = myPlot.addSeries("Sin");


  xVal = 0;

}

void loop() {

  xVal += 0.1;
  
  seriesSin->addData( sin(xVal) );

  myPlot.plot();

  
  delay(60);

}
