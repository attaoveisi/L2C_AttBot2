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
