#include "FlexiPlot.h"

FlexiPieChart piePlot;


void setup() {
  Serial.begin(115200);

  piePlot.setID("P0");
  
  piePlot.addSeries("CO2");
  piePlot.addSeries("NO");
  piePlot.addSeries("Acetone");

}

void loop() {

  piePlot.series(0)->setValue(random(0,100));
  piePlot.series(1)->setValue(random(0,100));
  piePlot.series(2)->setValue(random(0,100));
  
  piePlot.plot();
  delay(4000);


}

