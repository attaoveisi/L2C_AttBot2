#include "FlexiPlot.h"

FlexiBarPlot * barPlot;


void setup() {
  Serial.begin(115200);

  barPlot = new FlexiBarPlot("P0");

  //Columns are added in order, column 0 = Jan, column 1 = Feb, column 2 = Mar
  barPlot->addColumn("Jan")->addColumn("Feb")->addColumn("Mar");
  
  barPlot->addSeries("Col1");  
  barPlot->series(0)->setValue(0, 25.0)->setValue(1, 50)->setValue(2, 10.9);
  
  barPlot->addSeries("Col2");
  // Not recommended, to lookup column index by name. Convinient but considerably slower than specifying index manually
  // I'm just showing that it is possible
  barPlot->series(1)->setValue( barPlot->columnIndex("Jan"), 10 )
                    ->setValue( barPlot->columnIndex("Mar"), 65 )
                    ->setValue( barPlot->columnIndex("Feb"), 48 );
  
  barPlot->plot();
}

void loop() {

  barPlot->series(0)->setValue(0, random(0,1000)/10.0)->setValue(1, random(0,1000)/10.0)->setValue(2, random(0,1000)/10.0);
  barPlot->series(1)->setValue( barPlot->columnIndex("Jan"), random(0,1000)/10.0 )
                    ->setValue( barPlot->columnIndex("Mar"), random(0,1000)/10.0 )
                    ->setValue( barPlot->columnIndex("Feb"), random(0,1000)/10.0 );
  
  barPlot->plot();
  delay(4000);


}

