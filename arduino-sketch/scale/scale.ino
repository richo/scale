/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-load-cell-hx711/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

// Calibrating the load cell
#include <Arduino.h>
#include "soc/rtc.h"
#include "HX711.h"

// HX711 circuit wiring
const int LLOADCELL_DOUT_PIN = 16;
const int LLOADCELL_SCK_PIN = 4;

const int RLOADCELL_DOUT_PIN = 18;
const int RLOADCELL_SCK_PIN = 5;

HX711 lscale;
HX711 rscale;

void setup() {
  Serial.begin(115200);
  // rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);
  lscale.begin(LLOADCELL_DOUT_PIN, LLOADCELL_SCK_PIN);
  rscale.begin(RLOADCELL_DOUT_PIN, RLOADCELL_SCK_PIN);
}

void loop() {

  if (lscale.is_ready() && rscale.is_ready()) {
    lscale.set_scale();
    rscale.set_scale();
    Serial.println("Tare... remove any weights from the scale.");
    delay(5000);
    lscale.tare();
    rscale.tare();
    Serial.println("Tare done...");
    Serial.print("Place a known weight on the scale...");
    delay(5000);
    long lreading = lscale.get_units(10);
    long rreading = rscale.get_units(10);
    Serial.print("Result left: ");
    Serial.println(lreading);
    Serial.print("Result righ: ");
    Serial.println(rreading);
    
  } 
  else {
    Serial.println("HX711 not found.");
  }
  delay(1000);
}

//calibration factor will be the (reading)/(known weight)