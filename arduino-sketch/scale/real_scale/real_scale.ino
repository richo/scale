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
  // lscale.set_scale(-776.471);
  rscale.begin(RLOADCELL_DOUT_PIN, RLOADCELL_SCK_PIN);
  // lscale.set_scale(671.894);

  Serial.println("Tare... remove any weights from the scale.");
  delay(2000);
  lscale.tare();
  rscale.tare();
  Serial.println("Tare done...");
  Serial.print("Place weight");
  delay(2000);
}

void loop() {

  if (lscale.is_ready() && rscale.is_ready()) {

  Serial.println("get units: \t\t");

  Serial.println(lscale.get_units(1), 1);
  Serial.println(rscale.get_units(1), 1);
  } 
  else {
    // Serial.println("HX711 not found.");
    delay(100);
  }
  
}

//calibration factor will be the (reading)/(known weight)