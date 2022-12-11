#include <Arduino.h>
#include "disk91_LoRaE5.h"
#include"LIS3DHTR.h"
#include "TFT_eSPI.h"
#include "arduinoFFT.h"

// Requires
// - Disk91_LoRaE5 library (https://github.com/disk91/Disk91_LoRaE5/releases/tag/1.1.0)
// - LIS3DHTR library (https://github.com/Seeed-Studio/Seeed_Arduino_LIS3DHTR/releases/tag/v1.2.4)
// - Arduino FFT by Enrique Condes v 1.5.x

Disk91_LoRaE5 lorae5(false); // true, false whatever

uint8_t deveui[] = { 0xDC, ... 0x84, 0xEE };
uint8_t appeui[] = { 0xCD, ... 0x25, 0xAD };
uint8_t appkey[] = { 0x2A, ... 0x34, 0x43 };

LIS3DHTR<TwoWire> lis;
TFT_eSPI tft;

// FFT setup
arduinoFFT FFT = arduinoFFT(); 

void setup() {

  Serial.begin(9600);
  uint32_t start = millis();
  while ( !Serial );  // Open the Serial Monitor to get started or wait for 1.5"
  Serial.println("Setup starts");

  // init the library, search the LORAE5 over the different WIO port available
  if ( ! lorae5.begin(DSKLORAE5_SEARCH_WIO) ) {
    Serial.println("LoRa E5 Init Failed");
    while(1); 
  }

  // Setup the LoRaWan Credentials
  if ( ! lorae5.setup(
          DSKLORAE5_ZONE_EU868,     // LoRaWan Radio Zone EU868 here
          deveui,
          appeui,
          appkey
       ) ){
    Serial.println("LoRa E5 Setup Failed");
    while(1);         
  }
  Serial.println("LoRaWan setup done");

  // setup screen
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK); //Red background

  // setup Accelerometer
  lis.begin(Wire1); 
  lis.setOutputDataRate(LIS3DHTR_DATARATE_100HZ); // Setting output data rage to 25Hz, can be set up tp 5kHz 
  lis.setFullScaleRange(LIS3DHTR_RANGE_2G); // Setting scale range to 2g, select from 2,4,8,16g

  Serial.println("Setup done");

}
#define AXL_SIZE 256
void loop() {
  static uint8_t data[] = { 0x00 }; 
  static bool alarmDetected = false;
  static long lastKeepAlive = millis();

  // get accelerometer samples 128 values
  Serial.println("Sampling");
  float x_values[AXL_SIZE], y_values[AXL_SIZE], z_values[AXL_SIZE];
  for (int i=0 ; i < AXL_SIZE ; i++ ) {
    while ( ! lis.available() );
    lis.getAcceleration(&x_values[i], &y_values[i], &z_values[i]); 
    z_values[i] = 0.0; // because this lib is stupid and does not implement low pass filter
  }
  Serial.println("Sampling done");

  // compute max force
  float f = 0.0;
  double f_values[AXL_SIZE];
  double i_values[AXL_SIZE];
  for (int i=0 ; i < AXL_SIZE ; i++ ) {
    f_values[i] = sqrt((double)x_values[i]*x_values[i] + (double)y_values[i]*y_values[i] + (double)z_values[i]*z_values[i]);
    i_values[i] = 0.0;
    if (f_values[i] > f) f = f_values[i];
  }

  Serial.print("Force max:");Serial.println(f);
  // draw something on screen ... 320 px max, 2G Value is 4.0 
  tft.fillRect(0, 10, 320, 10, TFT_BLACK);
  tft.fillRect(0, 10, (int)(f*80.0), 10, TFT_RED);
  
  // get FFT
  FFT.Compute(f_values, i_values, AXL_SIZE, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(f_values, i_values, AXL_SIZE); 
  double x = FFT.MajorPeak(f_values, AXL_SIZE, 100 /*samplingFrequency*/);
  Serial.println(x, 6);
  
  // Send a message on alarm or on every 30 seconds
  if ( alarmDetected || (millis() - lastKeepAlive) > 30000 ) {
      // Send an uplink message. The Join is automatically performed
      data[0] = (alarmDetected)?1:0;
      if ( lorae5.send_sync(
            1,              // LoRaWan Port
            data,           // data array
            sizeof(data),   // size of the data
            false,          // we are not expecting a ack
            9,              // Spread Factor
            14              // Tx Power in dBm
           ) 
      ) {
          Serial.println("Uplink done");
          if ( lorae5.isDownlinkReceived() ) {
            Serial.println("A downlink has been received");
            if ( lorae5.isDownlinkPending() ) {
              Serial.println("More downlink are pending");
            }
          }
      }
      lastKeepAlive = millis();
      alarmDetected = false;
  }
    
}
