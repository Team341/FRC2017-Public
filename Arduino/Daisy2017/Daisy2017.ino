// DaisyLights
// I2C Slave
// By AJN341

// This program controls the led light strips for Daisy2017. It 
// communicates with the roborio over I2C.

// Include the required Wire library for I2C
#include <Wire.h>
#include <PololuLedStrip.h>

// Create an ledStrip object and specify the pin it will use.
PololuLedStrip<11> turretStrip;
PololuLedStrip<12> gearStrip;

// Create a buffer for holding the colors (3 bytes per color).
#define LED_GEAR_COUNT 47
#define LED_TURRET_COUNT 29
rgb_color gearColors[LED_GEAR_COUNT];
rgb_color turretColors[LED_TURRET_COUNT];

int LED = LED_BUILTIN;
int gearPin = 3;
int shooterPin = 4;
int lightShowPin = 5;
int gearDelivered = 0;
int shooterOnTarget = 0;
int runLightShow = 0;
int8_t gearHalfWidthInLEDs = 4;

void setup() {
  Serial.begin(19200);
  
  // Define the LED pin as Output
  pinMode (LED, OUTPUT);

  pinMode(gearPin, INPUT);
  pinMode(shooterPin, INPUT);
  pinMode(lightShowPin, INPUT);

  turnTurretOff();
  turnGearOff();
  Serial.println("Starting");
}

void loop() {
  
  gearDelivered = digitalRead(gearPin);
  shooterOnTarget = digitalRead(shooterPin);
  runLightShow = digitalRead(lightShowPin);

/*
  if (runLightShow == HIGH){
    // Run the light show instead of denoting gear & target positions
    Serial.println("running lightshow");    
    lightShow();
    
  } else {
  */
    // Check Gear Status
    if (gearDelivered == HIGH) {
      setGearGreen();
      Serial.print("gear delivered, ");
    } else {
      setGearColors(0);
      Serial.print("gear not on, ");
    }

    // Check Shooter Status
    if (shooterOnTarget == HIGH) {
      if (gearDelivered == HIGH) {
        turnTurretOn();
      } else {
        turnTurretOnRED();
        Serial.println("Shooter On Target");
      }
    } else {
      turnTurretOff();
      Serial.println("shooter not on");
    }
    
  //}
  /*
  for(int x = -127; x <= 127; x++){
    setGearColors(x);
    delay(10);
  }

  for(int x = 127; x >= 0; x--){
    setGearColors(x);
    delay(10);
  }

  setGearGreen();
  delay(1000);
  */

  delay(10);
}

void turnTurretOff(){
    // Update the colors buffer.
    rgb_color color = {0,0,0};
    for(uint16_t i = 0; i < LED_TURRET_COUNT; i++)
    {
      turretColors[i] = color;
    }
    turretStrip.write(turretColors, LED_TURRET_COUNT);
}

void turnTurretOn(){
    // Update the colors buffer.
    rgb_color color = {0,0,127};
    for(uint16_t i = 0; i < LED_TURRET_COUNT; i++)
    {
      turretColors[i] = color;
    }
    turretStrip.write(turretColors, LED_TURRET_COUNT);
}

void turnTurretOnRED(){
    // Update the colors buffer.
    rgb_color color = {127,0,0};
    for(uint16_t i = 0; i < LED_TURRET_COUNT; i++)
    {
      turretColors[i] = color;
    }
    turretStrip.write(turretColors, LED_TURRET_COUNT);
}

void turnGearOff(){
    // Update the colors buffer.
    rgb_color color = {0,0,0};
    for(uint16_t i = 0; i < LED_GEAR_COUNT; i++)
    {
      gearColors[i] = color;
    }
    gearStrip.write(gearColors, LED_GEAR_COUNT);
}

void lightShow(){
    // Update the colors buffer.
    uint16_t time = millis() >> 2;
    for(uint16_t i = 0; i < LED_GEAR_COUNT; i++)
    {
      byte x = (time >> 2) - (i << 3);
      gearColors[i] = hsvToRgb((uint32_t)x * 359 / 256, 255, 255);
    }

    for(uint16_t i = 0; i < LED_TURRET_COUNT; i++)
    {
      byte x = (time >> 2) - (i << 3);
      turretColors[i] = hsvToRgb((uint32_t)x * 359 / 256, 255, 255);
    }
    
    turretStrip.write(turretColors, LED_TURRET_COUNT);
    gearStrip.write(gearColors, LED_GEAR_COUNT);
}

void setGearGreen(){
    // Update the colors buffer.
    rgb_color color = {0,255,0};
    for(uint16_t i = 0; i < LED_GEAR_COUNT; i++)
    {
      gearColors[i] = color;
    }
    gearStrip.write(gearColors, LED_GEAR_COUNT);
}

void setGearColors(int8_t pos){
    rgb_color gearColor = {255,255,0};
    rgb_color openColor = {0, 0, 255};
    int newPos = 25;//map(pos, -127, 127, 0, 47);
    for(uint16_t i = 0; i < LED_GEAR_COUNT; i++)
    {
      if (i < 19 || i > 27){
        gearColors[i] = openColor;
      }else{
        gearColors[i] = gearColor; 
      }
    }
    gearStrip.write(gearColors, LED_GEAR_COUNT);
}

// Converts a color from HSV to RGB.
// h is hue, as a number between 0 and 360.
// s is the saturation, as a number between 0 and 255.
// v is the value, as a number between 0 and 255.
rgb_color hsvToRgb(uint16_t h, uint8_t s, uint8_t v)
{
    uint8_t f = (h % 60) * 255 / 60;
    uint8_t p = (255 - s) * (uint16_t)v / 255;
    uint8_t q = (255 - f * (uint16_t)s / 255) * (uint16_t)v / 255;
    uint8_t t = (255 - (255 - f) * (uint16_t)s / 255) * (uint16_t)v / 255;
    uint8_t r = 0, g = 0, b = 0;
    switch((h / 60) % 6){
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        case 5: r = v; g = p; b = q; break;
    }
    return (rgb_color){r, g, b};
}

