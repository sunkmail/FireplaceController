/*   Intro
  Fireplace_Controller.ino

  Controls
    Rotary Encoder with PB Switch - Most controls
    Momentary PB Switch - FP Override (if Off, Turn on - If On, Turn off)
                            ************** Timer to revert back at some point?  (if on temp control mode)
    LCD or OLED display for showing current mode/status & setting change info.

  Controlable features:
      LED lighting for 4 shelves.  On/off Dim, each individually settable PWM value
      Gas fireplace on/off.    mV control system, activated by relay contacts


  Add-Ons, to do:
           Temp control - thermostat
           Remote operation (IR?)
           Remote sense of power on at non-local switch controlled outlet
           Fireplace Blower Motor Control



  The circuit:

    https://easyeda.com/sunkmail/Fireplace_and_Lighting_Control-adee12656a2c4f5d9f324d52759f96b2



    list the components attached to each input
    list the components attached to each output




  Created: 14 January 2018
  By Scott Mitten (Sunkmail)

  Modified: day month year
  By author's name

  https://create.arduino.cc/editor/Sunkmail/9b2c17a9-e4cb-4785-927b-2c0635a6529a

*/



/*   Libraries not included

// HardWire - Version: Latest 
#include <HardWire.h>

// EnableInterrupt - Version: Latest 
#include <EnableInterrupt.h>

*/

/*
// RadioHead - Version: Latest 
#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile
*/

/*
// RFReceiver - Version: Latest 
#include <RFReceiver.h>       // for 433 MHz Radio
// https://andreasrohner.at/posts/Electronics/New-Arduino-library-for-433-Mhz-AM-Radio-Modules/

// PinChangeInterruptHandler - Version: Latest 
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptHandler.h>
   // Based on below Library, below, to work with RF library, above

// PinChangeInterrupt - Version: Latest 
//#include <PinChangeInterrupt.h>
//#include <PinChangeInterruptBoards.h>
//#include <PinChangeInterruptPins.h>
//#include <PinChangeInterruptSettings.h>
*/

      

// U8g2 - Version: Latest   // I2C Displays
#include <U8g2lib.h>
//#include <U8x8lib.h>

// Wire - Version: Latest   // I2C Library
//#include <Wire.h>     

// DallasTemperature - Version: Latest 
#include <DallasTemperature.h>
#include <OneWire.h>

/*
// IRremote - Version: Latest 
#include <IRremote.h>
#include <IRremoteInt.h>
#include <boarddefs.h>

// EEPROMEx - Version: Latest 
#include <EEPROMVar.h>
#include <EEPROMex.h>
*/





//const bool isDebug = true;              // Debug messages?
const bool isDebug = false;

const bool isTriacDimmer = false;           // Is the motor speed control (Triac Dimmer) present
const bool isTriacDimmerSmart = false;      // Set FALSE for local triac dimmer Sense & firing -> Dumb Module  
//const bool isTriacDimmerSmart = true;      // Set TRUE for ATtiny85 Triac Dimmer -> 'Smart' Module

const bool isTempSensorPresent = false;    // Is the Temp sensor installed

const bool isRF = false;                  // Is the 433MHZ ASK RF module installed

const bool isOLED = true;                // Is the OLED display attached


const bool ButtonActive = LOW;    // Pushbuttons are using pull-ups.  Signal is Low when active

// **************************************************************
// ************** Rotary Encoder Assignments ********************


const byte EncodeSW_PIN = 4;     // Push button switch on Rotary Encoder
const byte EncodeA_PIN = 3;      // HW Int1 - CLK signal from Rotary Encoder (Pin A) - Used for generating interrupts      
const byte EncodeB_PIN = A0;      // DT (data) signal from Rotary Encoder (Pin B) - Used for reading direction

const byte EncoderBounce = 5;       // Encoder Debounce time (max) in milliseconds
const byte SwBounce = 50;           // Encoder Switch Debounce time (max) in milliseconds

volatile int virtualPosition = 50;   // Updated by the ISR (Interrupt Service Routine)





// **************************************************************
// ******************** LED Assignments *************************

const byte LED_PIN[] = {9, 5, 6, 10, 11};  // Array for Pin Assignments for PWM Shelf Lightning rows  - First spot Not currently used (LED_0) for easier numbering  - On PCB for future expansion
                                        // (LED1 is Top shelf, LED4 is bottom)       

const byte LED_PwrMax = 100;       // Max. number of steps for LED brightness, 1% per step if set to 100   - DO NOT SET HIGHER THAN 255

volatile byte LED_Pwr[] = {50, 50, 50, 50, 50};   // PERCENTAGE of FULL Bright - To be fed to PWM signal through Map function

                                      //   Eventually - Assign to EEPROM - To save value between power ups               
                                                // Keep here for testing



volatile bool LEDsUpdate = true;   // Flag for changes to LED PWM Settings - Set by encoder ISR
bool shelfLEDsOn = false;        // Flag for LEDs On/Off status

// **************************************************************
// ****************** Module Set-ups ****************************

// 0.91" OLED display module  128x32

/* Constructor */    // U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C(rotation, [reset [, clock, data]])
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C OLED(U8G2_R0);  // roation is only setting due to HW I2C and no reset pin

bool updateDisplay_FLAG = true;              // Set initially true to do first display
const byte Contrast = 0;


// Fireplace & Triac Module Control
const byte FP_On_PIN = 13;             // Pin Assignment for Fireplace On/Off
const byte FP_Sense_PIN = 2;           // HWint0 - Pin Assignment for Zero Cross Detect from dumb, or Rx from 'Smart', Triac Dimmer unit
const byte FP_Blower_PIN = A1;         // Pin Assignment for blower motor control

bool fireplaceOn_FLAG = false;             // Set initial value for if fireplace is currently On or not


// RF Rx Module (433 MHz ASK Module)
const byte RadioIn_PIN = A2; 
/*
RFReceiver RFRx(RadioIn_PIN);               // Set up RF Receiver library on RadioIn Pin
const byte RFNodeID = 0;                    // Set this unit to Node 0  - Tx unit must be the same
const char dataRFExpecting = "I'm Alive";   // Only valid message expected
const byte MaxRFDataSize = 10;
*/


// IR Rx module (VS1838B)
const byte IR_Data_PIN = 8;

// Temperature Sensor (DS18B20)   
const byte TempSense_PIN = 7;

/* Constructor */
/*
OneWire oneWire(TempSense_PIN);           // Setup a oneWire instance to communicate with any OneWire devices  
                                          // (not just Maxim/Dallas temperature ICs)
DallasTemperature tempSensor(&oneWire);   // Pass our oneWire reference to Dallas Temperature Library. 

//const byte TempSensorCount = 1;           // How many sensors are we expecting      // not needed if only using 1
byte tempSensorAddr[8];                   // Variable to store device's uniquie ID
const byte TempSensorResolution = 9;      // Integer value for sensor precision.  Can be 9, 10, 11 or 12 bits
/*
          Mode      Resol   Conversion time
          9 bits    0.5°C     93.75 ms
          10 bits   0.25°C    187.5 ms
          11 bits   0.125°C   375 ms
          12 bits   0.0625°C  750 ms
*/
/*
const unsigned int TempSensorConvTime = 100;  // Time in ms to wait between request for temp conv. and read of temperature - Based on info above.
float currentTempC = 0.0;                 // Current Room Temperature in C
float targetTempC = 21.0;                 // Target Room Temperature (for thermostat control)
byte tempHysteresis = 1;                  // To prevent frequent On/Off cycles near target temp

*/

// **************************************************************
// ****************** General Assignments ***********************

const byte ModeSwitch_PIN = 12;          // Pin Assignment for Mode Switch

const unsigned int MaxModeChangFreq = (1000 * 10);    // Min time between switching modes (1000 * x), where x = seconds between changes


unsigned long currentMillis;      // Working/scratchpad variable for checking times






// ----------------------------------------------------------------------------
// DEBUG      DEBUG      DEBUG      DEBUG      DEBUG      DEBUG      DEBUG
// One size fits all Serial Monitor debugging messages
// ----------------------------------------------------------------------------
//bool isDebug = true;              // Debug messages?          Moved to top of code - easier to find
//bool isDebug = false;

template<typename T> void debugPrint(T printMe, bool newLine = false) {
  if (isDebug) {
    if (newLine) {
      Serial.println(printMe);
    }
    else {
      Serial.print(printMe);
    }
    Serial.flush();
  }
}


// ------------------------------------------------------------------------------------------------------------------
// SETUP    SETUP    SETUP    SETUP    SETUP    SETUP    SETUP    SETUP    SETUP    SETUP    SETUP    SETUP    SETUP
// ------------------------------------------------------------------------------------------------------------------
void setup() {
  // Just for debug, view output on serial monitor -
  if(isDebug)
    Serial.begin(9600);       // Only open Serial comms if in Debug mode


  // **************************************************************
  // **************** Rotary Encoder Setup ************************  
  
  pinMode(EncodeSW_PIN, INPUT_PULLUP);      // Switch is floating - use the in-built PULLUP so we don't need a resistor
  
  pinMode(EncodeA_PIN, INPUT_PULLUP);       // Rotary pulses are INPUTs - Possibly floating if not using module board.  (Extra Pull up shouldn't hurt, even if using module board.)
  pinMode(EncodeB_PIN, INPUT_PULLUP);



  // **************************************************************
  // ********************** LEDs Setup ****************************
  for(int i = 0; i <= 4; i++){
    pinMode(LED_PIN[i], OUTPUT);      // explicitly Make all LED pins Output
    }


  // **************************************************************
  // ********************* Fireplace Setup ************************
  
  digitalWrite(FP_On_PIN, fireplaceOn_FLAG);     // Ensure FP isn't turned on at boot-up - fireplaceOn_FLAG initially set to false
  pinMode(FP_On_PIN, OUTPUT);           // explicitly Make pin Output to control fireplae relay

  digitalWrite(FP_Blower_PIN, 0);       // Not enabled yet, but make sure off anyway
  pinMode(FP_Blower_PIN, OUTPUT);       // for Triac trigger from dumb, or Tx to 'Smart', Triac Dimmer unit
  pinMode(FP_Sense_PIN, INPUT);         // for Zero Cross Detect from dumb, or Rx from 'Smart', Triac Dimmer unit
  
  pinMode(ModeSwitch_PIN, INPUT_PULLUP);   // Mode Switch for Fireplace on/off - Internal Pull-up for push button
  
  
  
  // **************************************************************
  // ********************** Misc Setup ****************************
  
  // 0.91" OLED display module  128x32
  OLED.begin();                             // Start the OLED display Object/Library
//  OLED.setFont(u8g2_font_logisoso32_tr);    // Set the (initial?) font for 32 Pixel high
//  OLED.setCursor(0,32);                     // Set cursor to prepare for first write
  OLED.setContrast(Contrast);         // Set OLED contrast (0 - 255) - Lower number is slightly darker
  OLED.setFontPosTop();               // Set Font to start from Top of character set, rather than bottom (the default)
  
  // RF Rx Module (433 MHz ASK Module)
  pinMode(RadioIn_PIN, INPUT); 
/*
  if(isRF){                                     // If RF Module installed:
    RFRx.begin();                               //Start the Library 
  }
*/  

  // IR Rx module (VS1838B)
  pinMode(IR_Data_PIN, INPUT_PULLUP); 

  // Temperature Sensor (DS18B20)
  pinMode(TempSense_PIN, INPUT);     
  
  if(isTempSensorPresent){                              // If Temp sensor present:
/*    
    tempSensor.begin();                                 // Start the Library
    tempSensor.getAddress(tempSensorAddr, 0);           // Get addrress for device at index 0  (only sensor, no loop needed)
                                                        // Addresses used as faster than doing by index.

    tempSensor.setResolution(tempSensorAddr, TempSensorResolution);     // Set resolution of temp sensor
    tempSensor.setWaitForConversion(false);             // Put into Async. Mode
    // tempSensor.setCheckForConversion(true);             // Program will look for flag that conversion complete    //NOT ACTUALLY SURE what this setting does.  Will just track millis
    tempSensor.requestTemperaturesByAddress(tempSensorAddr);    // Send request for current temp - To get initial value going
*/
  }
  
    // Attach the routines to service the interrupts
  attachInterrupt(digitalPinToInterrupt(EncodeA_PIN), isr_EncoderKnob, LOW);            //Move to sub-routine that changes LED values, updates settings,wakes up backlight, etc)
                                                                        // Here just for initial testing of encoder - Or adjust to turn on Backlight of display ??
/*
 if(isTriacDimmerSmart == false)        // Only attch interrupt if using dumb dimmer
  attachInterrupt(digitalPinToInterrupt(FP_Sense_PIN), isr_ZeroCrossDetect, HIGH);   // Handles when zero cross detected on dumb Triac dimmer module
 */ 
  delay(100);         // Let everything settle for 100 ms on boot-up
  
  // Ready to go!
  debugPrint("Setup finished successfully.",1);

}



// ------------------------------------------------------------------
// MAIN LOOP     MAIN LOOP     MAIN LOOP     MAIN LOOP     MAIN LOOP
// ------------------------------------------------------------------
void loop() {
  static bool lightsOnRF = false;         // RF module saying to turn light on?
  unsigned long lastValidRFMessage = 0;   // when was the last signal from the RF Module
  const unsigned int MaxTimeBetweenRFSignal = 600;  //Max allowable time between valid RF signals
  
  currentMillis = millis();     // Update current time for this loop
  
  checkModeSwitch();            // Poll the Mode Switch once per loop  - Turn On/Off 'fireplaceOn_FLAG' Flag as needed  
  digitalWrite(FP_On_PIN, fireplaceOn_FLAG);   // Turn Fireplace On/Off based on current setting

  checkEncoderButton();       // Poll Encoder Pushbutton    // Currently just changes shelf lights flag

  
  if (LEDsUpdate == true) {   // Check if any LED settings have been changed
    updateLEDs();               // If any changed, update and clear flag
    updateDisplay_FLAG = true;  
  }
  
  if(isTempSensorPresent){            // If the temp sensor present
    debugPrint("Temp check routine entered",1);
    checkRoomTemp();                  // get current temp
  }
  
  if(isRF){                            // If the RF module is connected
    debugPrint("RF Routine entered",1);
    lightsOnRF = checkRFRecieved();   // Has a valid signal been received
    
    if(shelfLEDsOn == true && lightsOnRF == false){                          // if the lights are on, but RF signal not received
      if((currentMillis - lastValidRFMessage) > MaxTimeBetweenRFSignal){    // AND if the max time between hearing from the remote RF signal has passed
        shelfLEDsOn = false;          // Turn off shelf lights (via flag)
        LEDsUpdate = true;            // Activate flag to do LED update on next loop
      }
    }
  }
  
  if(updateDisplay_FLAG == true){      // Display needs updating?
    updateDisplay();
  }

  
  
}   // End Main Loop


 


void doNothing();       //spacer for code readability - compiler should ignore

// ------------------------------------------------------------------
// updateDisplay     updateDisplay     updateDisplay     updateDisplay
// ------------------------------------------------------------------
void updateDisplay(){
  
  OLED.firstPage();
  do {
    /* all graphics commands have to appear within the loop body. */    
    OLED.setFont(u8g2_font_logisoso32_tr);    // Set the font for 32 Pixel high
    OLED.setCursor(0,0);                     // Set cursor to prepare for write  
    //u8g2.drawStr(0,20,"Hello World!");
//    OLED.print(currentTempC,1);                 // Print the current temp with 1 decimal point
//    OLED.print(" C");                           // add a C for Celsius
    OLED.print(virtualPosition);
  } while ( OLED.nextPage() );
  updateDisplay_FLAG = false;            // Reset Flag
 
  debugPrint("Update Display Routine finished",1);
  debugPrint("      ",1);
}


// ------------------------------------------------------------------
// CheckModeSwitch     CheckModeSwitch     CheckModeSwitch     CheckModeSwitch
// ------------------------------------------------------------------
void checkModeSwitch() {
  // static unsigned long LastModeChange = 0;                               // Debounce check not needed due to really long delay between allowable changes.
  static unsigned long lastModeChange = (currentMillis - MaxModeChangFreq - 1);  // Set lastModeChange to assure Toggle on first call to routine.
  
  if (digitalRead(ModeSwitch_PIN) == ButtonActive) {                     // If Mode Switch being pressed
   if ((currentMillis - lastModeChange) > MaxModeChangFreq) {      // & if outside of last time mode was switched limit
      // digitalWrite(FP_On_PIN, !digitalRead(FP_On_PIN));                 // Toggle Fireplace On / Off
      fireplaceOn_FLAG = !fireplaceOn_FLAG;                               // Toggle Fireplace On / Off
      lastModeChange = currentMillis;                           // Reset Timer reference Start for next possible change (MaxModeChangFreq)
    }                                                             
  }
  // Else, do nothing - Button isn't pressed, or been pressed too recently.
  
}


// ------------------------------------------------------------------
// checkRFRecieved     checkRFRecieved     checkRFRecieved     checkRFRecieved     checkRFRecieved
// ------------------------------------------------------------------
bool checkRFRecieved(){
/*  
  char RFData[MaxRFDataSize];                        // make array to hold received RF data
  byte senderId = 0;
  byte packageId = 0;
  
  if(RFRx.ready()){                                                         // If the Rx buffer has data
    byte len = RFRx.recvPackage((byte *)RFData, &senderId, &packageId);     // Get the data
    
    if(dataRFExpecting == RFData && senderId == RFNodeID){                //If correct message, from correct node
//        if(dataRFExpecting == RFData && &senderId == RFNodeID){
      return true;
    }
    
  }
*/
  return false;                          // if buffer not ready - no data received since last check, or wrong data, return false
}

// ------------------------------------------------------------------
// updateLEDs     updateLEDs     updateLEDs     updateLEDs     updateLEDs
// ------------------------------------------------------------------
void updateLEDs() {
  debugPrint("Update LEDs routine entered",1);
  if (shelfLEDsOn == true) {                                     // If LEDs should be on:
    debugPrint("LEDs should be On",1);
    for(int i = 1; i <= 4; i++){                            // Set testing 'virtual position' to all LED settings
      LED_Pwr[i] = virtualPosition;                            // remove once code for individual power settings written
    }
  
  
    for(int i = 1; i <= 4; i++){                            // Update PWM setting to latest values from Power setting
      analogWrite(LED_PIN[i],map(LED_Pwr[i],0,LED_PwrMax,0,255));      // Map LED_PWR Value (Percentage of full On) to actual PWM value
    }
    debugPrint("LED PWM written ");
    debugPrint(map(LED_Pwr[1],0,LED_PwrMax,0,255),1);
  }
  else {                                                    // If Shelf LEDs should be off, turn off
    debugPrint("LEDs off",1);
    for(int i = 1; i <= 4; i++){
      digitalWrite(LED_PIN[i],LOW);
    }
  }
  
  LEDsUpdate = false;                                   // Reset Flag

  debugPrint("updateLEDs function virtual Position: ");
  debugPrint(virtualPosition,1);
}


// ------------------------------------------------------------------
// checkEncoderButton     checkEncoderButton     checkEncoderButton     checkEncoderButton
// ------------------------------------------------------------------
void checkEncoderButton ()  {
  static unsigned long lastEncoderButtonPress = 0;       // Static =  Variable to retain value (like global) but only used in this routine.
  static bool encoderSwStillPressed = false;

  if(digitalRead(EncodeSW_PIN) == ButtonActive){
    debugPrint("Button pressed",1);
    if(encoderSwStillPressed == false){
      if (currentMillis - lastEncoderButtonPress > SwBounce) {     // If checked faster than SwBounce ms, assume it's a bounce and ignore
        shelfLEDsOn = !shelfLEDsOn;                                   //  If not a bounce ... Toggle LEDs ON / Off Flag
        LEDsUpdate = true;                                         // Set Flag to indicate an LED Setting has changed
        encoderSwStillPressed = true;
        
        debugPrint("Button press registered",1);
        debugPrint("              - LEDS should be ");
        debugPrint(shelfLEDsOn,1);
      }  
    }
    
  }
    else if(encoderSwStillPressed == true){
      lastEncoderButtonPress = currentMillis;                   // Update time of last change
      encoderSwStillPressed = false;
      debugPrint("Button released",1);
    }
}


// ------------------------------------------------------------------
// checkRoomTemp     checkRoomTemp     checkRoomTemp     checkRoomTemp
// ------------------------------------------------------------------
void checkRoomTemp(){
  static unsigned long lastTempCheck = 0;                       // keep track of when last converstion started
/*  
  if(currentMillis - lastTempCheck > TempSensorConvTime){         // if the last read was more than conversion time: 
    currentTempC = tempSensor.getTempC(tempSensorAddr);           // Read Current Temperature in Degrees C 
    tempSensor.requestTemperaturesByAddress(tempSensorAddr);    // Send request for new temp conversion
    lastTempCheck = currentMillis;                                // reset timer reference for conversion
    updateDisplay_FLAG = true;                                       // Set flag to trigger display update
  }
*/  
}

// ------------------------------------------------------------------
// INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT
// ------------------------------------------------------------------
void isr_EncoderKnob()  {
  static unsigned long lastInterruptTime = 0;       // Static =  Variable to retain value (like global) but only used in this routine.  Doesn't need to be volitle as only called from within the isr.
  unsigned long interruptTime = millis();

  // Turn on Display Backlight    ************************* Add when doing Display Stuff
  
  if (interruptTime - lastInterruptTime > EncoderBounce) {     // If interrupts come faster than 'EncoderBounce' ms, assume it's a bounce and ignore
//    debugPrint("knob ISR entered - Not bounce",1);    
    if (digitalRead(EncodeB_PIN) == LOW)    // If encoder moving Counter-Clockwise
    {
      if(virtualPosition > 0){
        virtualPosition-- ;             // Don't decrement if already 0
      }
    }
    else {
      if(virtualPosition < LED_PwrMax)
        virtualPosition++ ;             // Don't increment above LED_PWR_Max
    }
  
    LEDsUpdate = true;      // Set Flag to indicate an LED Setting has changed - since in ISR, Something MUST have changed
  }
  
  lastInterruptTime = interruptTime;      // Keep track of when we were here last
//  debugPrint("Exiting ISR",1);
}

void isr_ZeroCrossDetect() {
  
}


