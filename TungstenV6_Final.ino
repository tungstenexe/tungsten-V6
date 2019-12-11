/********************************************************************************************************
* Nerf Rapidstrike - Tungsten V6 - II
*
* Description
* program for Nerf Rapidstrike with single shot, burst mode and full auto. 
* Ammo counter. Combo mode
*
* created  13 Jun 2019
* modified 09 DEC 2019
* by TungstenEXE
* 
* For non commercial use
* 
* If you find my code useful, do support me by subscribing my YouTube Channel, thanks.
*
* My YouTube Channel Link - Nerf related
* https://www.youtube.com/tungstenexe
* 
* Board used      - Arduino Nano
* Pusher Motor    - 35 mm Solenoid 
* FlyWheel Motors - Worker 180 Motors
********************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include the Bounce2 library found here :
// https://github.com/thomasfredericks/Bounce-Arduino-Wiring
/////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Bounce2.h>
#include <Servo.h>

#include <SPI.h>
#include <Wire.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include the PWM library, to change the PWM Frequency for pin controlling the flywheel, found here :
// https://code.google.com/archive/p/arduino-pwm-frequency-library/downloads
// Note: unzip the files to the library folder, you might need to rename the folder name
/////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <PWM.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include the Adafruit-GFX library found here :
// https://github.com/adafruit/Adafruit-GFX-Library
/////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_GFX.h>
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include the Adafruit_SSD1306 library found here :
// https://github.com/adafruit/Adafruit_SSD1306
// you might need to comment away the line "#define SSD1306_128_32" and uncomment the line
// "#define SSD1306_128_64" so that the display works for OLED screen size 128 by 64
/////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_SSD1306.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// PIN Assigment
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// The colors for for my wiring reference only, you can use your own color

#define PIN_FLYWHEEL_MOSFET         3    // (Orange) PIN to control DC Flywheel MOSFET 
#define PIN_OLED_RESET              4    //          for OLED
#define PIN_REV                     5    // (White)  PIN listening to change in the Nerf Rev Button 
#define PIN_DARTTRIGGER             6    // (Purple) PIN listening to trigger pull event

#define PIN_SOLENOID                9    // (Purple) PIN to control solenoid

#define PIN_SELECTOR_ONE            10   // (Grey)   Selector switch one
#define PIN_SELECTOR_TWO            11   // (Green)  Selector switch two
#define PIN_DARTRESET               12   // (Brown)  PIN listening to reset counter event 
  
#define PIN_SAFETY                  A0   // (Orange) PIN Listening to safety switch
// Note                             A4      (Blue)   are used by the OLED SDA (Blue)
// Note                             A5      (Yellow) are used by the OLED SCL (Yellow)
#define PIN_VOLTREAD                A6   // (Yellow) PIN to receive voltage reading from battery 

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// End Of PIN Assigment
/////////////////////////////////////////////////////////////////////////////////////////////////////////

#define AMMO_UPPER_LIMIT            35   // Maxmimum ammo configurable
#define AMMO_LOWER_LIMIT            6    // Minimum  ammo configurable

#define BURST_UPPER_LIMIT           4    // Maxmimum burst configurable
#define BURST_LOWER_LIMIT           2    // Minimum burst configurable

#define MODE_SINGLE                 0    // Integer constant to indicate firing single shot
#define MODE_BURST                  1    // Integer constant to indicate firing burst
#define MODE_AUTO                   2    // Integer constant to indicate firing full auto
#define NUM_OF_MODE                 3    // Number of mode available

#define DEFAULT_BURSTLIMIT          3    // Default number of burst fire darts

#define MODE_ROF_LOW                0    // Integer constant to indicate low rate of fire 
#define MODE_ROF_STANDARD           1    // Integer constant to indicate standard rate of fire 
#define MODE_ROF_HIGH               2    // Integer constant to indicate highest rate of fire 
#define NUM_OF_MODE_ROF             3    // Number of ROF available
                                         
#define REV_UP_DELAY                150  // Increase/decrease this to control the flywheel rev-up time (in milliseconds) 

#define BATTERY_MIN                 9.8  // Minimum voltage of battery for rev to operate
#define BATTERY_MIN_3DIGIT          98   // Minimum voltage of battery for rev to operate
#define BATTERY_MAX_3DIGIT          123  // Maximun voltage above 12.3 is consider to be full

String  rofLimitStrArr []         = {"        <<<>>>", "     <<<<<<>>>>>>", "<<<<<<<<<<<>>>>>>>>>>>"}; 
int     modeROFSelected           = MODE_ROF_HIGH;   // track the ROF selected, set default to High

int     delaySolenoidExtended  [] = {80, 75, 60};
int     delaySolenoidRetracted [] = {80, 55, 45};
  
int     ammoLimit                 = 18;          // default set as 18 dart mag
int     burstLimit;                              // darts per burst

int     modeFire                  = MODE_SINGLE; // track the mode of fire, Single, Burst or Auto, Single by default
int     dartToBeFire              = 0;           // track amount of dart(s) to fire when trigger pulled, 0 by default
int     dartLeft                  = ammoLimit;   // track amount of dart in mag, same default value as of ammoLimit
float   currentVoltage            = 99.0;

int     fwLimitArr []             = {75, 100};   // number are in percentage
int     FW_LOW                    = 0;
int     FW_HIGH                   = 1;
int32_t frequency                 = 10000;       //frequency (in Hz) for PWM controlling Flywheel motors
int     fwSpeed;
String  speedSelStr               = "";

boolean batteryLow                = false;       // track battery status
boolean isRevving                 = false;       // track if blaster firing         
boolean isFiring                  = false;       // track if blaster firing         
boolean magOut                    = false;       // track is there a mag 
boolean safetyOn                  = false;       // track is safetyOn 

boolean isV2Mode                  = false;       // true when V2 mode is selected
boolean isV2ModeFullAuto          = false;       // true when V2 mode is on and is on full auto firing

unsigned long timerSolenoidDetect = 0;
boolean       isSolenoidExtended  = false;


Adafruit_SSD1306 display(PIN_OLED_RESET);

// Declare and Instantiate Bounce objects
Bounce btnRev            = Bounce(); 
Bounce btnTrigger        = Bounce(); 
Bounce switchSelectorOne = Bounce();
Bounce switchSelectorTwo = Bounce();
Bounce btnDartReset      = Bounce();
Bounce btnSafety         = Bounce();

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: shotFiredHandle
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void shotFiringHandle() {
  if (isFiring) {
    if (isSolenoidExtended) {
      if ((millis() - timerSolenoidDetect) >= delaySolenoidExtended[modeROFSelected]) {
        digitalWrite(PIN_SOLENOID, LOW); // Retract Solenoid

        // to refresh the whole display is too time consuming
        // therefore just overwrite the old dart count by switching to background color
        // write the current count on the screen, this remove it from the screen
        display.setTextColor(BLACK);
        display.setCursor(90,18);
        display.print(dartLeft);
  
        dartLeft--;     // decrease dart count
        dartToBeFire--;

        // switch back to white text and write the new count
        display.setTextColor(WHITE);
        display.setCursor(90,18);
        display.print(dartLeft);
        display.display();
        
        isSolenoidExtended = false;
        timerSolenoidDetect = millis();        
      }
    } else { // Solenoid had returned to rest position
      if (dartToBeFire == 0) {
        isFiring = false;
        if (!isRevving || isV2Mode) { // Rev button not pressed or in V2 mode
          isRevving = false;
          // digitalWrite(PIN_FLYWHEEL_MOSFET, LOW); // stop flywheels
          pwmWrite(PIN_FLYWHEEL_MOSFET, 0);
        }
        updateDisplay();
      } else if ((millis() - timerSolenoidDetect) >= delaySolenoidRetracted[modeROFSelected]) {
        digitalWrite(PIN_SOLENOID, HIGH); // Extend Solenoid
        isSolenoidExtended = true;
        timerSolenoidDetect = millis();
      }      
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: triggerPressedHandle
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void triggerPressedHandle(int caseModeFire) {  
  //updateSettingDisplay();
  if (dartLeft > 0) {
    if (isV2Mode && !isRevving) {
      pwmWrite(PIN_FLYWHEEL_MOSFET, fwSpeed); // start flywheels
      delay(REV_UP_DELAY);
      isRevving = true;
    }
  
    if (isRevving){
      switch(caseModeFire) {
        case MODE_SINGLE: dartToBeFire++; break;
        case MODE_BURST : dartToBeFire += burstLimit; 
            if (dartToBeFire > dartLeft) {
              dartToBeFire = dartLeft;
            }
          break;
        case MODE_AUTO  : dartToBeFire = dartLeft;
      }

      // Start Firing    
      if (!isFiring) {
        isFiring = true;
        display.setTextSize(3);
  
        digitalWrite(PIN_SOLENOID, HIGH); // extend pusher
        timerSolenoidDetect = millis();
        isSolenoidExtended = true;      
      }    
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: triggerReleasedHandle
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void triggerReleasedHandle() {  
  if ((modeFire == MODE_AUTO || isV2ModeFullAuto) && isFiring) {
    isV2ModeFullAuto = false;
    if (dartToBeFire > 1) {      
      dartToBeFire = 1;    // fire off last shot
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: readVoltage
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void readVoltage() {
  int voltagePinAnalogValue = analogRead(PIN_VOLTREAD);
    
  // you might have to adjust the formula according to the voltage sensor you use
  int   voltagePinValue = (int) (voltagePinAnalogValue / 0.3890 );
  float newVoltage      = (voltagePinValue / 100.0);

  if (!batteryLow) {
    currentVoltage = (newVoltage < currentVoltage) ? newVoltage : currentVoltage;      
  } else {
    currentVoltage = (newVoltage > BATTERY_MIN) ? newVoltage : currentVoltage;  
  }
  batteryLow = (currentVoltage <= BATTERY_MIN);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: updateNormalDisplay
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateNormalDisplay() {
  readVoltage();
  int numOfCircle = 1;
  int intCurrentVolt = (int) (currentVoltage * 10);

  if (intCurrentVolt < BATTERY_MIN_3DIGIT) {
    intCurrentVolt = BATTERY_MIN_3DIGIT;
  } else if (intCurrentVolt > BATTERY_MAX_3DIGIT) {
    intCurrentVolt = BATTERY_MAX_3DIGIT;
  }
  
  int batt = map(intCurrentVolt, BATTERY_MIN_3DIGIT, BATTERY_MAX_3DIGIT, 0, 16);

  display.clearDisplay();
  
  display.fillRect(0, 0, (8*batt), 4, WHITE);
  for (int i=1; i<=batt; i++) {
    display.drawLine(i*8, 0, i*8, 4, BLACK);
  }
  
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0,8);
  display.print(">> ");
  display.print(currentVoltage);
  display.println(" volts");  
  
  display.setCursor(0,19);
  display.print(speedSelStr);
  display.println("-TEXE-V6-<T>");  
  
  display.setCursor(0,32);
  
  switch(modeFire) {
      case MODE_SINGLE: 
          display.println("Single Shot");  
        break;
      case MODE_BURST : 
          numOfCircle = burstLimit;
          display.print(burstLimit);          
          display.println(" Rounds Burst");  
        break;
      case MODE_AUTO  : 
          numOfCircle = 10;
          display.println("Full Auto");  
        break;
    }
    
  display.setCursor(0,57);
  display.println(rofLimitStrArr[modeROFSelected]);  
  
  display.setCursor(90,18);
  display.setTextSize(3);
  display.println(dartLeft);  

  for(int i=0; i<numOfCircle; i++) {
    display.fillCircle((i * 9) + 3, 48, 3, WHITE);
  }
  display.display();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: updateV2ModeDisplay
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateV2ModeDisplay() {
  readVoltage();
  int numOfCircle = burstLimit;
  int intCurrentVolt = (int) (currentVoltage * 10);

  if (intCurrentVolt < BATTERY_MIN_3DIGIT) {
    intCurrentVolt = BATTERY_MIN_3DIGIT;
  } else if (intCurrentVolt > BATTERY_MAX_3DIGIT) {
    intCurrentVolt = BATTERY_MAX_3DIGIT;
  }
  
  int batt = map(intCurrentVolt, BATTERY_MIN_3DIGIT, BATTERY_MAX_3DIGIT, 0, 16);

  display.clearDisplay();
  
  display.fillRect(0, 0, (8*batt), 4, WHITE);
  for (int i=1; i<=batt; i++) {
    display.drawLine(i*8, 0, i*8, 4, BLACK);
  }
  
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0,8);
  display.print(">> ");
  display.print(currentVoltage);
  display.println(" volts");  
      
  display.setCursor(0,57);
  display.println(rofLimitStrArr[modeROFSelected]);  

  display.setTextSize(2);
  display.setCursor(0,23);
  display.print(speedSelStr);
  display.println("~V6II~");   

  display.setCursor(90,18);
  display.setTextSize(3);
  display.println(dartLeft);  

  for(int i=0; i<numOfCircle; i++) {
    display.fillCircle((i * 9) + 3, 48, 3, WHITE);
  }
  display.display();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: updateDisplay
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateDisplay() {
  if (isV2Mode) {
    updateV2ModeDisplay();
  } else {
    updateNormalDisplay();
  }  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: updateMagOutDisplay
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateMagOutDisplay() {
  readVoltage();
  
  int intCurrentVolt = (int) (currentVoltage * 10);

  if (intCurrentVolt < BATTERY_MIN_3DIGIT) {
    intCurrentVolt = BATTERY_MIN_3DIGIT;
  } else if (intCurrentVolt > BATTERY_MAX_3DIGIT) {
    intCurrentVolt = BATTERY_MAX_3DIGIT;
  }
  
  int batt = map(intCurrentVolt, BATTERY_MIN_3DIGIT, BATTERY_MAX_3DIGIT, 0, 16);

  display.clearDisplay();
  
  display.fillRect(0, 0, (8*batt), 4, WHITE);
  for (int i=1; i<=batt; i++) {
    display.drawLine(i*8, 0, i*8, 4, BLACK);
  }
  
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0,8);
  display.print(">> ");
  display.print(currentVoltage);
  display.println(" volts");  
  
  display.setCursor(0,19);
  display.println("MAG OUT - SET");  
      
  display.setCursor(0,57);
  display.println("<*-*-*-*-*^*-*-*-*-*>");  

  display.setTextSize(2);
  display.setCursor(0,32);
  display.println("-AMMO-");  
  
  display.setCursor(90,18);
  display.setTextSize(3);
  display.println(ammoLimit);  

  display.display();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: updateSafetyDisplay
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateSafetyDisplay() {
  readVoltage();
  int numOfCircle = burstLimit;
  int intCurrentVolt = (int) (currentVoltage * 10);

  if (intCurrentVolt < BATTERY_MIN_3DIGIT) {
    intCurrentVolt = BATTERY_MIN_3DIGIT;
  } else if (intCurrentVolt > BATTERY_MAX_3DIGIT) {
    intCurrentVolt = BATTERY_MAX_3DIGIT;
  }
  
  int batt = map(intCurrentVolt, BATTERY_MIN_3DIGIT, BATTERY_MAX_3DIGIT, 0, 16);

  display.clearDisplay();
  
  display.fillRect(0, 0, (8*batt), 4, WHITE);
  for (int i=1; i<=batt; i++) {
    display.drawLine(i*8, 0, i*8, 4, BLACK);
  }
  
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0,8);
  display.print(">> ");
  display.print(currentVoltage);
  display.println(" volts");  
      
  display.setCursor(0,57);
  display.println(rofLimitStrArr[modeROFSelected]);  

  display.setTextSize(2);
  display.setCursor(0,21);
  display.println("SAFETY");  

  display.setCursor(90,18);
  display.setTextSize(3);
  display.println(dartLeft);  

  for(int i=0; i<numOfCircle; i++) {
    display.fillCircle((i * 9) + 3, 48, 3, WHITE);
  }
  display.display();
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: updateBatteryLowDisplay
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateBatteryLowDisplay() {
  readVoltage();
  if (batteryLow) {
    display.clearDisplay();
        
    display.setTextSize(2);
    display.setCursor(0,14);
    display.println("+-------+");  
    display.println("|BattLow|=");  
    display.println("+-------+");  
  
    display.display();
  } else {
    if (magOut) {
      updateMagOutDisplay();
    } else {
      updateDisplay();
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: shutdown
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void shutdownSys() {
  dartToBeFire = 0;
  digitalWrite(PIN_SOLENOID, LOW);
  pwmWrite(PIN_FLYWHEEL_MOSFET, 0);
  isFiring = false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: setup
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() { // initilze  
  //initialize all timers except for 0, to save time keeping functions
  InitTimersSafe();

  //sets the frequency for the specified pin
  bool success = SetPinFrequencySafe(PIN_FLYWHEEL_MOSFET, frequency);
  
  // if the pin frequency was set successfully, turn pin 13 on, a visual check
  // can be commented away in final upload to arduino board
  if(success) {
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);    
  }
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // INPUT PINs setup
  // Note: Most input pins will be using internal pull-up resistor. A fall in signal indicate button pressed.
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
   
  pinMode(PIN_REV,INPUT_PULLUP);              // PULLUP
  btnRev.attach(PIN_REV);
  btnRev.interval(5);
    
  pinMode(PIN_DARTTRIGGER,INPUT_PULLUP);      // PULLUP
  btnTrigger.attach(PIN_DARTTRIGGER);
  btnTrigger.interval(5);

  pinMode(PIN_SELECTOR_ONE,INPUT_PULLUP);     // PULLUP
  switchSelectorOne.attach(PIN_SELECTOR_ONE);
  switchSelectorOne.interval(5);

  pinMode(PIN_SELECTOR_TWO,INPUT_PULLUP);     // PULLUP
  switchSelectorTwo.attach(PIN_SELECTOR_TWO);
  switchSelectorTwo.interval(5);

  pinMode(PIN_DARTRESET, INPUT_PULLUP);       // PULLUP
  btnDartReset.attach(PIN_DARTRESET);
  btnDartReset.interval(5);

  pinMode(PIN_SAFETY, INPUT_PULLUP);          // PULLUP
  btnSafety.attach(PIN_SAFETY);  
  btnSafety.interval(5);

  pinMode(PIN_VOLTREAD, INPUT);               // Not using PULLUP analog read 0 to 1023

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // OUTPUT PINs setup
  ///////////////////////////////////////////////////////////////////////////////////////////////////////  

  pinMode (PIN_FLYWHEEL_MOSFET, OUTPUT);
  pwmWrite(PIN_FLYWHEEL_MOSFET, 0);  

  digitalWrite(PIN_SOLENOID, LOW);
  pinMode(PIN_SOLENOID, OUTPUT);
  
  magOut   = (digitalRead(PIN_DARTRESET) == HIGH);
  safetyOn = (digitalRead(PIN_SAFETY) == LOW);
  modeFire = 3 - ((digitalRead(PIN_SELECTOR_TWO) * 2) + (digitalRead(PIN_SELECTOR_ONE) * 1)); // 0, 1 or 2        

  isV2Mode   = (digitalRead(PIN_DARTTRIGGER) == LOW);
  burstLimit = (isV2Mode) ? modeFire + 2 : DEFAULT_BURSTLIMIT;

  fwSpeed     = (digitalRead(PIN_REV) == LOW) ? map(fwLimitArr[FW_HIGH] , 0, 100, 0, 255) : map(fwLimitArr[FW_LOW] , 0, 100, 0, 255);
  speedSelStr = (digitalRead(PIN_REV) == LOW) ? "H" : "L";
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  display.clearDisplay();
  if (magOut) {
    dartLeft     = 0;
    updateMagOutDisplay(); 
  } else if (safetyOn) { 
    updateSafetyDisplay();
  } else {
    dartLeft = ammoLimit;
    updateDisplay();   
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function: loop
//           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() { // Main Loop  
  if (!batteryLow) {  
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    // Update all buttons
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    
    btnRev.update();
    btnTrigger.update();
    switchSelectorOne.update();
    switchSelectorTwo.update();    
    btnDartReset.update();    
    btnSafety.update();
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Listen to Mag Out
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    if (btnDartReset.fell()) { // pressed, there is a Mag in the blaster
      dartLeft = ammoLimit;
      magOut   = false;      
      
      if (safetyOn) {
        updateSafetyDisplay();
      } else {
        updateDisplay();
      }
    } else if (btnDartReset.rose()) { // No Mag in the blaster      
      shutdownSys();
      magOut = true;
      updateMagOutDisplay();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Listen to Safety
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    if (btnSafety.fell()) {               // Safety on
      safetyOn = true;
      shutdownSys(); 
      
      if (!magOut) {
        updateSafetyDisplay();
      }
    } else if (btnSafety.rose()) {        // Safety off
      safetyOn = false;
      if (!magOut) {
        updateDisplay();
      }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Listen to Rev Press/Release
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    if (btnRev.fell()) {                   // press
      if (magOut) {
        if (ammoLimit > AMMO_LOWER_LIMIT) {
          ammoLimit--;      
          updateMagOutDisplay();
        }
      } else if (safetyOn) {
        if (digitalRead(PIN_DARTTRIGGER) == LOW ) {
          modeROFSelected = ++modeROFSelected % NUM_OF_MODE_ROF;
          updateSafetyDisplay();
        }      
      } else if (isV2Mode) { 
        if (digitalRead(PIN_DARTTRIGGER) == HIGH) {
          triggerPressedHandle(MODE_BURST);
          isV2ModeFullAuto = false;
        } else {
          triggerPressedHandle(MODE_AUTO);
          isV2ModeFullAuto = true;
        }      
      } else {      
        isRevving = true;
        // digitalWrite(PIN_FLYWHEEL_MOSFET, HIGH); // start flywheels
        pwmWrite(PIN_FLYWHEEL_MOSFET, fwSpeed);        
      }
    } else if (btnRev.rose() && !isV2Mode) {        // released
      isRevving = false;
      if (!isFiring) {        
        // digitalWrite(PIN_FLYWHEEL_MOSFET, LOW); // stop flywheels
        pwmWrite(PIN_FLYWHEEL_MOSFET, 0);
      }
    }
  
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Listen to Trigger Pull/Release
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    if (btnTrigger.fell()) {               // pull
      if (magOut) {
        if (ammoLimit < AMMO_UPPER_LIMIT) {
          ammoLimit++;      
          updateMagOutDisplay();
        }
      } else if (safetyOn) {
        if (digitalRead(PIN_REV) == LOW && !isV2Mode) {
          burstLimit = (burstLimit == BURST_UPPER_LIMIT) ? BURST_LOWER_LIMIT : burstLimit + 1;
          updateSafetyDisplay();
        }
      } else if (isV2Mode) {
        if (digitalRead(PIN_REV) == HIGH) {
          triggerPressedHandle(MODE_SINGLE);
          isV2ModeFullAuto = false;
        } else {
          triggerPressedHandle(MODE_AUTO);
          isV2ModeFullAuto = true;
        }    
      } else {
        triggerPressedHandle(modeFire);
      }        
    } else if (btnTrigger.rose()) {        // released
      triggerReleasedHandle();
    }
  
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Listen to Firing
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    shotFiringHandle();
  
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Listen to Firing Mode change: Single Shot, Burst, Full Auto
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    if (switchSelectorOne.fell() || switchSelectorOne.rose() || switchSelectorTwo.fell() || switchSelectorTwo.rose() ) {  
      modeFire = 3 - ((digitalRead(PIN_SELECTOR_TWO) * 2) + (digitalRead(PIN_SELECTOR_ONE) * 1)); // 0, 1 or 2    
      
      if (isV2Mode) {
        burstLimit = modeFire + 2;
      }
      
      if (!magOut && !safetyOn) {
        updateDisplay();
      }
    }    
  } else {
    // Battery is Low
    // Stop all Motors just in case.
    shutdownSys();    
    updateBatteryLowDisplay();
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// END OF PROGRAM           
/////////////////////////////////////////////////////////////////////////////////////////////////////////
