/*,,,
PROGRAM BASE

- Expecting 3 diferent inputs from the IR sensor for 4 different steps.
Steps:
1. Moist hands with a bit of water in WATER_OUT
2. Dispense liquid soap in SOAP_OUT
3. Wait until 20 seconds of hand rubbing is done (flash led if input is done in this timeframe)
4. Open WATER_OUT to wash out soap

Extra. Reset cycle if nothing happens for more than 30 seconds
*/
/*
  HIGH SENSITIBITY AND AUTOMATICALLY AMBIENT REJECTING INFRARED SENSOR
  This microcontroller program was made to work in attiny85 to run in
  the arduino development environment, for further compatibility pins
  should be remmaped according to the characteristics of your micro-
  controller or development board.
  Routine and algorithm designed by Harry Lisby
  Webpage: lisby.co
  2017 - 11 - 10
TO-DO:
  >> Better timing adjustment method
  >> Extra timing step between moist and soap - Version 1 implemented
  >> Save timing adjustment into eeprom - Version 1 implemented
*/

#include <EEPROM.h> //EEPROM library for STM32duino


#define BOARD_V10 //1.0 board version

#ifdef BOARD_V10
//Washing state machine related variables
#define IR_IN PB15 //Input from sensor side of the circuit
//#define GLOBAL_BUTTON_IN PA8//GOTO GLOBAL_BUTTON_IN ... Equivalent to IR_IN, allows for an external input instead of sensor
#define WATER_OUT PA15
#define SOAP_OUT PB3
#define DRYER_OUT PB4
#endif
#ifndef BOARD_V10
//Washing state machine related variables
#define IR_IN PB15 //Input from sensor side of the circuit
//#define GLOBAL_BUTTON_IN PA8//GOTO GLOBAL_BUTTON_IN ... Equivalent to IR_IN, allows for an external input instead of sensor
#define WATER_OUT PA9  //PA15 board
#define SOAP_OUT PA10  //PB3 board
#define DRYER_OUT PA8  //PB4 board
#endif

//state machine leds for each step
#define MOIST_STEP_INDICATOR PB6
#define SOAP_STEP_INDICATOR PB7
#define RUBBING_STEP_INDICATOR PB8
#define WASHING_STEP_INDICATOR PB9
#define DRYING_STEP_INDICATOR PB5

//Timing values
uint16_t MOIST_TIME = 3000; //time to wet hands
uint16_t PAUSE_TIME = 1500; //dwell time to prevent soap from falling
uint16_t SOAP_TIME = 3000; //time to soap hands
uint16_t RUBBING_TIME = 20000; //time to rub hands
uint16_t WASHING_TIME = 18000; //time to wash away soap and dirt!
uint16_t ENDING_TIME = 7500; //time to prevent incorrect input after wash

//IR sensor related variables
#define IR_OUT   PB12   //IR LED output2
//#define RLAY_OUT PB13   //Relay ON/OFF output
#define ANLG_IN  PB1  //Reads input photodiode
#define GLOBAL_ANALOG_IN PA0  //IR_SENSITIVITY adjustment pin
#define GLOBAL_BUTTON_IN PA8  //IR FORCE input
//#define TIME_ADJ A3  //Output ON time
#define DETECT_MODE PB13 //detect on interruption by default

//debug synchro
#define READ_CAL_DEBUG
#ifdef READ_CAL_DEBUG
#define READ_CAL_DEBUG_OUT PB14 //this optional input will send a signal during analogRead cycle.
#endif //READ_CAL_DEBUG


#define ON_TIME 10000   //On time for output pin (in microseconds)
#define RUN_SPD 400     //Per-cycle time (in microseconds)
#define ADJ_SPD 2000    //register new variables time
#define HC_ACT_TMR 200    //define delay to read input

#define TIMER_SCALE 8
#define IR_READ_DELAY 200

#define IR_PULSES 3

bool butStat=true;
int16_t counter = 0;
int16_t preRead = 0;
int16_t afterRead = 0;
int16_t IR_SENSITIVITY = 1000;
int16_t globalDiff = 0;
int32_t timer = 0;
int32_t nonMapTimer,nonMapIR_SENSITIVITY;

uint8_t sMachineStateStorage = 0; //saves current value for state machine

uint32_t mainTime,mainTimeLast,adjTimeLast,offDelay;

uint32_t timeTrack = 0;
uint32_t timeTrackPrevious = 0;
bool currentForceInState=1,lastForceInState=1;
bool sensorDetectFlag = 0, inhibitSensor=0;

bool once1,once2,once3,once4,once5,once6;

uint32_t stateMachineTimeTrack,moistTimer,pauseTimer,soapTimer,rubbingTimer,washingTimer,noActionTimer;

bool timersButtonFlag=false;
bool timersAdvanceStep=false;
uint8_t timersStep=0;

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306_STM32.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

#define BARPOS_X 10
#define BARPOS_Y 40
#define BAR_WIDTH 110
#define BAR_HEIGHT 12

const unsigned char lisbyBMP [] PROGMEM = {
	// 'bitmap, 128x64px
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0xc0, 0x01, 0x80, 0x1f, 0x00, 0x3f, 0x00, 0x30, 0xc0, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0xc0, 0x01, 0x80, 0x1f, 0x80, 0x3f, 0x80, 0x30, 0xc0, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0xc0, 0x01, 0x80, 0x31, 0x80, 0x31, 0x80, 0x38, 0x80, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0xc0, 0x01, 0x80, 0x31, 0x80, 0x31, 0x80, 0x19, 0x80, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0xc0, 0x01, 0x80, 0x30, 0x00, 0x31, 0x80, 0x19, 0x80, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0xc0, 0x01, 0x80, 0x38, 0x00, 0x31, 0x80, 0x19, 0x80, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0xc0, 0x01, 0x80, 0x1c, 0x00, 0x33, 0x80, 0x09, 0x00, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0xc0, 0x01, 0x80, 0x1e, 0x00, 0x3f, 0x00, 0x0f, 0x00, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0xc0, 0x01, 0x80, 0x0f, 0x00, 0x3f, 0x80, 0x0f, 0x00, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0xc0, 0x01, 0x80, 0x07, 0x00, 0x31, 0x80, 0x06, 0x00, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0xc0, 0x01, 0x80, 0x03, 0x80, 0x31, 0x80, 0x06, 0x00, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0xc0, 0x01, 0x80, 0x01, 0x80, 0x31, 0x80, 0x06, 0x00, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0xc0, 0x01, 0x80, 0x31, 0x80, 0x31, 0x80, 0x06, 0x00, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0xc0, 0x01, 0x80, 0x31, 0x80, 0x31, 0x80, 0x06, 0x00, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0xfc, 0x01, 0x80, 0x3f, 0x80, 0x3f, 0x80, 0x06, 0x00, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0xfc, 0x01, 0x80, 0x1f, 0x00, 0x3f, 0x00, 0x06, 0x00, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void firstEEPROMPROG(){
  EEPROM.write(0,MOIST_TIME);
  //EEPROM.write(2,PAUSE_TIME);
  EEPROM.write(4,SOAP_TIME);
  EEPROM.write(6,RUBBING_TIME);
  EEPROM.write(8,WASHING_TIME);
  //EEPROM.write(10,ENDING_TIME);
}

void readEEPROM(){
  EEPROM.read(0,&MOIST_TIME);
  //EEPROM.read(2,&PAUSE_TIME);
  EEPROM.read(4,&SOAP_TIME);
  EEPROM.read(6,&RUBBING_TIME);
  EEPROM.read(8,&WASHING_TIME);
  //EEPROM.read(10,&ENDING_TIME);
}

void setup() {
  enableDebugPorts();

  //state machine startup
  pinMode(IR_IN,INPUT_PULLUP);
  pinMode(GLOBAL_BUTTON_IN,INPUT_PULLUP);
  pinMode(WATER_OUT,OUTPUT);
  pinMode(SOAP_OUT,OUTPUT);
  pinMode(DRYER_OUT,OUTPUT);

  pinMode(SOAP_STEP_INDICATOR,OUTPUT);
  pinMode(MOIST_STEP_INDICATOR,OUTPUT);
  pinMode(RUBBING_STEP_INDICATOR,OUTPUT);
  pinMode(WASHING_STEP_INDICATOR,OUTPUT);
  pinMode(DRYING_STEP_INDICATOR,OUTPUT);


  //IR Circuit startup
  pinMode(IR_OUT,OUTPUT);
//  pinMode(RLAY_OUT,OUTPUT);
  //pinMode(ANLG_IN,INPUT);
  //pinMode(GLOBAL_BUTTON_IN,INPUT_PULLUP); //button input mistaken
  //pinMode(TIME_ADJ,INPUT);
  pinMode(DETECT_MODE,INPUT_PULLUP);

  #ifdef READ_CAL_DEBUG
  pinMode(READ_CAL_DEBUG_OUT,OUTPUT);
  #endif //READ_CAL_DEBUG

  // digitalWrite(RLAY_OUT,HIGH);
  // delay(100);
  // digitalWrite(RLAY_OUT,LOW);

  Serial.begin(115200);

  //Display OLED initialization
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
   // init done
   // Clear the buffer.
   delay(500);
   display.clearDisplay();
   display.setTextSize(2);
   display.setTextColor(WHITE);

   timerAdjustmentRoutine(); //if global button is pressed, timer adjustment mode is entered

   //Lisby splashscreen
   display.drawBitmap(0,0,lisbyBMP,128,62,WHITE);
   display.display();
   delay(2000);
   display.clearDisplay();

   // text display tests
   display.setTextSize(1);
   display.setTextColor(WHITE);
   display.setCursor(16,0);
   display.println("LISBY INDUSTRIAL");
   //display.setTextColor(BLACK, WHITE); // 'inverted' text
   display.setCursor(21,10);
   display.println("Inicializando");
   display.setTextSize(2);
   display.setTextColor(WHITE);
   display.setCursor(20,32);
   display.println("ECO-LAV");
   display.display();
   delay(2000);

   // display.clearDisplay();
   // display.setCursor(0,0);
   // display.println("Coloque su  mano en el sensor");
   // display.display();
   // delay(2000);



   //firstEEPROMPROG(); //only activate on new microcontrollers
   readEEPROM();

}

void loop() {
  timeTrack = millis();

  if(timeTrack-timeTrackPrevious>=350){
    currentForceInState=digitalRead(GLOBAL_BUTTON_IN);
    if((sensorDetectFlag)||((currentForceInState!=lastForceInState)&&(!currentForceInState))){
      Serial.println("Sequence activated");
      sensorDetectFlag=false;
      inhibitSensor=true;
      lastForceInState=currentForceInState; //store current state for future comparison
      sMachineStateStorage++; //increment stateMachine by 1
      if(sMachineStateStorage>=6)sMachineStateStorage=0; //reset value if it overflows
    }else if(currentForceInState){
      lastForceInState=currentForceInState; //reset state if onbutton is not pressed
    }
    stateMachine(); //this function decodes the state machine steps
    timeTrackPrevious=timeTrack; //update last tracking time value
  }

  //if(!inhibitSensor)IR_SENSOR(); //this is activated only when inhibitSensor is set to false;
  if(!inhibitSensor)IR_SENSOR_MA_ALGORITHM(); //this is activated only when inhibitSensor is set to false;

}


// void drawStepStatus(string stepTitle, uint32_t stepTimer, uint32_t stepSetTime){
//   display.clearDisplay();
//   display.setTextColor(WHITE);
//   display.setTextSize(2);
//   display.setCursor(0,0);
//   display.print(stepTitle);
//   display.drawRoundRect(BARPOS_X, BARPOS_Y, BAR_WIDTH, BAR_HEIGHT, 3, WHITE); //(x,y,width,height,radius,color)
//   caseTracker = BAR_WIDTH*(stateMachineTimeTrack-stepTimer)/(stepSetTime);
//   display.drawRoundRect(BARPOS_X, BARPOS_Y, caseTracker, BAR_HEIGHT, 3, WHITE); //(x,y,width,height,radius,color)
//   display.display();
// }



void stateMachine(){
uint8_t cState = sMachineStateStorage;
stateMachineTimeTrack=millis();
uint32_t caseTracker = 0;
  switch(cState){
    //flash LEDs depending on corresponding state
    case 0:
      //0. Wait for initial sensor trigger input
      //nothing needs to be done in this step, just wait


      //Write all outputs to LOW to ensure level
      digitalWrite(WATER_OUT,LOW);
      digitalWrite(SOAP_OUT,LOW);
      digitalWrite(DRYER_OUT,LOW);
      digitalWrite(DRYING_STEP_INDICATOR,LOW);

      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0,0);
      display.print("COLOCAR \nMANOS PARA\nINICIAR");
      display.setTextSize(1);
      display.setCursor(0,57);
      display.print("R: ");
      display.print(globalDiff);
      display.setCursor(64,57);
      display.print("S: ");
      display.print(IR_SENSITIVITY);
      display.display();

      once6=false; //reset last washing flag

      break;
    case 1:
      //1. Moist hands with a bit of water in WATER_OUT
      digitalWrite(MOIST_STEP_INDICATOR,HIGH); //turn on step LED
      digitalWrite(WASHING_STEP_INDICATOR,LOW); //ensure last LED is always off

      if(!once1){
        once1=true; //trigger flag to prevent more executions
        digitalWrite(WATER_OUT,HIGH); //turn on water output
        moistTimer=stateMachineTimeTrack;//register time
      }else if(stateMachineTimeTrack-moistTimer>=MOIST_TIME){
        digitalWrite(WATER_OUT,LOW); //turn off water output
        sMachineStateStorage++;
      }
      //wait for next input...

      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setTextSize(2);
      display.setCursor(16,23);
      display.print("1.MOJADO");
      display.drawRoundRect(BARPOS_X, BARPOS_Y, BAR_WIDTH, BAR_HEIGHT, 3, WHITE); //(x,y,width,height,radius,color)
      caseTracker = constrain(BAR_WIDTH*(stateMachineTimeTrack-moistTimer)/(MOIST_TIME),0,BAR_WIDTH);
      display.fillRoundRect(BARPOS_X, BARPOS_Y, caseTracker, BAR_HEIGHT, 3, WHITE); //(x,y,width,height,radius,color)
      display.display();

      //drawStepStatus("1.MOJADO",moistTimer,100);


      break;
    case 2:
      //2. Wait to prevent washing down the soap
      once1=false; //reset moist flag
      digitalWrite(SOAP_STEP_INDICATOR,LOW);
      digitalWrite(MOIST_STEP_INDICATOR,LOW);

      if(!once2){
        once2=true; //trigger flag to prevent more executions
        pauseTimer=stateMachineTimeTrack;//register time
      }else if(stateMachineTimeTrack-pauseTimer>=PAUSE_TIME){
        sMachineStateStorage++;
      }
      //wait for next input...

      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setTextSize(2);
      display.setCursor(16,23);
      display.print("x.PAUSA");
      display.drawRoundRect(BARPOS_X, BARPOS_Y, BAR_WIDTH, BAR_HEIGHT, 3, WHITE); //(x,y,width,height,radius,color)
      caseTracker = constrain(BAR_WIDTH*(stateMachineTimeTrack-pauseTimer)/(PAUSE_TIME),0,BAR_WIDTH);
      display.fillRoundRect(BARPOS_X, BARPOS_Y, caseTracker, BAR_HEIGHT, 3, WHITE); //(x,y,width,height,radius,color)
      display.display();

      break;
    case 3:
      //3. Dispense liquid soap in SOAP_OUT
      once2=false; //reset pause flag
      digitalWrite(SOAP_STEP_INDICATOR,HIGH);
      digitalWrite(MOIST_STEP_INDICATOR,LOW);
      if(!once3){
        once3=true; //trigger flag to prevent more executions
        digitalWrite(SOAP_OUT,HIGH); //turn on water output
        soapTimer=stateMachineTimeTrack;//register time
      }else if(stateMachineTimeTrack-soapTimer>=SOAP_TIME){
        digitalWrite(SOAP_OUT,LOW); //turn off water output
        sMachineStateStorage++;
      }
      //wait for next input...

      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setTextSize(2);
      display.setCursor(18,23);
      display.print("2.JABON");
      display.drawRoundRect(BARPOS_X, BARPOS_Y, BAR_WIDTH, BAR_HEIGHT, 3, WHITE); //(x,y,width,height,radius,color)
      caseTracker = constrain(BAR_WIDTH*(stateMachineTimeTrack-soapTimer)/(SOAP_TIME),0,BAR_WIDTH);
      display.fillRoundRect(BARPOS_X, BARPOS_Y, caseTracker, BAR_HEIGHT, 3, WHITE); //(x,y,width,height,radius,color)
      display.display();

      break;
    case 4:
      //4. Wait until 20 seconds of hand rubbing is done (flash led if input is done in this timeframe)
      once3=false;
      digitalWrite(RUBBING_STEP_INDICATOR,HIGH);
      digitalWrite(SOAP_STEP_INDICATOR,LOW);
      if(!once4){
        once4=true; //trigger flag to prevent more executions
        rubbingTimer=stateMachineTimeTrack;//register time
      }else if(stateMachineTimeTrack-rubbingTimer>=RUBBING_TIME){
        sMachineStateStorage++;
      }

      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setTextSize(2);
      display.setCursor(14,23);
      display.print("3.LAVADO");
      display.drawRoundRect(BARPOS_X, BARPOS_Y, BAR_WIDTH, BAR_HEIGHT, 3, WHITE); //(x,y,width,height,radius,color)
      caseTracker = constrain(BAR_WIDTH*(stateMachineTimeTrack-rubbingTimer)/(RUBBING_TIME),0,BAR_WIDTH);
      display.fillRoundRect(BARPOS_X, BARPOS_Y, caseTracker, BAR_HEIGHT, 3, WHITE); //(x,y,width,height,radius,color)
      display.display();

      break;
    case 5:
      //5. Open WATER_OUT to wash out soap for 15s
      once4=false;
      digitalWrite(WASHING_STEP_INDICATOR,HIGH);
      digitalWrite(RUBBING_STEP_INDICATOR,LOW);
      if(!once5){
        once5=true; //trigger flag to prevent more executions
        digitalWrite(WATER_OUT,HIGH); //turn on water output
        washingTimer=stateMachineTimeTrack;//register time
      }else if(stateMachineTimeTrack-washingTimer>=WASHING_TIME){
        digitalWrite(WATER_OUT,LOW); //turn off water output
        sMachineStateStorage++;
        //inhibitSensor=false; //enable the sensor again
      }

      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setTextSize(2);
      display.setCursor(6,23);
      display.print("4.ENJUAGUE");
      display.drawRoundRect(BARPOS_X, BARPOS_Y, BAR_WIDTH, BAR_HEIGHT, 3, WHITE); //(x,y,width,height,radius,color)
      caseTracker = constrain(BAR_WIDTH*(stateMachineTimeTrack-washingTimer)/(WASHING_TIME),0,BAR_WIDTH);
      display.fillRoundRect(BARPOS_X, BARPOS_Y, caseTracker, BAR_HEIGHT, 3, WHITE); //(x,y,width,height,radius,color)
      display.display();

      break;
    case 6:
      once5=false;
      digitalWrite(WASHING_STEP_INDICATOR,LOW);
      digitalWrite(DRYING_STEP_INDICATOR,HIGH);
      if(!once6){
        once6=true; //trigger flag to prevent more executions
        digitalWrite(DRYER_OUT,HIGH);
        noActionTimer=stateMachineTimeTrack;//register time
      }else if(stateMachineTimeTrack-noActionTimer>=ENDING_TIME){
        digitalWrite(DRYER_OUT,LOW);
        sMachineStateStorage=0;
        inhibitSensor=false; //enable the sensor again
      }

      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setTextSize(2);
      display.setCursor(16,23);
      display.print("5.SECADO");
      display.drawRoundRect(BARPOS_X, BARPOS_Y, BAR_WIDTH, BAR_HEIGHT, 3, WHITE); //(x,y,width,height,radius,color)
      caseTracker = constrain(BAR_WIDTH*(stateMachineTimeTrack-noActionTimer)/(ENDING_TIME),0,BAR_WIDTH);
      display.fillRoundRect(BARPOS_X, BARPOS_Y, caseTracker, BAR_HEIGHT, 3, WHITE); //(x,y,width,height,radius,color)
      display.display();

      break;
  }
}

void IR_SENSOR(){
  mainTime=millis();

  if((mainTime-adjTimeLast)>ADJ_SPD){ //Ejecuta cada ADJ_SPD milisegundos
    adjTimeLast=mainTime;
    IR_SENSITIVITY = map(analogRead(GLOBAL_ANALOG_IN),0,4095,0,4095);
    butStat = digitalRead(GLOBAL_BUTTON_IN);  //read GLOBAL_BUTTON_IN input button
  }


  if((mainTime-mainTimeLast)>RUN_SPD){ //Ejecuta cada RUN_SPD milisegundos
    mainTimeLast=mainTime;
    bool detect_algorithm_flag = digitalRead(DETECT_MODE);

    if(detect_algorithm_flag){ //Jumper in pos1
      for (int i = 0; i < IR_PULSES; i++) {
        if(pulsing()>IR_SENSITIVITY){ //Detects if the pulse is higher than sensitivity
          counter++;
          if(counter>IR_PULSES)counter=0;
        }else{
          counter=0;
        }
      }
      if(counter == IR_PULSES) {
        Serial.println("Mode: 1");
        sensorDetectFlag=true;
        counter=0;
      }
    }else if(!detect_algorithm_flag){ //Jumper in pos2
      for (int i = 0; i < IR_PULSES; i++) {
        if(pulsing()<IR_SENSITIVITY){  //detects if pulse is lower than sensitivity
          counter++;
          if(counter>IR_PULSES)counter=0;
        }else{
          counter=0;
        }
      }
      if(counter == IR_PULSES){
        Serial.println("Mode: 2");
        sensorDetectFlag = true;
        counter=0;
      }
    }
  }
}

uint16_t movingAverageMatrix[20] = {0}; //tracks the last 20 values
uint16_t movingAverageResult = 0; //saves the average of the last 20 values
uint8_t mAPos = 0;
bool firstTimeMA=true;

void IR_SENSOR_MA_ALGORITHM(){
  mainTime=millis();

  if((mainTime-adjTimeLast)>ADJ_SPD){ //Ejecuta cada ADJ_SPD milisegundos
    adjTimeLast=mainTime;
    IR_SENSITIVITY = map(analogRead(GLOBAL_ANALOG_IN),0,4095,0,4095);
    butStat = digitalRead(GLOBAL_BUTTON_IN);  //read GLOBAL_BUTTON_IN input button
  }

  if((mainTime-mainTimeLast)>RUN_SPD){ //Ejecuta cada RUN_SPD milisegundos
    mainTimeLast=mainTime;
    bool detect_algorithm_flag = digitalRead(DETECT_MODE);

    if(firstTimeMA){
      for(int i = 0; i < 20; i++){
        movingAverageMatrix[i]=pulsing();
        delay(100);
      }
      firstTimeMA=false;
    }

    movingAverageMatrix[mAPos] = pulsing();

    if(mAPos>=20)mAPos=0;

    uint32_t sumForAverage = 0;
    for(int i = 0; i < 20; i++){
      sumForAverage = sumForAverage + movingAverageMatrix[i];
    }
    movingAverageResult=sumForAverage/20;

    if(detect_algorithm_flag){ //Jumper in pos1: reflection algoritm
      for (int i = 0; i < IR_PULSES; i++) {
        if(movingAverageMatrix[mAPos]>IR_SENSITIVITY){ //Detects if the pulse is higher than sensitivity
          counter++;
          if(counter>IR_PULSES)counter=0;
        }else{
          counter=0;
        }
      }
      if(counter == IR_PULSES) {
        Serial.println("Mode: 1");
        sensorDetectFlag=true;
        counter=0;
      }
    }else if(!detect_algorithm_flag){ //Jumper in pos2: detection algorithm
      for (int i = 0; i < IR_PULSES; i++) {
        if(movingAverageMatrix[mAPos]<IR_SENSITIVITY){  //detects if pulse is lower than sensitivity
          counter++;
          if(counter>IR_PULSES)counter=0;
        }else{
          counter=0;
        }
      }
      if(counter == IR_PULSES){
        Serial.println("Mode: 2");
        sensorDetectFlag = true;
        counter=0;
      }
    }
    mAPos++; //increase moving average cell
  }
}

int16_t pulsing(){
  preRead = analogRead(ANLG_IN);
  #ifdef READ_CAL_DEBUG //Debug routine
  digitalWrite(READ_CAL_DEBUG_OUT,HIGH);
  #endif //READ_CAL_DEBUG
  digitalWrite(IR_OUT, HIGH);
  delayMicroseconds(IR_READ_DELAY);

  #ifdef READ_CAL_DEBUG //debug routine
  digitalWrite(READ_CAL_DEBUG_OUT,LOW);
  #endif //READ_CAL_DEBUG

  int difference_calc = abs(preRead-analogRead(ANLG_IN));

  digitalWrite(IR_OUT, LOW);
  delayMicroseconds(IR_READ_DELAY);

  // Serial.print("Difference: ");
  // Serial.println(difference_calc);
  globalDiff=difference_calc;
  return difference_calc;
}

void indicate_out(){
  // digitalWrite(RLAY_OUT,HIGH);
  // delay(50);
  // digitalWrite(RLAY_OUT,LOW);
  // delay(50);
}

void timerAdjustmentRoutine(){  //this function executes in the setup code, if the GLOBAL_BUTTON_IN button is pressed, this is executed
  if(!digitalRead(GLOBAL_BUTTON_IN)){
    timersButtonFlag=true; //flag active
    Serial.println("timer_config_mode");
    for(int x = 0; x<5; x++){ //indicate with flashy lights
      digitalWrite(MOIST_STEP_INDICATOR,HIGH);
      digitalWrite(SOAP_STEP_INDICATOR,HIGH);
      digitalWrite(RUBBING_STEP_INDICATOR,HIGH);
      digitalWrite(WASHING_STEP_INDICATOR,HIGH);
      delay(300);
      digitalWrite(MOIST_STEP_INDICATOR,LOW);
      digitalWrite(SOAP_STEP_INDICATOR,LOW);
      digitalWrite(RUBBING_STEP_INDICATOR,LOW);
      digitalWrite(WASHING_STEP_INDICATOR,LOW);
      delay(150);
    }
  }
  while(timersButtonFlag==true){ //keep inside this function while on it
    if(!digitalRead(GLOBAL_BUTTON_IN)&&!timersAdvanceStep){
      timersAdvanceStep=true;
      Serial.println("timer_next_step");
      timersStep++;
      delay(750);
    }

    switch(timersStep){
      case 0: //record MOIST_TIME
        digitalWrite(MOIST_STEP_INDICATOR,HIGH);
        digitalWrite(WASHING_STEP_INDICATOR,LOW);
        MOIST_TIME = map(analogRead(GLOBAL_ANALOG_IN),0,4095,30000,0);
        display.clearDisplay();
        display.setCursor(0,0);
        display.print("T.MOJADO:");
        display.setCursor(0,32);
        display.print(MOIST_TIME); display.print("ms");
        display.display();

        EEPROM.write(0,MOIST_TIME);
        timersAdvanceStep=false;
        break;
      case 1: //record SOAP_TIME
        digitalWrite(SOAP_STEP_INDICATOR,HIGH);
        digitalWrite(MOIST_STEP_INDICATOR,LOW);
        SOAP_TIME = map(analogRead(GLOBAL_ANALOG_IN),0,4095,30000,0);
        display.clearDisplay();
        display.setCursor(0,0);
        display.print("T.JABON:");
        display.setCursor(0,32);
        display.print(SOAP_TIME); display.print("ms");
        display.display();
        EEPROM.write(4,SOAP_TIME);
        timersAdvanceStep=false;
        break;
      case 2: //record RUBBING_TIME
        digitalWrite(RUBBING_STEP_INDICATOR,HIGH);
        digitalWrite(SOAP_STEP_INDICATOR,LOW);
        RUBBING_TIME = map(analogRead(GLOBAL_ANALOG_IN),0,4095,30000,0);
        display.clearDisplay();
        display.setCursor(0,0);
        display.print("T.ENJUAGUE:");
        display.setCursor(0,32);
        display.print(RUBBING_TIME); display.print("ms");
        display.display();
        EEPROM.write(6,RUBBING_TIME);
        timersAdvanceStep=false;
        break;
      case 3: //record WASHING_TIME
        digitalWrite(WASHING_STEP_INDICATOR,HIGH);
        digitalWrite(RUBBING_STEP_INDICATOR,LOW);
        WASHING_TIME = map(analogRead(GLOBAL_ANALOG_IN),0,4095,30000,0);
        display.clearDisplay();
        display.setCursor(0,0);
        display.print("T.LAVADO:");
        display.setCursor(0,32);
        display.print(WASHING_TIME); display.print("ms");
        display.display();
        EEPROM.write(8,WASHING_TIME);
        timersAdvanceStep=false;
        break;
      case 4:
        timersButtonFlag=false; //get out of timer adjustment mode
        for(int x = 0; x<2; x++){ //indicate with flashy lights and overkill animation lol
          digitalWrite(MOIST_STEP_INDICATOR,HIGH);
          digitalWrite(SOAP_STEP_INDICATOR,HIGH);
          digitalWrite(RUBBING_STEP_INDICATOR,HIGH);
          digitalWrite(WASHING_STEP_INDICATOR,HIGH);
          delay(300);
          digitalWrite(MOIST_STEP_INDICATOR,LOW);
          digitalWrite(SOAP_STEP_INDICATOR,LOW);
          digitalWrite(RUBBING_STEP_INDICATOR,LOW);
          digitalWrite(WASHING_STEP_INDICATOR,LOW);
          delay(100);
        }
        digitalWrite(MOIST_STEP_INDICATOR,HIGH);
        delay(200);
        digitalWrite(SOAP_STEP_INDICATOR,HIGH);
        digitalWrite(MOIST_STEP_INDICATOR,LOW);
        delay(200);
        digitalWrite(RUBBING_STEP_INDICATOR,HIGH);
        digitalWrite(SOAP_STEP_INDICATOR,LOW);
        delay(200);
        digitalWrite(WASHING_STEP_INDICATOR,HIGH);
        digitalWrite(RUBBING_STEP_INDICATOR,LOW);
        delay(200);
        digitalWrite(WASHING_STEP_INDICATOR,LOW);
        break;
    }

    if(millis()%1000==0){
      Serial.print("Read: ");
      Serial.println(map(analogRead(GLOBAL_ANALOG_IN),0,4095,30000,0));
    }

    //eeprom write stage
  }
}
