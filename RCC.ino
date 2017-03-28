/*-----( Feature Definitions )-----*/
const int CONFIG_VERSION = 1;

#include <Wire.h>                       // Included in Arduino
#include <PV_RTD_RS232_RS485_Shield.h>  // http://prods.protovoltaics.com/rtd-rs232-rs485/lib/PV_RTD_RS232_RS485_Shield.zip
#include <TToABV.h>			// https://github.com/VisionStills/TemperatureToABV
#include <PID_v1.h>
#include <TwoWayMotorisedBallValve.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

#define MAXIMUM_TEMPERATURE_SENSORS 1
#define RTD_SENSOR_WIRES 3              // Number of wires on RTD sensors - 2, 3 or 4
#define RTD_DRIVE_CURRENT 0.000250      // Set the RTD drive current to 250uA
#define RTD_PGA 32                      // A PGA value of 32 will allow measurements up to 463.5 deg C
#define RTD_SAMPLE_FREQUENCY 20

#define LIQUID 1
#define VAPOR 2

#define MOTOR_SPEED_PIN 10
#define OPEN_VALVE_PIN 12
#define CLOSE_VALVE_PIN 13
#define OPEN_LIMIT_PIN 2
#define CLOSE_LIMIT_PIN 3

#define LCD_BUFFER_SIZE 5
#define DISPLAY_DECIMALS 2

#define READ_TEMPERATURE_SENSORS_EVERY 250
#define COMPUTE_PID_EVERY 250
#define READ_PRESSURE_SENSORS_EVERY 300000
#define READ_USER_INPUT_EVERY 20
#define WRITE_DISPLAY_EVERY 200

volatile bool TwoWayMotorisedBallValve::openLimitReached;
volatile bool TwoWayMotorisedBallValve::closeLimitReached;
uint8_t TwoWayMotorisedBallValve::openLimitPin;
uint8_t TwoWayMotorisedBallValve::closeLimitPin;

// RTD Variables
int rtdChannel;       		// Channel value for RTD shield

// PID Variables
double PIDSetpoint;
double PIDInput;
double Kp = 20, Ki = 5, Kd = 1;

// Valve Variables
double ValveSetPoint = 0;

// LCD Variables
char lcd_buffer[LCD_BUFFER_SIZE];             // LCD buffer used for the better string format to LCD
bool targetChanged = true;
bool actualChanged = true;

//User Input Variables
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];// temporary array for use when parsing
// variables to hold the parsed data
char messageFromPC[numChars] = {0};
double doubleFromPC = 0.0;
boolean newData = false;

// Timing Variables
unsigned long lastTemperatureRead = 0;
unsigned long lastDisplayWrite = 0;

typedef char SensorName[36];
struct TemperatureSensor {
  int rtdChannel;       		// Channel value for RTD shield
  SensorName sensorName;			// Sensor Name
  int state;        			// Input for calculating LIQUID or VAPOR ABV
  TToABV tToABV; 			// Instance or ToToAVB.h
};

struct Settings {
  int version;
  int sensorsUsed;
  TemperatureSensor sensors[MAXIMUM_TEMPERATURE_SENSORS];
} settings = {
  // Place default values for settings here
  CONFIG_VERSION,
  1,
  { (TemperatureSensor) {
      1,
      "Column", VAPOR
    }
  }
};

//Objects
TemperatureSensor temperatureSensors[MAXIMUM_TEMPERATURE_SENSORS];
PV_RTD_RS232_RS485 rtds(0x52, 100.0);	// RTD shield with PT-100 sensors
PID myPID(&PIDInput, &ValveSetPoint, &PIDSetpoint, Kp, Ki, Kd, REVERSE);
TwoWayMotorisedBallValve valve;
LiquidCrystal_I2C	lcd(0x27, 2, 1, 0, 4, 5, 6, 7); // 0x27 is the I2C bus address for an unmodified module

void setup()
{
  Serial.begin(9600);
  // Initialise LCD Display
  init_display();
  // Initialise temperature sensors
  init_temperature_sensors();
  // Initialise Two Way Motorised Ball Valve
  valve.begin(&ValveSetPoint, OPEN_VALVE_PIN, CLOSE_VALVE_PIN, MOTOR_SPEED_PIN, OPEN_LIMIT_PIN, CLOSE_LIMIT_PIN);
  // Initialise PID
  init_PID();
  

}

void doFunctionAtInterval(void (*callBackFunction)(), unsigned long *lastEvent, unsigned long Interval) {

  unsigned long now = millis();

  if ((now - *lastEvent) >= Interval) {
    callBackFunction();
    *lastEvent = now;
  }

}

void loop()
{

  doFunctionAtInterval(read_temperature_sensors, &lastTemperatureRead, READ_TEMPERATURE_SENSORS_EVERY);
  myPID.Compute();
  valve.update();
  doFunctionAtInterval(display_actual, &lastDisplayWrite, WRITE_DISPLAY_EVERY);
  readUserInput();
}

void init_temperature_sensors() {

  for (int i = 0; i < settings.sensorsUsed; i++) {
    temperatureSensors[i].rtdChannel = settings.sensors[i].rtdChannel;
    memcpy(temperatureSensors[i].sensorName, settings.sensors[i].sensorName, sizeof(SensorName) / sizeof(char));
    temperatureSensors[i].state = settings.sensors[i].state;
  }

  // Initialise temperature sensors
  I2C_RTD_PORTNAME.begin();
  rtds.Disable_All_RTD_Channels();							// Disable all RTD channels
  rtds.Set_RTD_SPS(RTD_SAMPLE_FREQUENCY);						// Slow the shield down

  for (int i = 0; i < settings.sensorsUsed; i++) {
    // Initialise looping/repeated temperature sensor values
    rtds.Enable_RTD_Channel(RTD_SENSOR_WIRES, temperatureSensors[i].rtdChannel);                // Enable the RTD channel for each sensor
    rtds.Set_RTD_Idac(RTD_SENSOR_WIRES, temperatureSensors[i].rtdChannel, RTD_DRIVE_CURRENT);	// Set the RTD drive current for each sensor
    rtds.Set_RTD_PGA(RTD_SENSOR_WIRES, temperatureSensors[i].rtdChannel, RTD_PGA);              // Set the PGA value for each sensor


    // set tToABV object to calculate correct ABV values
    switch (temperatureSensors[i].state) {
      case VAPOR:
        temperatureSensors[i].tToABV.Vapor();
        break;
      case LIQUID:
        temperatureSensors[i].tToABV.Liquid();
        break;
      default:
        // something is wrong, set to vapor
        temperatureSensors[i].tToABV.Vapor();
        break;
    }

  }
  delay(2000);	// A short delay so that the first reading can be taken.
  read_temperature_sensors();

}

void init_PID() {

  //initialize the variables we're linked to
  PIDInput = 0;
  PIDSetpoint = 20;

  myPID.SetSampleTime(COMPUTE_PID_EVERY);
  myPID.SetOutputLimits(0, valve.getCycleTime());
  //turn the PID on
  myPID.SetMode(AUTOMATIC);

}


void init_display() {

  lcd.setBacklightPin(3, POSITIVE);
  lcd.setBacklight(HIGH); // NOTE: You can turn the backlight off by setting it to LOW instead of HIGH
  lcd.begin(16, 2);
  lcd.clear();
  display_target();
  display_actual();

}

void read_temperature_sensors() {

  double previousPIDInput = PIDInput;

  // Read temperature sensor values
  for (int i = 0; i < settings.sensorsUsed; i++) {
    temperatureSensors[i].tToABV.Temperature(rtds.Get_RTD_Temperature_degC(RTD_SENSOR_WIRES, temperatureSensors[i].rtdChannel));
  }

  //PIDInput = double(temperatureSensors[0].tToABV.ABV());
  PIDInput = double(temperatureSensors[0].tToABV.Temperature());
  if (previousPIDInput != PIDInput) {
    actualChanged = true;
  }

}

void display_target() {

  if (targetChanged) {
    lcd.setCursor(0, 0);
    dtostrf(PIDSetpoint, LCD_BUFFER_SIZE, DISPLAY_DECIMALS, lcd_buffer);
    lcd.print("TARGET:");
    lcd.print(lcd_buffer);
    targetChanged = false;
  }

}

void display_actual() {

  if (actualChanged) {
    lcd.setCursor(0, 1);
    dtostrf(PIDInput, LCD_BUFFER_SIZE, DISPLAY_DECIMALS, lcd_buffer);
    lcd.print("ACTUAL:");
    lcd.print(lcd_buffer);
    actualChanged = false;
  }

}

void readUserInput() {

  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0';// terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseUserInput();
    newData = false;
  }
}

void parseUserInput() {

  // split the data into its parts
  char * strtokIndx;	// this is used by strtok() as an index
  strtokIndx = strtok(tempChars, ",");	// get the first part - the string
  strcpy(messageFromPC, strtokIndx);	// copy it to messageFromPC
  if (String(messageFromPC) == "TARGET") {
    strtokIndx = strtok(NULL, ",");	// this continues where the previous call left off
    doubleFromPC = atof(strtokIndx);
    PIDSetpoint = double(doubleFromPC);
    Serial.print ("Changing target to "); Serial.println(doubleFromPC, 2);
    targetChanged = true;
    display_target();
  }
  else if (String(messageFromPC) == "KP") {
    strtokIndx = strtok(NULL, ",");	// this continues where the previous call left off
    doubleFromPC = atof(strtokIndx);
    myPID.SetTunings(doubleFromPC, Ki, Kd);
    Serial.print ("Changing Kp to "); Serial.println(doubleFromPC, 2);
  }
  else if (String(messageFromPC) == "KI") {
    strtokIndx = strtok(NULL, ",");	// this continues where the previous call left off
    doubleFromPC = atof(strtokIndx);
    myPID.SetTunings(Kp, doubleFromPC, Kd);
    Serial.print ("Changing Ki to "); Serial.println(doubleFromPC, 2);
  }
  else if (String(messageFromPC) == "KD") {
    strtokIndx = strtok(NULL, ",");	// this continues where the previous call left off
    doubleFromPC = atof(strtokIndx);
    myPID.SetTunings(Kp, Ki, doubleFromPC);
    Serial.print ("Changing Kd to "); Serial.println(doubleFromPC, 2);
  }
  else Serial.println ("Command not recognised.");
}
