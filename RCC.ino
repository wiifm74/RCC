#include <PID_v1.h>
#include <TwoWayMotorisedBallValve.h>

#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

#define MOTOR_SPEED_PIN 10
#define OPEN_VALVE_PIN 12
#define CLOSE_VALVE_PIN 13
#define OPEN_LIMIT_PIN 2
#define CLOSE_LIMIT_PIN 3

#define LCD_BUFFER_SIZE 5
#define DISPLAY_DECIMALS 2

volatile bool TwoWayMotorisedBallValve::openLimitReached;
volatile bool TwoWayMotorisedBallValve::closeLimitReached;
uint8_t TwoWayMotorisedBallValve::openLimitPin;
uint8_t TwoWayMotorisedBallValve::closeLimitPin;

//PID Variables
double PIDSetpoint;
double PIDInput;
double Kp = 2, Ki = 5, Kd = 1;

//Valve Variables
double ValveSetPoint;

//LCD Variables
char lcd_buffer[LCD_BUFFER_SIZE];             // LCD buffer used for the better string format to LCD

unsigned long lastDisplay = 0;

//Objects
PID myPID(&PIDInput, &ValveSetPoint, &PIDSetpoint, Kp, Ki, Kd, DIRECT);
TwoWayMotorisedBallValve valve;
LiquidCrystal_I2C	lcd(0x27, 2, 1, 0, 4, 5, 6, 7); // 0x27 is the I2C bus address for an unmodified module

void setup()
{

  //initialize the variables we're linked to
  PIDInput = 80;
  PIDSetpoint = 92;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  // Initialise Two Way Motorised Ball Valve
  valve.begin(&ValveSetPoint, OPEN_VALVE_PIN, CLOSE_VALVE_PIN, MOTOR_SPEED_PIN, OPEN_LIMIT_PIN, CLOSE_LIMIT_PIN);

  init_display();
  
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
	
  PIDInput = 80;
  myPID.Compute();
  valve.update();
  doFunctionAtInterval(display_actual, &lastDisplay, 200);
  
}


void init_display() {
	
  lcd.setBacklightPin(3, POSITIVE);
  lcd.setBacklight(HIGH); // NOTE: You can turn the backlight off by setting it to LOW instead of HIGH
  lcd.begin(16, 2);
  lcd.clear();
  display_target();
  display_actual();

}

void display_target() {

  lcd.setCursor(0, 0);
  dtostrf(PIDSetpoint, LCD_BUFFER_SIZE, DISPLAY_DECIMALS, lcd_buffer);
  lcd.print("TARGET:");
  lcd.print(lcd_buffer);

}

void display_actual() {

  lcd.setCursor(0, 1);
  dtostrf(PIDInput, LCD_BUFFER_SIZE, DISPLAY_DECIMALS, lcd_buffer);
  lcd.print("ACTUAL:");
  lcd.print(lcd_buffer);

}

