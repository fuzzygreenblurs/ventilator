#include <Servo.h>
#include <LiquidCrystal.h>
Servo BELLOWS_SERVO;

// display LEDs
#define POWER_LED 2 // LED to track when the system is powered ON/OFF
#define ALARM_MOD 3 // this module includes an ALARM LED and associated audio buzzer

// user and sensor inputs
#define START 0           // push button to start the ventilator
#define RR_POT 4          // respiration rate control
#define TV_POT 3          // tidal volume control
#define PRESSURE_SENSOR 1 // airway pressure sensor
#define FLOW_SENSOR 2     // airway flow rate sensor (only tracks inhalation)

// actuation and display
#define SERVO_PIN 13

// global variables
int inputTidalVol;
int inputRespRate;

float period;
float inhalePeriod;
float exhalePeriod;
float peepPeriod;

bool ongoing;
int currentPosition;
float currentPressure;

const int IE_RATIO = 4;
const int MAX_TV = 800;               // maximum volume of air that AMBU bag can deliver in mL
const int MAX_RR = 30;                // maximum respiration rate that the system should support in breaths per min
const int MAX_AIRWAY_PRESSURE = 3922; // maximum allowable pressure in Pa (40 cmH2o) at any time
const int AIRWAY_PRESSURE = 1960;     // maintainance pressure in Pa (20 cmH2o) during inhalation phase
const int PEEP = 980;                 // minimum maintainance pressure in Pa (10 cmH2o) at all times
const int TRIGGER_THRESHOLD = 490;    // assumes pressure under 5 cmH2o is due to patient triggering a breath cycle
const int MOTOR_CURRENT = 5;          // in amps
const int DRIVER_VOLTAGE = 12;
const int MAX_BAG_VOLUME = 800; // AMBU bag can deliver a maximum of 800 mL per squeeze
const int SCALE = 1;
const int MAX_SERVO_ROM = 180;       // The servo range of motion is limited to 0-180 degrees
const float MAX_FLOW_RATE = 2.7;     // in liters
const float HOLDING_PERIOD = 100;    // hold time at peak inhalation pressure (PIP) in mSec
const float PEEP_EXHALE_RATIO = 0.3; // 30% of exhalation period is spent at PEEP

void setup()
{
  Serial.begin(9600);
  pinMode(POWER_LED, OUTPUT);
  pinMode(ALARM_MOD, OUTPUT);
  digitalWrite(POWER_LED, HIGH);
  lcd.begin(20, 3);  // Begin LCD with 20 columns and 3 rows
  BELLOWS_SERVO.attach(SERVO_PIN);

  ongoing = false;
}

void loop() {
  readInputsAndStart();
  performBreath();
}

void performBreath()
{
  inhale();
  holdPip();
  exhale();
}

void readInputsAndStart()
{
  if (digitalRead(START) == 1 || ongoing == true)
  {
    ongoing = true;
    inputTidalVol = mapToTV(analogRead(TV_POT));
    inputRespRate = mapToRR(analogRead(RR_POT));
  }
  lcd.clear();
  lcd.print("Ventilator ON")
  delay(1000);

  lcd.clear();
  lcd.print("Tidal Volume=");
  lcd.setCursor(18,0);
  lcd.print(TV);
  lcd.setCursor(0, 1);
  lcd.print("Respiratory Rate=");
  lcd.setCursor(18,1);
  lcd.print(RR);
  lcd.setCursor(0, 2);
  lcd.print("I/E ratio= 4");
  delay(3000);

  period = 60 / inputRespRate;
  inhalePeriod = (period / (1 + IE_RATIO)) - HOLDING_PERIOD;
  exhalePeriod = period - (inhalePeriod + HOLDING_PERIOD);
  peepPeriod = PEEP_EXHALE_RATIO * exhalePeriod;
}

void inhale()
{
  currentPosition = 0;
  int finalPosition = mapVolumeToServoPosition(inputTidalVol);
  int stepDelay = calculateStepDelay(inhalePeriod, abs(finalPosition - currentPosition));

  sendPWM(finalPosition, stepDelay);
}

void holdPip()
{
  delay(HOLDING_PERIOD);
}

void exhale()
{
  int finalPosition = 0;
  sendPWM(finalPosition, 0);

  if (currentPressure < TRIGGER_THRESHOLD)
  {
    
    lcd.clear();
    lcd.print("Patient Triggered ");
    performBreath();
  }
}

void exceedMAX()
{
  lcd.clear();
  lcd.print("WARNING");
  lcd.setCursor(0, 1);
  lcd.print("Exceeded MAX Pressure");
  
  BELLOWS_SERVO.write(0);   //HALT Motor or make it return to initial position to decompress the bag
  delay(2000);
  lcd.clear();
  lcd.print("Please RESET");
}

// low level methods

int mapToTV(int tvPot)
{
  return map(tvPot, 0, 1023, 0, MAX_TV);
}

int mapToRR(int rrPot)
{
  return map(rrPot, 0, 1023, 0, MAX_RR);
}

int mapVolumeToServoPosition(int volume)
{
  return map(volume, 0, MAX_TV, 0, MAX_SERVO_ROM);
}

int mapToPressure(int pressureSensorVoltage)
{
  return map(pressureSensorVoltage, 0, 1023, 0, MAX_AIRWAY_PRESSURE);
}

float calculateStepDelay(float period, int angleDisplacement)
{
  return (angleDisplacement / period) * 1000
}

void sendPWM(int finalPosition, float stepDelay)
{
  bool goingForward = currentPosition < finalPosition;

  if (goingForward == true)
  {
    while (currentPosition != finalPosition)
    {
      BELLOWS_SERVO.write(currentPosition);
      delay(stepDelay);

      currentPosition++;
    }
  }
  else
  {
    while (currentPosition != finalPosition)
    {
      BELLOWS_SERVO.write(currentPosition);
      delay(stepDelay);

      currentPosition--;
    }
  }
}

/*
lowest  bpm -> 12 bpm -> 1sec    : 4 sec
highest bpm -> 24 bpm -> 0.5 sec : 2 sec
*/
