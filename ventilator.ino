#include <Servo.h>
Servo BELLOWS_SERVO;

// display LEDs
#define POWER_LED 2       // LED to track when the system is powered ON/OFF
#define ALARM_MOD 3       // this module includes an ALARM LED and associated audio buzzer

// user and sensor inputs
#define START           A5 // push button to start the ventilator
#define RR_POT          A4 // respiration rate control
#define TV_POT          A3 // tidal volume control
#define PRESSURE_SENSOR A1 // airway pressure sensor
#define FLOW_SENSOR     A2 // airway flow rate sensor (only tracks inhalation)

// constants
const int IE_RATIO            = 4;
const int MAX_TV              = 800;  // maximum volume of air that AMBU bag can deliver in mL
const int MAX_RR              = 30;   // maximum respiration rate that the system should support in breaths per min
const int MIN_RR              = 12;   // maximum respiration rate that the system should support in breaths per min
const int MAX_AIRWAY_PRESSURE = 3922; // maximum allowable pressure in Pa (40 cmH2o) at any time
const int AIRWAY_PRESSURE     = 1960; // maintainance pressure in Pa (20 cmH2o) during inhalation phase
const int PEEP                = 980;  // minimum maintainance pressure in Pa (10 cmH2o) at all times
const int TRIGGER_THRESHOLD   = 490;  // assumes pressure under 5 cmH2o is due to patient triggering a breath cycle
const int MOTOR_CURRENT       = 5;    // in amps
const int DRIVER_VOLTAGE      = 12;
const int MAX_BAG_VOLUME      = 800;  // AMBU bag can deliver a maximum of 800 mL per squeeze
const int SCALE               = 1;
const int MAX_SERVO_ROM       = 180;  // The servo range of motion is limited to 0-180 degrees
const int SERVO_ZERO_STATE   = -90;   // position signal zeroing the servo
const float MAX_FLOW_RATE     = 2.7;  // in liters
const float HOLDING_PERIOD    = 100;  // hold time at peak inhalation pressure (PIP) in mSec
const float PEEP_EXHALE_RATIO = 0.3;  // 30% of exhalation period is spent at PEEP

// actuation and display
#define SERVO_PIN 13

// global variables
int inputTidalVol, inputRespRate;
int currentPosition, currentPressure;
bool stopped;
float period, inhalePeriod, exhalePeriod, peepPeriod;

void setup()
{
  Serial.begin(9600);
  pinMode(POWER_LED, OUTPUT);
  pinMode(ALARM_MOD, OUTPUT);
  pinMode(START, INPUT);
  pinMode(TV_POT, INPUT);
  pinMode(RR_POT, INPUT);
  pinMode(PRESSURE_SENSOR, INPUT);
  pinMode(FLOW_SENSOR, INPUT);

  digitalWrite(POWER_LED, HIGH);

  BELLOWS_SERVO.attach(SERVO_PIN);

  sendPWM(SERVO_ZERO_STATE, 0);
  stopped = true;
}

void loop() {
  readInputsAndStart();
  performBreath();

  delay(1000);
}

void performBreath()
{
  inhale();
  holdPip();
  exhale();
}

void readInputsAndStart()
{
  int startButtonState = digitalRead(START);

  if (digitalRead(START) == 1 || stopped == false)
  {
    stopped = false;
    inputTidalVol = mapToTV(analogRead(TV_POT));
    inputRespRate = mapToRR(analogRead(RR_POT));
    period       = (60 / inputRespRate) * 1000;
    inhalePeriod = (period / (1 + IE_RATIO)) - HOLDING_PERIOD;
    exhalePeriod = period - (inhalePeriod + HOLDING_PERIOD);
    peepPeriod   = PEEP_EXHALE_RATIO * exhalePeriod;
  }
}

void inhale()
{
  if(stopped == true) {return;}

  currentPosition = 0;
  int finalPosition = mapVolumeToServoPosition(inputTidalVol);
  float stepDelay = calculateStepDelay(inhalePeriod, abs(finalPosition - currentPosition));

  sendPWM(finalPosition, stepDelay);
}

void holdPip()
{
  if(stopped == true) {return;}
  delay(HOLDING_PERIOD);
}

void exhale()
{
  if(stopped == true) {return;}
  sendPWM(SERVO_ZERO_STATE, 0);

  currentPressure = mapToPressure(analogRead(PRESSURE_SENSOR));
  if (currentPressure < TRIGGER_THRESHOLD)
  {
    performBreath();
  }
}

// low level methods

int mapToTV(int tvPot)
{
  return map(tvPot, 0, 1023, 0, MAX_TV);
}

int mapToRR(int rrPot)
{
  return map(rrPot, 0, 1023, MIN_RR, MAX_RR);
}

int mapVolumeToServoPosition(int volume)
{
  return map(volume, 0, MAX_TV, 0, MAX_SERVO_ROM);
}

float mapToPressure(int pressureSensorVoltage)
{
  return map(pressureSensorVoltage, 0, 1023, 0, MAX_AIRWAY_PRESSURE);
}

float calculateStepDelay(float period, int angleDisplacement)
{
  return (10 * (period/angleDisplacement));
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
