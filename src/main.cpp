#include <Arduino.h>
#include <ESP32Servo.h>
#include <MPU6050.h>

Servo esc;

volatile uint32_t pulseCount;
volatile boolean rotationCompleteFlag = false;
uint32_t rotationCount;
boolean runComplete = false;

//Speed Control Stuff
uint32_t previousRPMControllerMicros;
const uint32_t RPMControllerInterval = 100000;

uint32_t previousRotationMicros = micros();
uint32_t lastPulseMicros;
uint16_t currentRPM;
float averageTimePerPulseFromPreviousRevolution; //Using the total time required for the last revolution, divide by the number of poles to give an average duration per pole for estimating current position
const uint16_t targetRPM = 5500;

//Publishing Data Stuff
uint32_t previousDataOutputMicros;

//LED Strobe stuff
boolean strobeLEDFlag = false;
uint16_t strobeLEDDuration = 5000;
uint32_t strobeLEDPreviousMicros = 0;

//MPU Stuff
uint32_t previousMPUMicros;
uint16_t mpuInterval = 2000;

//Constants
const uint16_t dataInterval = 100;
const uint16_t zeroAngleOffset = -90;
const uint16_t numberOfPoles = 14;
const float_t degreesPerPole = 360 / numberOfPoles;
#define ESCMax 2000
#define ESCMin 1000
#define ESCRangeLimiter 1200
#define LEDPin 23
#define HallPin 18
#define ServoPin 19
#define NumberOfRotationsPerRun 1000
#define NumberOfRotationsToDiscard 100
const float_t kP = 0.25;
const float_t kI = 0.0;
const float_t kD = 0.00;
//#define Debug
//#define Verbose

//Magnitude of Accel, Angle
uint32_t runData[2][NumberOfRotationsPerRun];
int32_t maxAccelPerRotation;
int16_t maxAccelPosition;

MPU6050 mpu;
int16_t outputESC;

void calcRPM();
void IRAM_ATTR addPulse();
uint16_t getPosition();
float_t EMA_function(float alpha, float latest, float stored);
void strobeLED();
float_t processMPU();
void controlRPM();
void outputData();

//Add a pulse every time the hall sensor changes state
void IRAM_ATTR addPulse()
{
  pulseCount++;
  lastPulseMicros = micros();

  if (pulseCount % numberOfPoles == 0)
  {
    pulseCount = 0;
    rotationCompleteFlag = true;
    strobeLEDFlag = true;
  }
}

void setup()
{
  Serial.begin(500000);
  Serial.println("Started");

  Wire.setClock(400000);

  // put your setup code here, to run once:
  pinMode(LEDPin, OUTPUT);
  pinMode(HallPin, INPUT);

  attachInterrupt(HallPin, addPulse, CHANGE);

  //Initialize MPU
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  mpu.initialize();
  mpu.CalibrateAccel(6);

  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  //DLPF Setting
  mpu.setDLPFMode(MPU6050_DLPF_BW_98);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);

  esc.attach(ServoPin, 1000, 2000);

  esc.writeMicroseconds(2000);
  delay(3000);
  esc.writeMicroseconds(1000);
  delay(3000);
}

void loop()
{
  strobeLED();

  if (!runComplete)
  {
    outputData();
    if (rotationCompleteFlag)
    {
      Serial.print(rotationCount);
      Serial.print(",");
      Serial.print(maxAccelPosition);
      Serial.print(",");
      Serial.println(maxAccelPerRotation);

      runData[0][rotationCount] = maxAccelPerRotation;
      runData[1][rotationCount] = maxAccelPosition;

      maxAccelPerRotation = 0;
      maxAccelPosition = 0;

      rotationCount++;

      //Serial.println("Rotation Complete");
      //Based on when the last rotation started, store how long a rotation has taken
      averageTimePerPulseFromPreviousRevolution = (micros() - previousRotationMicros) / (float)numberOfPoles;

      calcRPM();
      controlRPM();

      if (rotationCount > NumberOfRotationsPerRun)
      {
        //Mark the run as complete after 50 rotations
        runComplete = true;
        //turn off the motor
        esc.writeMicroseconds(ESCMin);

        //Publish results
        int32_t accelerationSum = 0;
        int32_t positionSum = 0;
        for (int16_t i = NumberOfRotationsToDiscard; i < NumberOfRotationsPerRun; i++)
        {
          accelerationSum += runData[0][i];
          positionSum += runData[1][i];
        }

        Serial.println("--------------- End of Run ---------------");
        Serial.print("Avg Accel: ");
        Serial.println((float_t)accelerationSum / (NumberOfRotationsPerRun - NumberOfRotationsToDiscard));
        Serial.print("Avg Position: ");
        Serial.println((float_t)positionSum / (NumberOfRotationsPerRun - NumberOfRotationsToDiscard));
      }
      rotationCompleteFlag = false;
    }
  }
}

void outputData()
{
  uint16_t dt = micros() - previousDataOutputMicros;
  if (dt > dataInterval)
  {
    int32_t tempAccel = processMPU();
    int32_t tempPosition = getPosition();
#ifdef Verbose
    String data;
    data.concat(rotationCount);
    data.concat(",");
    data.concat(tempPosition);
    data.concat(",");
    data.concat(tempAccel);
    data.concat(",");
    data.concat(currentRPM);
    Serial.println(data);
#endif
    if (maxAccelPerRotation < tempAccel)
    {
      maxAccelPerRotation = tempAccel;
      maxAccelPosition = tempPosition;
    }
    previousDataOutputMicros = micros();
  }
}

uint16_t getPosition()
{
  uint16_t positionBasedonPulseCount;
  uint16_t positionBasedonTime;
  //2 parts, calculate the first portion based on the numbers of pulses and the degrees/pole
  // 2nd portion is the time from the last pulse to the current time to give time since last pulse
  //divide the time since the last pulse by a floating average pulse length to give the portion of the time complete for the current pulse cycle
  //multiply the portion complete of the latest pulse by the degrees per pole to give the current position
  positionBasedonPulseCount = (degreesPerPole)*pulseCount;
  positionBasedonTime = (micros() - lastPulseMicros) / averageTimePerPulseFromPreviousRevolution * degreesPerPole;

  //Constrain the time based angle of the last pole to a maximum of the angle of a single pole
  //Not really needed when the motor is running, but otherwise we get messy data when not running the motor
  positionBasedonTime = constrain(positionBasedonTime, 0, 360 / numberOfPoles);

  //Handle the angle offset to the zero position
  return (positionBasedonPulseCount + positionBasedonTime + zeroAngleOffset) % 360;
}

void calcRPM()
{
  uint32_t dt = micros() - previousRotationMicros;

  //Time for a single rotation divided into 60 to get RPM
  uint16_t tempRPM = 60 / (dt / 1000000.00);

  currentRPM = EMA_function(0.4, tempRPM, currentRPM);

  previousRotationMicros = micros();
}

void controlRPM()
{
  static int32_t cumulativeError;
  static int32_t previousError;
  float rateError;
  int32_t error;

  uint32_t dt = micros() - previousRPMControllerMicros;

  if (dt > RPMControllerInterval)
  {

    error = targetRPM - currentRPM;

#ifdef DEBUG
    Serial.print("Error: ");
    Serial.println(error);
#endif

    rateError = (float)(error - previousError) / dt;

    cumulativeError += error * dt;

    outputESC = kP * error + kI * cumulativeError + kD * rateError;

#ifdef DEBUG
    Serial.print("Raw Output: ");
    Serial.println(outputESC);
#endif

    //Map the PID output which can be negative from 0 to the ESC min to the range limiter which is our control authority range (1000-~1300us)
    outputESC = map(outputESC, -5000, 5000, 0, ESCRangeLimiter - ESCMin);

    //Add in the offset from 0 to 1000 which is min throttle
    outputESC += ESCMin;

    //Make sure the ESC doesn't recieve a signal it can't use and cap the upper limit
    outputESC = constrain(outputESC, ESCMin, ESCRangeLimiter);
#ifdef DEBUG
    Serial.print("Mapped Output: ");
    Serial.println(outputESC);

    Serial.print("Current RPM: ");
    Serial.println(currentRPM);
#endif

    esc.writeMicroseconds(outputESC);

    previousError = error;

    previousRPMControllerMicros = micros();
  }
}

void strobeLED()
{
  static boolean LEDOn;

  if (strobeLEDFlag)
  {
    // Serial.println("Begin Strobe");
    //Turn on Strobe LED
    digitalWrite(LEDPin, HIGH);
    //Start timer for strobe duration
    strobeLEDPreviousMicros = micros();
    LEDOn = true;
    strobeLEDFlag = false;
  }

  uint32_t dt = micros() - strobeLEDPreviousMicros;
  if (dt > strobeLEDDuration && LEDOn)
  {
    // Serial.println("End Strobe");
    //Turn off the strobe LED when the duration is complete and reset the strobe flag
    LEDOn = false;
    digitalWrite(LEDPin, LOW);
  }
}

float_t processMPU()
{
  float_t y_accel;
  y_accel = mpu.getAccelerationY();

#ifdef Debug
  Serial.println(y_accel);
#endif

  return y_accel;
}

float EMA_function(float alpha, float latest, float stored)
{
  return (alpha * latest) + ((1 - alpha) * stored);
}