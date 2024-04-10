#include <Arduino.h>
#include <NewPing.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <cmath>
#include "NostromoPinsAndGlobals.h"
#include "SGUltrasonic.h"

// Right motor side
#define RSmotorPin1 32 // Riv: IN1_PIN
#define RSmotorPin2 33 // Riv: IN2_PIN
#define RSenablePin 35 // Riv: ENA_PIN

// Left motor side
#define LSmotorPin1 26 // Riv: IN3_PIN
#define LSmotorPin2 25 // Riv: IN4_PIN
#define LSenablePin 34 // Riv: ENB_PIN

// Setting PWM properties
const int freq = 30000;
const int RSpwmChannel = 0;
const int LSpwmChannel = 0;
const int resolution = 8;
const int run_time = 2000; // Riv: Duration
int Lspeed = 255;          // Riv: straight_A
int Rspeed = 255;          // Riv: straight_B

// Accelerometer (IMU) LIS3DH variables
Adafruit_LIS3DH lis = Adafruit_LIS3DH();
int counter = 0;
float moving_x1 = 0;
float moving_x2 = 0;
float moving_y1 = 0;
float moving_y2 = 0;
const float xmoved = .2;
const float ymoved = .2;
float heading = 0;

// compass variables (this is based on location)
Adafruit_LIS3MDL lis3mdl;
const float declinationAngle = 7.6666666667; // Westminster
//  const float declinationAngle = 7.25;              //Tinkermill and Sand Dunes
//  const float declinationAngle = 7.566666667;       //Brighton
//  const float declinationAngle = 7.283333333;       //Strasburg

// other globals
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;
float ultimate_direction = 0;

// put function declarations here:
void Forward();
void Reverse(int Duration);
void Stop();
void RotateRight(int Duration);
void RotateLeft(int Duration);
void IMU_Setup();
bool are_we_moving();
void Compass_Setup();
void cardinalDir(float headingDegrees);
float facing_direction();
void get_input();
void show_input();
void do_turn(char receivedChars[numChars]);
int round_to_15(float temp);

void setup() // put your setup code here, to run once:
{
  // start serial communication for debugging/testing
  Serial.begin(9600);

  // MOTOR STUFF
  //  set pins as outputs
  pinMode(RSmotorPin1, OUTPUT);
  pinMode(RSmotorPin2, OUTPUT);
  pinMode(RSenablePin, OUTPUT);
  pinMode(LSmotorPin1, OUTPUT);
  pinMode(LSmotorPin2, OUTPUT);
  pinMode(LSenablePin, OUTPUT);

  // configure LED PWM functionalities
  ledcSetup(RSpwmChannel, freq, resolution);
  ledcSetup(LSpwmChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(RSenablePin, RSpwmChannel);
  ledcAttachPin(LSenablePin, LSpwmChannel);

  // Accelerometer (IMU) LIS3DH stuff
  delay(5000);
  IMU_Setup();
  Compass_Setup();
  // for Testing
  // Serial.println("Testing DC Motor...");
}

//
//
//

void loop()
{
  // Serial.println("in loop");
  // delay(2000);
  // counter = counter +1;

  unsigned int stop_or_go = is_to_close();
  if (stop_or_go == 1)
  {
    Stop();
    Serial.print("Turn in degrees, left is negative, right positive: ");
    get_input();
    show_input();
    delay(100);
    do_turn(receivedChars);
    //  are_we_moving();
    // delay(100);        //only for testing
  }
  else
  {
    Serial.println("Go");
    Forward();
    // are_we_moving();
    //  delay(100);        //only for testing
  }
  // Serial.println("loop end");
  Serial.println("");
}

//
//
//

// put function definitions here:

// Motor Functions
void Forward()
{
  // Function to drive forward for amount of time Duration
  // Serial.print("Forward function ");
  // Side A spins clockwise
  digitalWrite(RSmotorPin1, LOW);
  digitalWrite(RSmotorPin2, HIGH);
  // Side B spins clockwise
  digitalWrite(LSmotorPin1, LOW);
  digitalWrite(LSmotorPin2, HIGH);
  heading = facing_direction();
  // pwm yippee (S: not needed because of the way I setup)
  // analogWrite(RSenablePin, pwm_R);
  // analogWrite(LSenablePin, pwm_L);
}

//
//
//

void Reverse(int Duration)
{
  // Function to drive reverse for amount of time Duration

  Serial.print("Reverse for ");
  Serial.print(Duration / 1000);
  Serial.println(" seconds");
  // Side A spins counterclockwise
  digitalWrite(RSmotorPin1, HIGH);
  digitalWrite(RSmotorPin2, LOW);
  // Side B spins counterclockwise
  digitalWrite(LSmotorPin1, HIGH);
  digitalWrite(LSmotorPin2, LOW);
  // delay(Duration);
}

//
//
//

void Stop()
{
  Serial.println("Stop function");
  // Side A spins clockwise
  digitalWrite(RSmotorPin1, LOW);
  digitalWrite(RSmotorPin2, LOW);
  // Side B spins clockwise
  digitalWrite(LSmotorPin1, LOW);
  digitalWrite(LSmotorPin2, LOW);
}

//
//
//

void RotateRight() // int Duration)
{
  // Function to drive right rotation for amount of time Duration
  Serial.print("Rotating right.   ");
  // Serial.print(Duration / 1000);
  // Serial.println(" seconds");
  //  Side A spins counterclockwise
  digitalWrite(RSmotorPin1, HIGH);
  digitalWrite(RSmotorPin2, LOW);
  // Side B spins clockwise
  digitalWrite(LSmotorPin1, LOW);
  digitalWrite(LSmotorPin2, HIGH);
  // delay(Duration);
}

//
//
//

void RotateLeft() // int Duration)
{
  Serial.print("Rotating Left.   ");
  // Serial.print(Duration / 1000);
  // Serial.println(" seconds");
  // Side A spins clockwise
  digitalWrite(RSmotorPin1, LOW);
  digitalWrite(RSmotorPin2, HIGH);
  // Side B spins counterclockwise
  digitalWrite(LSmotorPin1, HIGH);
  digitalWrite(LSmotorPin2, LOW);
  // delay(Duration);
}

//
//
// Accelerometer (IMU) LIS3DH functions

void IMU_Setup()
{
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  // Serial.println("LIS3DH test!");

  if (!lis.begin(0x18)) //{  change this to 0x19 for alternative i2c address
  {
    Serial.println("Couldnt start");
    while (1)
      yield();
  }
  Serial.println("LIS3DH found!");

  lis.setRange(LIS3DH_RANGE_2_G); // 2, 4, 8 or 16 G!

  Serial.print("Range = ");
  Serial.print(2 << lis.getRange());
  Serial.println("G");

  lis.setDataRate(LIS3DH_DATARATE_50_HZ);
}

//
// just a note  true=1 false=0
//

bool are_we_moving()
{
  /* Or....get a new sensor event, normalized */
  sensors_event_t event;
  lis.getEvent(&event);

  // Display the results (acceleration is measured in m/s^2)

  moving_x1 = event.acceleration.x;
  // Serial.print("\t\tX: ");
  // Serial.print(event.acceleration.x);
  moving_y1 = event.acceleration.y;
  // Serial.print(" \tY: ");
  // Serial.print(event.acceleration.y);

  // Serial.print(" \tZ: ");
  // Serial.print(event.acceleration.z);
  // Serial.println(" m/s^2 ");

  // delay(200);
  lis.getEvent(&event);

  moving_x2 = event.acceleration.x;
  // Serial.print("\t\tX: ");
  // Serial.print(event.acceleration.x);
  moving_y2 = event.acceleration.y;
  // Serial.print(" \tY: ");
  // Serial.print(event.acceleration.y);

  if (abs(moving_x2 - moving_x1) > xmoved) // or (moving_x2 - moving_x1 < -xmoved))
  {
    Serial.println("moving in X");
    return true;
  }
  else if (abs(moving_y2 - moving_y1) > ymoved) // or (moving_y2 - moving_y1 < -ymoved))
  {
    Serial.println("moving in Y");
    return true;
  }
  else
  {
    Serial.println("NOT MOVING");
    return false;
  }
}

//
//
//

void Compass_Setup()
{
  // Serial.println("Adafruit LIS3MDL test!");

  // Try to initialize!
  if (!lis3mdl.begin_I2C())
  {
    Serial.println("Failed to find LIS3MDL chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("LIS3MDL Found!");

  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true,               // polarity
                          false,              // don't latch
                          true);              // enabled!
}

//
//
//

void cardinalDir(float headingDegrees)
{
  String cardinal;
  if (headingDegrees > -11.25 && headingDegrees < 11.25)
  {
    cardinal = " N";
  }
  else if (headingDegrees > 11.25 && headingDegrees < 33.75)
  {
    cardinal = " NNE";
  }
  else if (headingDegrees > 33.75 && headingDegrees < 56.25)
  {
    cardinal = " NE";
  }
  else if (headingDegrees > 56.25 && headingDegrees < 78.75)
  {
    cardinal = " ENE";
  }
  else if (headingDegrees > 78.75 && headingDegrees < 101.25)
  {
    cardinal = " E";
  }
  else if (headingDegrees > 101.25 && headingDegrees < 123.75)
  {
    cardinal = " ESE";
  }
  else if (headingDegrees > 123.75 && headingDegrees < 146.25)
  {
    cardinal = " SE";
  }
  else if (headingDegrees > 146.25 && headingDegrees < 168.75)
  {
    cardinal = " SSE";
  }
  else if (headingDegrees > 168.75 && headingDegrees < -168.75)
  {
    cardinal = " S";
  }
  else if (headingDegrees > -168.75 && headingDegrees < -146.25)
  {
    cardinal = " SSW";
  }
  else if (headingDegrees > -146.25 && headingDegrees < -123.75)
  {
    cardinal = " SW";
  }
  else if (headingDegrees > -123.75 && headingDegrees < -101.25)
  {
    cardinal = " WSW";
  }
  else if (headingDegrees > -101.25 && headingDegrees < -78.75)
  {
    cardinal = " W";
  }
  else if (headingDegrees > -78.75 && headingDegrees < -56.25)
  {
    cardinal = " WNW";
  }
  else if (headingDegrees > -56.25 && headingDegrees < -33.75)
  {
    cardinal = " NW";
  }
  else if (headingDegrees > -33.75 && headingDegrees < -11.25)
  {
    cardinal = " NNW";
  }
  Serial.println(cardinal);
}

//
//
//

float facing_direction()
{
  // Serial.println("facing dir function");
  /* Or....get a new sensor event, normalized to uTesla */
  sensors_event_t event;
  lis3mdl.getEvent(&event);

  float headingRadians = atan2(event.magnetic.y, event.magnetic.x);
  float headingDeg = headingRadians * 180 / PI;

  headingDeg += declinationAngle;

  // if (headingDeg < 0)
  //{
  // headingDeg += 360;
  //}

  Serial.print("Current HeadingDeg: ");
  Serial.print(headingDeg);
  cardinalDir(headingDeg);

  // delay(2000);
  return headingDeg;
}

//
//
//

void get_input()
{
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() < 3)
  {
    delay(1000);
    // Serial.println("waiting for data");
  }
  delay(1000);
  while (Serial.available() > 0 && newData == false)
  {
    rc = Serial.read();
    /*
    Serial.println();
    Serial.print("numChars: ");
    Serial.print(numChars);
    Serial.print(" rc: ");
    Serial.print(rc);
    Serial.print(" ndx: ");
    Serial.print(ndx);
    Serial.print(" newData: ");
    Serial.print(newData);
    */
    if (rc != endMarker)
    {
      receivedChars[ndx] = rc;
      Serial.print(" receivedChars[ndx]: ");
      Serial.print(receivedChars[ndx]);
      ndx++;
      Serial.print(" ndx: ");
      Serial.println(ndx);
      if (ndx >= numChars)
      {
        ndx = numChars - 1;
      }
    }
    else
    {
      // Serial.println();
      // Serial.println("In the else section, because end char");
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

//
//
//

void show_input()
{
  if (newData == true)
  {
    Serial.println(receivedChars);
    newData = false;
  }
}

//
//
// left is negative, right is positive.

void do_turn(char receivedChars[])
{
  int val = atoi(receivedChars);
  if (val > 90 || val < -90)
  {
    Serial.println("Outside allowable range.");
    return;
  }
  val = round_to_15(val);
  Serial.print("do_turn val (rounded to nearest 15): ");
  Serial.println(val);
  heading = facing_direction();
  int new_heading = heading + val;

  if (new_heading > 180)
  {
    new_heading = new_heading - 360; // converts to neg ver of number (which is what the sensor is using for 180-360)
  }
  Serial.print("New heading is: ");
  Serial.println(new_heading);
  if (val < 0)
  {
    do
    {
      RotateLeft();
      heading = facing_direction();
    } while (heading > new_heading);
  }
  else
  {
    do
    {
      RotateRight();
      heading = facing_direction();
    } while (heading < new_heading);
  }
}

//
//
//

int round_to_15(float temp)
{
  int num_to_fix = round(temp);
  int multiple = 15;

  int remainder = abs(num_to_fix) % multiple;
  if (remainder == 0)
    return num_to_fix;

  if (num_to_fix < 0)
    return -(abs(num_to_fix) - remainder);
  else
    return num_to_fix + multiple - remainder;
}
