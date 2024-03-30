#include <Arduino.h>
#include <NewPing.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
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

// put function declarations here:
void Forward();
void Reverse(int Duration);
void Stop();
void RotateRight(int Duration);
void RotateLeft(int Duration);
void IMU_Setup();
bool are_we_moving();

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
  delay(10000);
  IMU_Setup();

  // for Testing
  // Serial.println("Testing DC Motor...");
}

//
//
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
    Serial.println("STOP");
    Serial.println("Motor stopped");
    Stop();
    are_we_moving();
    // delay(100);        //only for testing
  }
  else
  {
    Serial.println("Go");
    Serial.println("Moving Forward");
    Forward();
    are_we_moving();
    // delay(100);        //only for testing
  }
  Serial.println("loop end");
  Serial.println("");
}

//
//
//
//
//

// put function definitions here:

// Motor Functions
void Forward()
{
  // Function to drive forward for amount of time Duration
  Serial.print("Forward function ");
  // Side A spins clockwise
  digitalWrite(RSmotorPin1, LOW);
  digitalWrite(RSmotorPin2, HIGH);
  // Side B spins clockwise
  digitalWrite(LSmotorPin1, LOW);
  digitalWrite(LSmotorPin2, HIGH);
  // pwm yippee (S: not needed because of the way I setup)
  // analogWrite(RSenablePin, pwm_R);
  // analogWrite(LSenablePin, pwm_L);
}

//
//
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
  delay(Duration);
}

//
//
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
//
//

void RotateRight(int Duration)
{
  // Function to drive right rotation for amount of time Duration
  Serial.print("RotateRight for ");
  Serial.print(Duration / 1000);
  Serial.println(" seconds");
  // Side A spins counterclockwise
  digitalWrite(RSmotorPin1, HIGH);
  digitalWrite(RSmotorPin2, LOW);
  // Side B spins clockwise
  digitalWrite(LSmotorPin1, LOW);
  digitalWrite(LSmotorPin2, HIGH);
  delay(Duration);
}

//
//
//
//
//

void RotateLeft(int Duration)
{
  // Function to drive left rotation for amount of time Duration

  Serial.print("RotateLeft for ");
  Serial.print(Duration / 1000);
  Serial.println(" seconds");
  // Side A spins clockwise
  digitalWrite(RSmotorPin1, LOW);
  digitalWrite(RSmotorPin2, HIGH);
  // Side B spins counterclockwise
  digitalWrite(LSmotorPin1, HIGH);
  digitalWrite(LSmotorPin2, LOW);
  delay(Duration);
}

//
//
//
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

// just a note  true=1 false=0
bool are_we_moving()
{
  /* Or....get a new sensor event, normalized */
  sensors_event_t event;
  lis.getEvent(&event);

  // Display the results (acceleration is measured in m/s^2)
  moving_x1 = event.acceleration.x;
  Serial.print("\t\tX: ");
  Serial.print(event.acceleration.x);
  moving_y1 = event.acceleration.y;
  Serial.print(" \tY: ");
  Serial.print(event.acceleration.y);
  // Serial.print(" \tZ: ");
  // Serial.print(event.acceleration.z);
  // Serial.println(" m/s^2 ");

  delay(200);
  lis.getEvent(&event);

  moving_x2 = event.acceleration.x;
  Serial.print("\t\tX: ");
  Serial.print(event.acceleration.x);
  moving_y2 = event.acceleration.y;
  Serial.print(" \tY: ");
  Serial.print(event.acceleration.y);

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