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
#include <vector>
#include <ESP32Servo.h>

using namespace std;

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
int Lspeed = 100;          // Riv: straight_A
int Rspeed = 100;          // Riv: straight_B

// Accelerometer (IMU) LIS3DH variables
Adafruit_LIS3DH lis = Adafruit_LIS3DH();
int counter = 0;
float moving_x1 = 0;
float moving_x2 = 0;
float moving_y1 = 0;
float moving_y2 = 0;
const float xmoved = .1;
const float ymoved = .1;
float heading = 0;

// compass variables (this is based on location)
Adafruit_LIS3MDL lis3mdl;
const float declinationAngle = 7.6666666667; // Westminster
//  const float declinationAngle = 7.25;              //Tinkermill and Sand Dunes
//  const float declinationAngle = 7.566666667;       //Brighton
//  const float declinationAngle = 7.283333333;       //Strasburg

// other globals
const byte numChars = 32;
// char receivedChars[numChars];
int receivedChars = 0;
boolean newData = false;
float ultimate_direction = 0;

// Data Structures
SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 bytes of RAM, Used to store distances
Servo rotationalServo;                // Creates servo objects
Servo pitchServo;
vector<int> detectedObjectVector = {};
vector<int> correspondingDistanceVector = {};

// ToF Servo Assembly Variables
int rotServoPin = 4;
int pitchServoPin = 5;

// Universal Variables
int imageResolution = 0; // Used to pretty print output
int imageWidth = 0;      // Used to pretty print output
int minDistance = 150;   // Set nearest distance an object can get to the sensor (mm)
int height1 = 128;       // Know height of the ToF sensor off the ground on a flat plane (mm)
int zone1Locations[24] = {0, 1, 2, 8, 9, 10, 16, 17, 18, 24, 25, 26, 32, 33, 34, 40, 41, 42, 48, 49, 50, 56, 57, 58};
int zone2Locations[16] = {3, 4, 11, 12, 19, 20, 27, 28, 35, 36, 43, 44, 51, 52, 59, 60};
int zone3Locations[24] = {5, 6, 7, 13, 14, 15, 21, 22, 23, 29, 30, 31, 37, 38, 39, 45, 46, 47, 53, 54, 55, 61, 62, 63};
int zone1Values[24] = {};
int zone2Values[16] = {};
int zone3Values[24] = {};
int which_way = 0;

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
// void do_turn(char receivedChars[numChars]);
void do_turn(int receivedChars);
int round_to_15(float temp);
void remindTheSensorItExists();
void setup_servos();
void servo_sweep();
void sensorIntitiation();
bool dataProcessed();
bool objectDetection();
int heightCalculator(int smol);
int getDistance();
int objectAvoidance();
void get_input();
void get_turn_info();

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
  setup_servos();
  sensorIntitiation();
  servo_sweep();
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
    // Serial.print("Turn in degrees, left is negative, right positive: ");
    // get_input();
    // show_input();
    get_turn_info();
    delay(100);
    do_turn(receivedChars);
    //  are_we_moving();
    // delay(100);        //only for testing
  }
  else
  {
    Serial.println("Go");
    Forward();
    are_we_moving();
    //  delay(100);        //only for testing
  }
  // Serial.println("loop end");
  //Serial.println("");
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

   delay(200);
  lis.getEvent(&event);

  moving_x2 = event.acceleration.x;
  // Serial.print("\t\tX: ");
  // Serial.print(event.acceleration.x);
  moving_y2 = event.acceleration.y;
  // Serial.print(" \tY: ");
  // Serial.print(event.acceleration.y);

  if (abs(moving_x2 - moving_x1) > xmoved) // or (moving_x2 - moving_x1 < -xmoved))
  {
    Serial.print("moving in X: ");
    //Serial.print(moving_x2);
    //Serial.print(" - ");
    //Serial.print(moving_x1);
    //Serial.print(" = ");
    //Serial.println(moving_x2 - moving_x1);
    return true;
  }
  else if (abs(moving_y2 - moving_y1) > ymoved) // or (moving_y2 - moving_y1 < -ymoved))
  {
    Serial.print("moving in Y");
    //Serial.print(moving_y2);
    //Serial.print(" - ");
    //Serial.print(moving_y1);
    //Serial.print(" = ");
    //Serial.println(moving_y2 - moving_y1);
    return true;
  }
  else
  {
    Serial.println("NOT MOVING");
    Stop();
    Reverse(1000);
    do_turn(receivedChars);
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
 /* static byte ndx = 0;
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
    /*
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
  */
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

// void do_turn(char receivedChars[])
void do_turn(int receivedChars)
{
  //int val = atoi(receivedChars);
  int val = receivedChars;
  if (val > 90 || val < -90)
  {
    Serial.println("Outside allowable range.");
    return;
  }
  //val = round_to_15(val);
  //Serial.print("do_turn val (rounded to nearest 15): ");
  //Serial.println(val);
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
      delay(200);
    } while (heading > new_heading);
  }
  else if (val == 0)
  {
    Forward();
    heading = facing_direction();
  }
  else
  {
    do
    {
      RotateRight();
      heading = facing_direction();
      delay(200);
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

//
//
// Flip the sensor upwards to see a close neutral surface to help reset the zone values before taking data

void remindTheSensorItExists()
{
  delay(500);

  // Cycle Servo up and dwn 3 times
  for (int i = 0; i <= 3; ++i)
  {
    pitchServo.write(5);
    delay(1000);
    pitchServo.write(90);
    delay(1000);
  }
}

//
//
//

void setup_servos()
{
  // Sets up servo objects to be used in the rest of the code
  // rotationalServo.setPeriodHertz(50);
  // pitchServo.setPeriodHertz(50);

  rotationalServo.attach(rotServoPin); //, 500, 2400);
  pitchServo.attach(pitchServoPin);    //, 500, 2400);

  // rotationalServo.write(90);
  //  pitchServo.write(90);

  // Serial.println("Set Servos to 90");
}

//
//
//

void servo_sweep()
{
  rotationalServo.write(45);
  delay(200);
  rotationalServo.write(60);
  delay(200);
  rotationalServo.write(75);
  delay(200);
  rotationalServo.write(90);
  delay(200);
  rotationalServo.write(105);
  delay(200);
  rotationalServo.write(120);
  delay(200);
  pitchServo.write(10);
  delay(200);
  pitchServo.write(45);
  delay(200);
  pitchServo.write(60);
  delay(200);
  pitchServo.write(75);
  delay(200);
  pitchServo.write(90);
  delay(200);
  rotationalServo.write(90);
  delay(200);
}

//
//
// Function to intitiat ToF Sensor Settings so I can Clean my code a bit and make it easier to copy and paste
void sensorIntitiation()
{
  delay(1000);
  Serial.println("SparkFun VL53L5CX Imager Example");

  Wire.begin();          // This resets to 100kHz I2C
  Wire.setClock(400000); // Sensor has max I2C freq of 400kHz

  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  if (myImager.begin() == false)
  {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1)
      ;
  }

  myImager.setResolution(8 * 8); // Enable all 64 pads

  imageResolution = myImager.getResolution(); // Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution);         // Calculate printing width

  // The sensor starts collecting data
  myImager.startRanging();
}

//
//
// checks to see if data can be collected from sensor and if true reads the data into an array
bool dataProcessed()
{
  Serial.println("Is data processed?");
  delay(500);
  // Checks to see if the TOF sensor has data prepared to be outputed
  if (myImager.isDataReady() == true)
  {
    if (myImager.getRangingData(&measurementData)) // Read distance data into array
    {
      return true;
    }
  }
  Serial.println("NO data processed");
  delay(500);
  return false;
}

//
//
// Checks to see if an object has come within the min detect distance
bool objectDetection()
{
  Serial.println("In object detection function");
  delay(200);
  for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth)
  {
    for (int x = imageWidth - 1; x >= 0; x--)
    {
      // Rejects the values from all of the corner zones when doing general object detection
      switch (x + y)
      {
      case 56:
        break;
      case 57:
        break;
      case 48:
        break;
      case 62:
        break;
      case 63:
        break;
      case 55:
        break;
      case 15:
        break;
      case 7:
        break;
      case 6:
        break;
      case 8:
        break;
      case 0:
        break;
      case 1:
        break;
      default:
        if (measurementData.distance_mm[x + y] < 300)
        {
          Serial.println("Object detected");
          delay(200);
          return true;
        }
        break;
      }
    }
  }
  Serial.println("No object detected");
  delay(200);
  return false;
}

//
//
// Checks to see if an object has come within the min detect distance
int objectAvoidance()
{
  Serial.println("In object avoidance");
  delay(200);
  // Sets direction to undecided(0)
  which_way = 0;

  // Logic for zone 1 & 3
  //Serial.println("logic for zone 1 & 3");
  //delay(500);

  /*Serial.println("zone1locations");
  for (int a = 0; a <= 23; a++)
  {
    Serial.print(measurementData.distance_mm[zone1Locations[a]]);
    Serial.print("   ");
  }

  Serial.println("\n");
  Serial.println("zone2locations");
  for (int d = 0; d <= 15; d++)
  {
    Serial.print(measurementData.distance_mm[zone2Locations[d]]);
    Serial.print("   ");
  }

  Serial.println("\n");
  Serial.println("zone3locations");
  for (int c = 0; c <= 23; c++)
  {
    Serial.print(measurementData.distance_mm[zone3Locations[c]]);
    Serial.print("   ");
  }
  */

  for (int i = 0; i <= 23; ++i)
  {
    int x = measurementData.distance_mm[zone1Locations[i]];
    int y = measurementData.distance_mm[zone3Locations[i]];

    if (x < minDistance)
    {
      zone1Values[i] = 1;
    }
    else
    {
      zone1Values[i] = 0;
    }

    if (y < minDistance)
    {
      zone3Values[i] = 1;
    }
    else
    {
      zone3Values[i] = 0;
    }
  }
  Serial.println();
  //Serial.println("logic zone 2");
  //delay(500);
  // Logic for zone 2
  for (int i = 0; i <= 15; ++i)
  {
    int z = measurementData.distance_mm[zone2Locations[i]];

    if (z < minDistance)
    {
      zone2Values[i] = 1;
    }
    else
    {
      zone2Values[i] = 0;
    }
  }
  
  // Logic to calculate percentages
  double zone1Total = 0;
  double zone2Total = 0;
  double zone3Total = 0;

  for (int i = 0; i <= 23; i++)
  {
    zone1Total = zone1Total + zone1Values[i];
    zone3Total = zone3Total + zone3Values[i];
  }

  for (int i = 0; i <= 15; i++)
  {
    zone2Total = zone2Total = zone2Values[i];
  }
  /*
   Serial.print(zone1Total);
    Serial.print("   ");
    Serial.print(zone2Total);
    Serial.print("   ");
    Serial.println(zone3Total);
    */

  Serial.println("compare percentages");
  //delay(500);
  // Logic get percetages and compare them
  double z1P = (zone1Total / 24) * 100;
  double z2P = (zone2Total / 16) * 100;
  double z3P = (zone3Total / 24) * 100;

  Serial.print(z1P);
  Serial.print("   ");
  Serial.print(z2P);
  Serial.print("   ");
  Serial.println(z3P);

  if ((z1P <= z2P) && (z1P <= z3P))
  {
    // Set direction to left
    Serial.println("left");
    return 1;
  }
  else if ((z2P <= z1P) && (z2P <= z3P))
  {
    // Set direction to forward
    Serial.println("Firward");
    return 2;
  }
  else if ((z3P <= z1P) && (z3P <= z2P))
  {
    // Set direction to right
    Serial.println("right");
    return 3;
  }
  //Serial.println("end object avoidance");
  //delay(500);
  return 0;
}

//
//
//

void get_turn_info()
{
  // Variables
  int direction;
  bool detection;
  // put your main code here, to run repeatedly:
  //delay(1000);

  // Prime sensor to gather good data
  remindTheSensorItExists();

  // Detect if the sensor has data that can be collected and checks the data for any objects within range.
  if (dataProcessed() == true)
  {
    Serial.println("Data Ready!");
    //delay(1000);

    // Runs objectDetection algorithm
    Serial.println("Running Object detection");
    detection = objectDetection();
    //delay(500);
    // remindTheSensorItExists();
    // delay(500);
    pitchServo.write(5);
    //delay(500);

    // If object was detected starts object avoidance
    if (detection == true)
    {
      Serial.println("Object was detected!");
      //Serial.println("Deciding which direction to move...");

      // Runs objectAvoidance algorithm
      which_way = objectAvoidance();
      //Serial.println(which_way);
      if (which_way == 1)
      {
        receivedChars = -90;
        Serial.println("Direction set to left");
      }
      else if (which_way == 2)
      {
        receivedChars = 0;
        Serial.println("Direction set to forward");
      }
      else if (which_way == 3)
      {
        receivedChars = 90;
        Serial.println("Direction set to right");
      }
      else
      {
        Reverse(1000);
        Serial.println("Direction couldn't be decided upon.");
        // reverse a little bit and try again.
      }
    }
  }
}