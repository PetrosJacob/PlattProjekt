
// Basic demo for accelerometer readings from Adafruit LIS3DH

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
// Used for hardware & software SPI
#define LIS3DH_CS 10



Servo Servo_x;
Servo Servo_y;

int pos_x = 0;
int pos_y = 0;

Adafruit_LIS3DH lis = Adafruit_LIS3DH();
float accel_angle_x;
float accel_angle_y;

// X värde
const int ARRAY_SIZE = 15;
int pos_x_history[ARRAY_SIZE] = {0};
int current_index_x = 0;

// y värde
int pos_y_history[ARRAY_SIZE] = {0};
int current_index_y = 0;

void setup(void) {
  Servo_x.attach(9); // Servo_x is attacted to pin 9 
  Servo_y.attach(10);//Servo_x is attacted to pin 10
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("LIS3DH test!");

  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1) yield();
  }
  Serial.println("LIS3DH found!");

  // lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

  Serial.print("Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");

  // lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  Serial.print("Data rate set to: ");
  switch (lis.getDataRate()) {
    case LIS3DH_DATARATE_1_HZ: Serial.println("1 Hz"); break;
    case LIS3DH_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3DH_DATARATE_25_HZ: Serial.println("25 Hz"); break;
    case LIS3DH_DATARATE_50_HZ: Serial.println("50 Hz"); break;
    case LIS3DH_DATARATE_100_HZ: Serial.println("100 Hz"); break;
    case LIS3DH_DATARATE_200_HZ: Serial.println("200 Hz"); break;
    case LIS3DH_DATARATE_400_HZ: Serial.println("400 Hz"); break;
    case LIS3DH_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
    case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.println("5 Khz Low Power"); break;
    case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.println("16 Khz Low Power"); break;
  }
}

void loop() {

  /* Or....get a new sensor event, normalized */
  sensors_event_t event;
  lis.getEvent(&event);
  calc_xy_angles();
  delay(10);
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tX: "); Serial.print(event.acceleration.x);
  Serial.print(" \tY: "); Serial.print(event.acceleration.y);
  Serial.print(" \tZ: "); Serial.print(event.acceleration.z);
  Serial.println(" m/s^2 ");

  Serial.println();

}

void calc_xy_angles(void){
    sensors_event_t event;
    lis.getEvent(&event);
  
   // Using x y and z from accelerometer, calculate x and y angles
   float x_val, y_val, z_val, result;
   float x2, y2, z2; //24 bit

   // Lets get the deviations from our baseline
   x_val = (float)event.acceleration.x;
   y_val = (float)event.acceleration.y;
   z_val = (float)event.acceleration.z;
   // Work out the squares
   x2 = (float)(x_val*x_val);
   y2 = (float)(y_val*y_val);
   z2 = (float)(z_val*z_val);

   //X Axis
   result=sqrt(y2+z2);
   result=x_val/result;
   accel_angle_x = atan(result)*180/PI;
   pos_x = accel_angle_x;
   pos_x = map(pos_x,-90,90,0,180);
   Serial.print(" \tVInkel X: "); Serial.print(accel_angle_x);
   Serial.print(" \tServo X: "); Serial.print(pos_x);
   
    // Add the current value of pos_x to the array
  pos_x_history[current_index_x] = pos_x;

  // Update the current index in the array
  current_index_x = (current_index_x + 1) % ARRAY_SIZE;

  // Calculate the average value of the pos_x_history array
  int sum_x = 0;
  for (int i = 0; i < ARRAY_SIZE; i++) {
    sum_x += pos_x_history[i];
  }
  float average_value_x = (float)sum_x / ARRAY_SIZE;
  //Output desired angle to Servo_x
  Servo_x.write(average_value_x);

   //Y Axis
   result=sqrt(x2+z2);
   result=y_val/result;
   accel_angle_y = atan(result)*180/PI;
   pos_y = accel_angle_y;
   pos_y = map(pos_y,-90,90,0,180);
   Serial.print(" \tVInkel Y: "); Serial.print(accel_angle_y);
   Serial.print(" \tServo Y: "); Serial.print(pos_y);
   pos_y_history[current_index_y] = pos_y;

  // Update the current index in the array
  current_index_y = (current_index_y + 1) % ARRAY_SIZE;

  // Calculate the average value of the pos_x_history array
  int sum_y = 0;
  for (int i = 0; i < ARRAY_SIZE; i++) {
    sum_y += pos_y_history[i];
  }
  float average_value_y = (float)sum_y / ARRAY_SIZE;
  //Output desired angle to Servo_y
  Servo_y.write(average_value_y);
   
}
