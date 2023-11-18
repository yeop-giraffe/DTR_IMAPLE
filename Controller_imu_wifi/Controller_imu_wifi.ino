#include <Bluepad32.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// // Barometer
// #include <Adafruit_DPS310.h>
// Adafruit_DPS310 dps;
// #define DPS310_CS 10

// IMU
#include <Arduino.h>
#include <Adafruit_BNO08x.h>
// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9
#define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

//////// Define objects from packages//////////
// Controller (GamePad)
GamepadPtr myGamepads[BP32_MAX_GAMEPADS];
// Motor I2C
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorRB = AFMS.getMotor(4);  // left front motor
Adafruit_DCMotor *motorLB = AFMS.getMotor(3);  // right front motor

Adafruit_MotorShield AFMS2 = Adafruit_MotorShield(0x61);
Adafruit_DCMotor *motorLF = AFMS2.getMotor(3);  // right back motor 
Adafruit_DCMotor *motorRF = AFMS2.getMotor(4);  // left back motor
Adafruit_DCMotor *motorCapture = AFMS2.getMotor(1);  // capture motor

//////// Define global variables ////////////
float turn = 0; // positive = right, negative = left
float forward = 0; /// positive = forward, negative = backward
int catchState = 0;

float yaw_init=0;
float alt_init;
float alt;
float target_yaw = 0;
float target_alt = 0;

float alt_p_error = 0;
float yaw_p_error = 0;

int powerLF; //#1
int powerRF; //#2
int powerLB; //#4
int powerRB; //#3

int leftJoystickX;  // (-511 - 512) left X axis
int Throttlebutton; // for altitude increase (0-1023)
int Brakebutton; // for altitude decrease (0-1023)
int leftJoystickY;  // (-511 - 512) left Y axis
int rightJoystickX; // (-511 - 512) right X axis
int rightJoystickY; // (-511 - 512) right Y axis

// sensors_event_t temp_event, pressure_event;

bool was_turning = false;

//////// Define constants ////////////
#define CATCHSPEED 250
#define SPEED_SCALE 100


void motorSetup(){
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  if (!AFMS2.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield 2. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield 2 found.");

  Serial.println("Motor Setup Completed!");
}


// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myGamepads[i] == nullptr) {
      Serial.printf("CALLBACK: Gamepad is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      GamepadProperties properties = gp->getProperties();
      Serial.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n",
                    gp->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myGamepads[i] = gp;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Gamepad connected, but could not find empty slot");
  }
}

void onDisconnectedGamepad(GamepadPtr gp) {
  bool foundGamepad = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myGamepads[i] == gp) {
      Serial.printf("CALLBACK: Gamepad is disconnected from index=%d\n", i);
      myGamepads[i] = nullptr;
      foundGamepad = true;
      break;
    }
  }

  if (!foundGamepad) {
    Serial.println("CALLBACK: Gamepad disconnected, but not found in myGamepads");
  }
}

// Function to check if a gamepad is connected
bool isGamepadConnected() {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    GamepadPtr myGamepad = myGamepads[i];
    if (myGamepad && myGamepad->isConnected()) {
      return true;
    }
  }
  return false;
}


/////////////////////////// imu ///////////////////////////
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);
 
    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
 
    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}
 
void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}
 
void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}


float imu(){
  if (bno08x.wasReset()) {
    // Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    static long last = 0;
    long now = micros();
    last = now;
  }  
  return ypr.yaw;
}

// float barometer(){
//   //########################  Barometer  ########################//
  
//   while (!dps.temperatureAvailable() || !dps.pressureAvailable()) {
//     return 0; // wait until there's something to read --->>>>>>>>> change into previous value
//   }
 
//   dps.getEvents(&temp_event, &pressure_event);
//   // Serial.print(F("Temperature = "));
//   // Serial.print(temp_event.temperature);
//   // Serial.println(" *C");
 
//   // Serial.print(F("Pressure = "));
//   // Serial.print(pressure_event.pressure);
//   // Serial.println(" hPa");
 
//   // Serial.println();

//   return pressure_event.pressure;
// }


float Calculate_PID_Error(){
  float Kp_alt = 5000;
  float Kp_yaw = 6;

  // alt_p_error = Kp_alt*((pressure_event.pressure-alt_init)-target_alt); // - barometer

  if (target_yaw - (ypr.yaw-yaw_init) > 180){
    yaw_p_error = -Kp_yaw*(target_yaw - (ypr.yaw-yaw_init) - 360);
  } else if (target_yaw - (ypr.yaw-yaw_init) < -180){
    yaw_p_error = -Kp_yaw*(target_yaw - (ypr.yaw-yaw_init) + 360);
  } else{
    yaw_p_error = -Kp_yaw*(target_yaw - (ypr.yaw-yaw_init));
  }

  return yaw_p_error;
}

// Arduino setup function. Runs in CPU 1
void setup() {
  Serial.begin(115200);

  ////--------------- Gamepad Setup ---------------////
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2],
                addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  Serial.println("Setup finished");

  // Wait for Bluetooth connection
  Serial.println("Waiting for Bluetooth connection...");
  while (!isGamepadConnected()) {
    BP32.update();
    delay(100);
  }
  Serial.println("Bluetooth connected!");

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But might also fix some connection / re-connection issues.
  // BP32.forgetBluetoothKeys();

  ////--------------- Motor Setup ---------------////
  motorSetup();

  ////--------------- IMU Setup ---------------////
  byte error, imu_address;
  Wire.begin(22,20);
  imu_address= 0x4A;

  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  Wire.beginTransmission(imu_address);
  error = Wire.endTransmission();
  Serial.println("Adafruit BNO08x test!");
  // delay(50);
  if (error == 0){
      Serial.printf("I2C device found at address 0x%02X\n", imu_address);
      
    } else if(error != 2){
      Serial.printf("Error %d at address 0x%02X\n", error, imu_address);
    }

  // Try to initialize!
  delay(500);
  if (!bno08x.begin_I2C(imu_address,&Wire)) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) yield();
  }
  Serial.println("BNO08x Found!");


  setReports(reportType, reportIntervalUs);
 
  Serial.println("Reading events");
  delay(100);

  // SET INITIAL YAW ANGLE 0
  imu();
  delay(1);
  target_yaw = imu();

  delay(2000);  
}

// Arduino loop function. Runs in CPU 1
void loop() {
  // Serial.print("loop");
  // This call fetches all the gamepad info from the NINA (ESP32) module.
  // Just call this function in your main loop.
  // The gamepads pointer (the ones received in the callbacks) gets updated
  // automatically.
  // delay(1);
  BP32.update();

  // It is safe to always do this before using the gamepad API.
  // This guarantees that the gamepad is valid and connected.
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    GamepadPtr myGamepad = myGamepads[i];
    if (myGamepad && myGamepad->isConnected()) {
      // Serial.println("Gamepad is connected");

      leftJoystickX  =  myGamepad->axisX();  // (-511 - 512) left X axis
      Throttlebutton =  myGamepad->throttle(); // for altitude increase (0-1023)
      Brakebutton    =  myGamepad->brake(); // for altitude decrease (0-1023)
      leftJoystickY  = -myGamepad->axisY();  // (-511 - 512) left Y axis
      rightJoystickX =  myGamepad->axisRX(); // (-511 - 512) right X axis
      rightJoystickY = -myGamepad->axisRY(); // (-511 - 512) right Y axis

      /////////// Quad Control ////////////
      imu();
      Calculate_PID_Error();

      // Altitude control
      if (Throttlebutton > 30){
        target_alt += Throttlebutton/600;
        if (target_alt > 250) {
          target_alt = 250;
        } else if (target_alt < -250){
          target_alt = -250;
        }
      }

      if (Brakebutton > 30){
        target_alt -= Brakebutton/600;
        if (target_alt > 250) {
          target_alt = 250;
        } else if (target_alt < -250){
          target_alt = -250;
        }
      }      

      // Yaw control
      if (abs(rightJoystickX) > 30){
        target_yaw -= rightJoystickX/400;
        if (target_yaw > ypr.yaw+30){
          target_yaw = ypr.yaw+30;
        }
        if (target_yaw < ypr.yaw-30){
          target_yaw = ypr.yaw-30;
        }

        if (target_yaw > 180){
          target_yaw -= 360;
        }
        if (target_yaw < -180){
          target_yaw += 360;
        }
      } else if (abs(rightJoystickX) > 1){
        // turn = 0;
        turn = yaw_p_error;
      }

      turn = yaw_p_error;

      // turn = yaw_p_error;
      
      // Direction control
      if (abs(leftJoystickX) > 30){
        forward = leftJoystickX/2;
      } else if (abs(leftJoystickX) > 1){
        forward = 0;
      }

      if ( forward > 200){
        forward = 200;
      } 
      if ( forward < -200){
        forward = -200;
      }
      
      // Kill altitude
      if (myGamepad->x()){
        target_alt = 0;
      }

      if ( turn > 200){
        turn = 200;
      } 
      if ( turn < -200){
        turn = -200;
      }

      powerLF = target_alt + turn + forward; //#1
      powerRF = target_alt - turn + forward; //#2
      powerLB = target_alt - turn - forward; //#4
      powerRB = target_alt + turn - forward; //#3

      motorLF->setSpeed(constrain(abs(powerLF), 0, 240));
      motorRF->setSpeed(constrain(abs(powerRF), 0, 240));
      motorLB->setSpeed(constrain(abs(powerLB), 0, 240));
      motorRB->setSpeed(constrain(abs(powerRB), 0, 240));

      if (powerLF > 0){
        motorLF->run(BACKWARD);
      } else {
        motorLF->run(FORWARD);
      }

      if (powerRF > 0){
        motorRF->run(FORWARD);
      } else {
        motorRF->run(BACKWARD);
      }

      if (powerLB > 0){
        motorLB->run(FORWARD);
      } else {
        motorLB->run(BACKWARD);
      }

      if (powerRB > 0){
        motorRB->run(BACKWARD);
      } else {
        motorRB->run(FORWARD);
      }

      ///////// Catching Mechanism Control ////////////
      
      // Catch
      if (myGamepad->r1()){
        catchState = 1;
      }
      // Release
      if (myGamepad->b()){
        catchState = 2;
      }
      // Hold
      if (myGamepad->l1()){
        catchState = 3;
      }
      // Neutral
      if (myGamepad->a()){
        catchState = 0;
      }

      // send command to the motor based on the states
      switch (catchState){
        case 0: // Neutral
          motorCapture->setSpeed(0);
          motorCapture->run(FORWARD);
          break;

        case 1: // Capture
          motorCapture->setSpeed(CATCHSPEED);
          motorCapture->run(BACKWARD);
          break;

        case 2: // Release
          motorCapture->setSpeed(CATCHSPEED/3);
          motorCapture->run(FORWARD);
          break;

        case 3: // Hold
          motorCapture->setSpeed(CATCHSPEED/2);
          motorCapture->run(BACKWARD);
          break;
      }


    }
  }

  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
  Serial.print("LF: ");
  Serial.print(powerLF);
  Serial.print(" RF: ");
  Serial.print(powerRF);
  Serial.print(" RB: ");
  Serial.print(powerRB);
  Serial.print(" LB: ");
  Serial.print(powerLB);
  Serial.print(" | ");

  Serial.print(leftJoystickX);
  Serial.print(" / ");
  Serial.print(rightJoystickX);
  Serial.print(" | ");

  Serial.print(target_alt);
  Serial.print(" | ");
  Serial.print(target_yaw);
  Serial.print(" | ");
  Serial.print(turn);
  Serial.print(" | ");
  Serial.println(forward);
  


  vTaskDelay(1);
  // delay(150);
}
