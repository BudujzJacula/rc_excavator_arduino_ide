#include <Arduino.h>
#include <SPI.h>
#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"
#include <Bluepad32.h>

enum Actuators {
  LEFT_DRIVE = 1,
  RIGHT_DRIVE,
  SWING_MOTOR,
  ARM,
  DIPPER,
  BUCKET,
  THUMB,
  AUX,
};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

/*
 left track 1  ** dipper 1
 left track 2  ** dipper 2
 right track 1 ** bucket 1
 right track 2 ** bucket 2
 arm 1         ** thumb 1
 arm 2         ** thumb 2
 swing 1       ** aux 1
 swing 2       ** aux 2
*/

const uint8_t left_motor_A = 0;
const uint8_t left_motor_B = 1;

const uint8_t right_motor_A = 2;
const uint8_t motor_2_B = 3;

const uint8_t swing_motor_A = 4;
const uint8_t swing_motor_B = 7;

const uint8_t arm_motor_A = 5;
const uint8_t arm_motor_B = 6;

const uint8_t dipper_motor_A = 8;
const uint8_t dipper_motor_B = 9;

const uint8_t bucket_motor_A = 10;
const uint8_t bucket_motor_B = 11;

const uint8_t thumb_motor_A = 12;
const uint8_t thumb_motor_B = 13;

const uint8_t aux_motor_A = 14;
const uint8_t aux_motor_B = 15;

// REFACTOR zamiast ARM_OUT=1 i BUCKET_OUT=1 zrobic jedno OUT=1, tak samo z forward i backward
const bool LEFT_TRACK_FORWARD  = 1;
const bool LEFT_TRACK_STOP     = 0;
const int LEFT_TRACK_BACKWARD  = -1;

const int RIGHT_TRACK_FORWARD = -1;
const bool RIGHT_TRACK_STOP     = 0;
const bool RIGHT_TRACK_BACKWARD = 1;

const bool SWING_RIGHT = 1;
const bool SWING_STOP = 0;
const int SWING_LEFT = -1;

const bool ARM_OUT = 1;
const bool ARM_STOP = 0;
const int ARM_IN = -1;

const bool DIPPER_OUT = 1;
const bool DIPPER_STOP = 0;
const int DIPPER_IN = -1;

const bool BUCKET_OUT = 1;
const bool BUCKET_STOP = 0;
const int BUCKET_IN = -1;

const bool THUMB_OUT = 1;
const bool THUMB_STOP = 0;
const int THUMB_IN = -1;

int right_joystick_Y = 0;
int left_joystick_Y = 0;
int right_drive_speed = 0;
int left_drive_speed = 0;

int arm_speed = 0;
int bucket_speed = 0;
int dipper_speed = 0;
int swing_speed = 0;

int thumb_in_speed = 0;
int thumb_out_speed = 0;

bool a = false;
bool b = false;
bool x = false;
bool y = false;

bool l1 = false;
bool r1 = false;
int l2 = 0;
int r2 = 0;

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                      properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
    "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
    "misc: 0x%02x, l1:%1x, r1:%1x, b:%1x\n",
    ctl->index(),        // Controller Index
    ctl->dpad(),         // D-pad
    ctl->buttons(),      // bitmask of pressed buttons
    ctl->axisX(),        // (-511 - 512) left X Axis
    ctl->axisY(),        // (-511 - 512) left Y axis
    ctl->axisRX(),       // (-511 - 512) right X axis
    ctl->axisRY(),       // (-511 - 512) right Y axis
    ctl->l2(),           // (0 - 1023): brake button
    ctl->r2(),           // (0 - 1023): throttle (AKA gas) button
    ctl->miscButtons(),  // bitmask of pressed "misc" buttons
    //
    ctl->l1(),
    ctl->r1(),
    ctl->b()
  );
}

void processGamepad(ControllerPtr ctl) {
  right_joystick_Y = ctl->axisRY();
  left_joystick_Y = ctl->axisY();
  r1 = ctl->r1();

  thumb_in_speed = ctl->throttle();
  thumb_out_speed = ctl->brake();

  // // jesli r1 wcisniety to analogi steruja gasienicami
  // // jesli r1 nie wcisniety to analogi steruja ramieniem i obrotem koparki wg standardu ISO
  if (r1) {
    // Serial.println("Driving tracks");
    right_drive_speed = right_joystick_Y;
    left_drive_speed = left_joystick_Y;

    arm_speed = 0;
    dipper_speed = 0;
    bucket_speed = 0;
    swing_speed = 0;
  }
  else {
    // Serial.println("Arm control");
    right_drive_speed = 0;
    left_drive_speed = 0;

    arm_speed = right_joystick_Y;
    dipper_speed = left_joystick_Y;
    bucket_speed = ctl->axisRX();
    swing_speed = ctl->axisX();
  }
  Serial.print(">Left track: ");
  Serial.println(left_drive_speed);
  Serial.print(">Right track: ");
  Serial.println(right_drive_speed);
  Serial.print(">Swing: ");
  Serial.println(swing_speed);
  Serial.print(">Arm: ");
  Serial.println(arm_speed);
  Serial.print(">Dipper: ");
  Serial.println(dipper_speed);
  Serial.print(">Bucket: ");
  Serial.println(bucket_speed);

  drive_motor(right_motor_A, right_motor_A, right_drive_speed);
  drive_motor(left_motor_A, left_motor_B, left_drive_speed);
  drive_motor(swing_motor_A, swing_motor_B, swing_speed);
  drive_motor(arm_motor_A, arm_motor_B, arm_speed);
  drive_motor(dipper_motor_A, dipper_motor_B, dipper_speed);
  drive_motor(bucket_motor_A, bucket_motor_B, bucket_speed);
  

  // // right drive
  // if (right_drive_speed <= -40) {
  //   // drive_actuator(RIGHT_DRIVE, right_drive_speed);
  //   drive_motor(right_motor_A, right_motor_A, right_drive_speed);
  // }
  // else if (right_drive_speed >= 40) {
  //   // drive_actuator(RIGHT_DRIVE, right_drive_speed);
  //   drive_motor(right_motor_A, right_motor_A, right_drive_speed);
  // }

  // // left drive
  // if (left_drive_speed <= -40) {
  //   // drive_actuator(LEFT_DRIVE, left_drive_speed);
  //   drive_motor(left_motor_A, left_motor_B, left_drive_speed);
  // }
  // else if (left_drive_speed >= 40) {
  //   // drive_actuator(LEFT_DRIVE, left_drive_speed);
  //   drive_motor(left_motor_A, left_motor_B, left_drive_speed);
  // }

  // // swing
  // if (swing_speed > 50) {
  //   // drive_actuator(SWING_MOTOR, swing_speed);
  //   drive_motor(swing_motor_A, swing_motor_B, swing_speed);
  // }
  // else if (swing_speed < -50) {
  //   // drive_actuator(SWING_MOTOR, swing_speed);
  //   drive_motor(swing_motor_A, swing_motor_B, swing_speed);
  // }

  // // arm
  // // zwiekszone limity treshold do +/-50
  // if (arm_speed > 50) {
  //   // drive_actuator(ARM, arm_speed);
  //   drive_motor(arm_motor_A, arm_motor_B, arm_speed);
  // }
  // else if (arm_speed < -50) {
  //   // drive_actuator(ARM, arm_speed);
  //   drive_motor(arm_motor_A, arm_motor_B, arm_speed);
  // }

  // // dipper
  // if (dipper_speed > 50) {
  //   // drive_actuator(DIPPER, dipper_speed);
  //   drive_motor(dipper_motor_A, dipper_motor_B, dipper_speed);
  // }
  // else if (dipper_speed < -50) {
  //   // drive_actuator(DIPPER, dipper_speed);
  //   drive_motor(dipper_motor_A, dipper_motor_B, dipper_speed);
  // }

  // // bucket
  // if (bucket_speed > 50) {
  //   // drive_actuator(BUCKET, bucket_speed);
  //   drive_motor(bucket_motor_A, bucket_motor_B, bucket_speed);
  // }
  // else if (bucket_speed < -50) {
  //   // drive_actuator(BUCKET, bucket_speed);
  //   drive_motor(bucket_motor_A, bucket_motor_B, bucket_speed);
  // }
  
  // // thumb
  // if (thumb_in_speed > thumb_out_speed + 10) {
  //   // drive_actuator(THUMB, thumb_in_speed);
  //   drive_motor(thumb_motor_A, thumb_motor_B, thumb_in_speed);
  // }
  // else if (thumb_in_speed < thumb_out_speed - 10) {
  //   // drive_actuator(THUMB, thumb_out_speed);
  //   drive_motor(thumb_motor_A, thumb_motor_B, thumb_out_speed);
  // }

  // Another way to query controller data is by getting the buttons() function.
  // See how the different "dump*" functions dump the Controller info.
  // dumpGamepad(ctl);
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
        Serial.print("NEW DATA******************* ");
        Serial.println(millis());
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

void drive_motor(uint8_t pinA, uint8_t pinB, int16_t speed) {
  // tutaj tez przerobic i wywalic calkowicie direction i okreslac go na podstawie czy speed > 0 czy < 0 (z jakimś marginesem)
  // multiplying *4 because pwm is up to 4096 (12bit) and joystick is up to 512
  // if (speed < 0)
  // {
  //   speed = speed * -8;
  //   // Serial.print("Speed after conversion: ");
  //   // Serial.println(speed);
  // }
  // else {
  //   speed = speed * 8;
  //   // Serial.print("Speed after conversion: ");
  //   // Serial.println(speed);
  // }
  // Serial.print(">Drive motor speed: ");
  // Serial.println(speed);
  // Serial.print(">Direction: ");
  // Serial.println(direction);

  if (speed <= -511) {
    speed = -511;
  }

  if (speed > 80) {
    // Serial.println("Direction 1");
    // Serial.println("PinA set");
    speed = speed * 8;
    pwm.setPWM(pinA, 0, 4096);
    pwm.setPWM(pinB, 0, speed);
  }
  else if (speed < -80) {
    // Serial.println("Direction -1");
    speed = speed * -8;
    pwm.setPWM(pinB, 0, 4096);
    pwm.setPWM(pinA, 0, speed);
  }
  else {
    // Serial.println("Direction 0");
    pwm.setPWM(pinA, 4096, 0);
    pwm.setPWM(pinB, 4096, 0);
  }

  // pwm.setPWM(pin, 4096, 0);       // turns pin fully on
  // pwm.setPWM(pin, 0, 4096);       // turns pin fully off
}

// void drive_actuator(Actuators motor, int16_t speed) {
//   // przerobic funkcje i wywalic direction, zostawic speed jako wartosc <-512, 512>
//   // Serial.println("Drive actuator");
//   switch (motor)
//   {
//   case LEFT_DRIVE:
//     // Serial.println("LEFT_DRIVE");
//     Serial.print(">Left track: ");
//     Serial.println(speed);
//     drive_motor(left_motor_A, left_motor_B, speed);

//     break;
//   case RIGHT_DRIVE:
//     // Serial.println("RIGHT_DRIVE");
//     Serial.print(">Right track: ");
//     Serial.println(speed);
//     drive_motor(right_motor_A, motor_2_B, speed);

//     break;
//   case SWING_MOTOR:
//     // Serial.println("SWING MOTOR");
//     Serial.print(">Swing: ");
//     Serial.println(speed);
//     drive_motor(swing_motor_A, swing_motor_B, speed);

//     break;
//   case ARM:
//     // Serial.println("ARM MOTOR");
//     Serial.print(">Arm: ");
//     Serial.println(speed);
//     drive_motor(arm_motor_A, arm_motor_B, speed);

//     break;
//   case DIPPER:
//     // Serial.println("DIPPER MOTOR");
//     Serial.print(">Dipper: ");
//     Serial.println(speed);
//     drive_motor(dipper_motor_A, dipper_motor_B, speed);

//     break;
//   case BUCKET:
//     // Serial.println("BUCKET MOTOR");
//     Serial.print(">Bucket: ");
//     Serial.println(speed);
//     drive_motor(bucket_motor_A, bucket_motor_B, speed);

//     break;
//   case THUMB:
//     // Serial.println("THUMB MOTOR");
//     Serial.print(">Thumb: ");
//     Serial.println(speed);
//     drive_motor(thumb_motor_A, thumb_motor_B, speed);

//     break;
//   case AUX:
//     // Serial.println("AUX");
//     drive_motor(aux_motor_A, aux_motor_B, speed);
    
//     break;
  
//   default:
//     break;
//   }
// }

/*
do poprawy:
- poprawienie martwej strefy w napedzie obrotu,
- zdebuggowanie zatrzymania silnika przy pelnym wychyleniu drazka
*/

// Arduino setup function. Runs in CPU 1
void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  // BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
}

void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();

  // drive_actuator(THUMB, THUMB_IN, 200);
  // drive_actuator(BUCKET, BUCKET_IN, 200);

  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  //     vTaskDelay(1);
  // delay(5);

  // Serial.println("Driving actuators in loop()");
  // drive_actuator(LEFT_DRIVE, 0, 2048);
  // drive_actuator(RIGHT_DRIVE, 0, 2048);
  // drive_actuator(SWING_MOTOR, 0, 2048);
  // drive_actuator(ARM, 0, 2048);
  // drive_actuator(DIPPER, 0, 2048);
  // drive_actuator(BUCKET, 0, 2048);
  // drive_actuator(THUMB, 0, 2048);
  // drive_actuator(AUX, 0, 2048);
  // delay(1000);
  // drive_actuator(LEFT_DRIVE, 1, 2048);
  // drive_actuator(RIGHT_DRIVE, 1, 2048);
  // drive_actuator(SWING_MOTOR, 1, 2048);
  // drive_actuator(ARM, 1, 2048);
  // drive_actuator(DIPPER, 1, 2048);
  // drive_actuator(BUCKET, 1, 2048);
  // drive_actuator(THUMB, 1, 2048);
  // drive_actuator(AUX, 1, 2048);
  // delay(1000);
}
