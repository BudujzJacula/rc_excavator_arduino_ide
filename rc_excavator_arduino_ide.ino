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

const uint8_t motor_1_A = 0;
const uint8_t motor_1_B = 1;

const uint8_t motor_2_A = 2;
const uint8_t motor_2_B = 3;

const uint8_t motor_3_A = 4;
const uint8_t motor_3_B = 7;

const uint8_t motor_4_A = 5;
const uint8_t motor_4_B = 6;

const uint8_t motor_5_A = 8;
const uint8_t motor_5_B = 9;

const uint8_t motor_6_A = 10;
const uint8_t motor_6_B = 11;

const uint8_t motor_7_A = 12;
const uint8_t motor_7_B = 13;

const uint8_t motor_8_A = 14;
const uint8_t motor_8_B = 15;

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

  // right drive
  if (right_drive_speed <= -40) {
    Serial.println("Driving right track forward");
    drive_actuator(RIGHT_DRIVE, RIGHT_TRACK_FORWARD, right_drive_speed);
  }
  else if (right_drive_speed >= 40) {
    // Serial.println("Driving right track backward");
    drive_actuator(RIGHT_DRIVE, RIGHT_TRACK_BACKWARD, right_drive_speed);
  }
  else {
    // Serial.println("Stopping right track");
    drive_actuator(RIGHT_DRIVE, RIGHT_TRACK_STOP, 0);
  }

  // left drive
  if (left_drive_speed <= -40) {
    Serial.println("Driving left track forward");
    drive_actuator(LEFT_DRIVE, LEFT_TRACK_FORWARD, left_drive_speed);
  }
  else if (left_drive_speed >= 40) {
    // Serial.println("Driving left track backward");
    drive_actuator(LEFT_DRIVE, LEFT_TRACK_BACKWARD, left_drive_speed);
  }
  else {
    // Serial.println("Stopping left track");
    drive_actuator(LEFT_DRIVE, LEFT_TRACK_STOP, 0);
  }

  // swing
  if (swing_speed > 50) {
    Serial.println("Swing right");
    drive_actuator(SWING_MOTOR, SWING_RIGHT, swing_speed);
  }
  else if (swing_speed < -50) {
    // Serial.println("Swing left");
    drive_actuator(SWING_MOTOR, SWING_LEFT, swing_speed);
  }
  else {
    // Serial.println("Swing stop");
    drive_actuator(SWING_MOTOR, SWING_STOP, 0);
  }

  // arm
  // zwiekszone limity treshold do +/-50
  if (arm_speed > 50) {
    Serial.println("Arm in");
    drive_actuator(ARM, ARM_IN, arm_speed);
  }
  else if (arm_speed < -50) {
    // Serial.println("Arm out");
    drive_actuator(ARM, ARM_OUT, arm_speed);
  }
  else {
    // Serial.println("Arm stop");
    drive_actuator(ARM, ARM_STOP, 0);
  }

  // dipper
  if (dipper_speed > 50) {
    Serial.println("Dipper in");
    drive_actuator(DIPPER, DIPPER_IN, dipper_speed);
  }
  else if (dipper_speed < -50) {
    // Serial.println("Dipper out");
    drive_actuator(DIPPER, DIPPER_OUT, dipper_speed);
  }
  else {
    // Serial.println("Dipper stop");
    drive_actuator(DIPPER, DIPPER_STOP, 0);
  }

  // bucket
  if (bucket_speed > 50) {
    Serial.println("Bucket in");
    drive_actuator(BUCKET, BUCKET_IN, bucket_speed);
  }
  else if (bucket_speed < -50) {
    // Serial.println("Bucket out");
    drive_actuator(BUCKET, BUCKET_OUT, bucket_speed);
  }
  else {
    // Serial.println("Bucket stop");
    drive_actuator(BUCKET, BUCKET_STOP, 0);
  }

  // thumb
  if (thumb_in_speed > thumb_out_speed + 10) {
    Serial.println("Thumb in");
    Serial.print("thumb in speed: ");
    Serial.println(thumb_in_speed);
    drive_actuator(THUMB, THUMB_IN, thumb_in_speed);
  }
  else if (thumb_in_speed < thumb_out_speed - 10) {
    Serial.println("Thumb out");
    Serial.print("thumb out speed: ");
    Serial.println(thumb_out_speed);
    drive_actuator(THUMB, THUMB_OUT, thumb_out_speed);
  }
  else {
    // Serial.println("Thumb stop");
    Serial.print("thumb in speed: ");
    Serial.print(thumb_in_speed);
    Serial.print(" thumb out speed: ");
    Serial.print(thumb_out_speed);
    Serial.print(" thumb in - thumb out: ");
    Serial.println(thumb_in_speed - thumb_out_speed);
    drive_actuator(THUMB, THUMB_STOP, 0);
  }






  if (ctl->l1()) {
    // Serial.println("L1");
  }

  if (ctl->l2()) {
    // Serial.println("L2");
  }

  if (ctl->r1()) {
    // Serial.println("R1");
  }

  if (ctl->r2()) {
    // Serial.println("R2");
  }
  // Another way to query controller data is by getting the buttons() function.
  // See how the different "dump*" functions dump the Controller info.
  // dumpGamepad(ctl);
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    } else {
    // tutaj dopisac zabezpieczenie wylaczajace wszystkie silniki kiedy nie ma polaczenia z kontrolerem
    // LEFT_DRIVE = 1,
    // RIGHT_DRIVE,
    // SWING_MOTOR,
    // ARM,
    // DIPPER,
    // BUCKET,
    // THUMB,
    // Serial.println("**************************************************");
    // drive_actuator(LEFT_DRIVE, LEFT_TRACK_STOP, 0);
    // drive_actuator(RIGHT_DRIVE, RIGHT_TRACK_STOP, 0);
    // drive_actuator(SWING_MOTOR, SWING_STOP, 0);
    // drive_actuator(ARM, ARM_STOP, 0);
    // drive_actuator(DIPPER, DIPPER_STOP, 0);
    // drive_actuator(BUCKET, BUCKET_STOP, 0);
    // drive_actuator(THUMB, THUMB_STOP, 0);
    }
  }
}

void drive_motor(uint8_t pinA, uint8_t pinB, int8_t direction, int16_t speed) {
  // Serial.println("Drive motor");
  // Serial.print("Speed before conversion: ");
  // Serial.println(speed);
  if (speed < 0)
  {
    speed = speed * -8;
    // Serial.print("Speed after conversion: ");
    // Serial.println(speed);
  }
  else {
    speed = speed * 8;
    // Serial.print("Speed after conversion: ");
    // Serial.println(speed);
  }

  // multiplying *4 because pwm is up to 4096 (12bit) and joystick is up to 512
  // speed = speed * 4;
  // Serial.print("Speed * 4:");
  // Serial.println(speed);

  if (direction == 1) {
    // Serial.println("Direction 1");
    // Serial.println("PinA set");
    pwm.setPWM(pinA, 0, 4096);
    pwm.setPWM(pinB, 0, speed);
  }
  else if (direction == -1) {
    // Serial.println("Direction -1");
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

void drive_actuator(Actuators motor, int8_t direction, uint16_t speed) {
  // Serial.println("Drive actuator");
  switch (motor)
  {
  case LEFT_DRIVE:
    // Serial.println("LEFT_DRIVE");
    drive_motor(motor_1_A, motor_1_B, direction, speed);

    break;
  case RIGHT_DRIVE:
    // Serial.println("RIGHT_DRIVE");
    drive_motor(motor_2_A, motor_2_B, direction, speed);

    break;
  case SWING_MOTOR:
    // Serial.println("SWING MOTOR");
    drive_motor(motor_3_A, motor_3_B, direction, speed);

    break;
  case ARM:
    // Serial.println("ARM MOTOR");
    drive_motor(motor_4_A, motor_4_B, direction, speed);

    break;
  case DIPPER:
    // Serial.println("DIPPER MOTOR");
    drive_motor(motor_5_A, motor_5_B, direction, speed);

    break;
  case BUCKET:
    // Serial.println("BUCKET MOTOR");
    drive_motor(motor_6_A, motor_6_B, direction, speed);

    break;
  case THUMB:
    // Serial.println("THUMB MOTOR");
    drive_motor(motor_7_A, motor_7_B, direction, speed);

    break;
  case AUX:
    // Serial.println("AUX");
    drive_motor(motor_8_A, motor_8_B, direction, speed);
    
    break;
  
  default:
    break;
  }
}

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
  delay(5);

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
