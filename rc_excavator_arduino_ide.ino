#include <Arduino.h>
#include <SPI.h>
#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"
#include <Bluepad32.h>

/*
TODO:
- przemyslec czy dodac jakis input zeby robic forgetBluetoothKeys(),
- dodac sterowanie thumbem przez l2, r2
- dodac sterowanie auxem i wybrac do niego przyciski
*/

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
Motor connectors pinouts:
 brown left track 1  ** dipper 1 red
 yellow left track 2  ** dipper 2 black
 orange right track 1 ** bucket 1 yellow
 green right track 2 ** bucket 2 white
 yellow arm 1         ** thumb 1 black
 white arm 2         ** thumb 2 red
 yellow swing 1       ** aux 1
 white swing 2       ** aux 2
*/

const uint8_t left_motor_A   = 0;
const uint8_t left_motor_B   = 1;

const uint8_t right_motor_A  = 2;
const uint8_t right_motor_B  = 3;

const uint8_t swing_motor_A  = 4;
const uint8_t swing_motor_B  = 7;

const uint8_t arm_motor_A    = 5;
const uint8_t arm_motor_B    = 6;

const uint8_t dipper_motor_A = 8;
const uint8_t dipper_motor_B = 9;

const uint8_t bucket_motor_A = 10;
const uint8_t bucket_motor_B = 11;

const uint8_t thumb_motor_A  = 12;
const uint8_t thumb_motor_B  = 13;

const uint8_t aux_motor_A    = 14;
const uint8_t aux_motor_B    = 15;

const uint8_t BATTERY_INPUT       = 34;
// voltage divider ratio: 32560 / (9777 + 32560) = 32560 / 42337 = 0.769
// 1/ratio = 1,30
const float VBAT_DIVIDER_COMP     = 1.30F;
const uint16_t ADC_RES            = 1024;
const float MAX_VOLTAGE           = 3.3F;
const float BATTERY_OFFSET        = 6.0F;
const float USABLE_BATTERY_RANGE  = 2.4F;
const uint8_t ADC_AVERAGE_SAMPLES = 10;
float voltage                     = 0;
uint8_t voltage_percent           = 0;

int16_t right_joystick_Y  = 0;
int16_t left_joystick_Y   = 0;
int16_t right_drive_speed = 0;
int16_t left_drive_speed  = 0;

int16_t arm_speed    = 0;
int16_t bucket_speed = 0;
int16_t dipper_speed = 0;
int16_t swing_speed  = 0;

int16_t thumb_speed     = 0;
int16_t thumb_in_speed  = 0;
int16_t thumb_out_speed = 0;

bool a = false;
bool b = false;
bool x = false;
bool y = false;

bool l1 = false;
bool r1 = false;

// dopoki check_battery() nie bedzie dzialalo jak powinno to ta flaga jest utrzymywana jako true zeby nie blokowac silnikow
bool battery_flag = true;

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
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

void processGamepad(ControllerPtr ctl) {
  right_joystick_Y = ctl->axisRY();
  left_joystick_Y = ctl->axisY();
  r1 = ctl->r1();

  thumb_in_speed = ctl->throttle();
  thumb_out_speed = ctl->brake();
  thumb_speed = (thumb_in_speed - thumb_out_speed) / 2;

  // jesli r1 wcisniety to analogi steruja gasienicami
  // jesli r1 nie wcisniety to analogi steruja ramieniem i obrotem koparki wg standardu ISO
  if (r1) {
    // Driving tracks
    right_drive_speed = right_joystick_Y;
    left_drive_speed = left_joystick_Y;

    arm_speed = 0;
    dipper_speed = 0;
    bucket_speed = 0;
    swing_speed = 0;
  }
  else {
    // Arm control
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
  Serial.print(">Thumb: ");
  Serial.println(thumb_speed);
  // Serial.print(">Battery voltage: ");
  // Serial.println(voltage);
  // Serial.print(">Battery percent: ");
  // Serial.println(voltage_percent);
  // Serial.print(">Battery flag: ");
  // Serial.println(battery_flag);

  if (battery_flag) {
    drive_motor(right_motor_A, right_motor_B, right_drive_speed, -80, 80);
    drive_motor(left_motor_A, left_motor_B, left_drive_speed, -80, 80);
    drive_motor(swing_motor_A, swing_motor_B, swing_speed, -80, 80);
    drive_motor(arm_motor_A, arm_motor_B, arm_speed, -80, 80);
    drive_motor(dipper_motor_A, dipper_motor_B, dipper_speed, -80, 80);
    drive_motor(bucket_motor_A, bucket_motor_B, bucket_speed, -80, 80);
    drive_motor(thumb_motor_A, thumb_motor_B, thumb_speed, -0, 0);
  }
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

void drive_motor(uint8_t pinA, uint8_t pinB, int16_t speed, int8_t n_threshold, int8_t p_threshold) {
  // clamping to -511 to prevent overflow and motor stop
  if (speed <= -511) {
    speed = -511;
  }

  // multiplying *8 because pwm is up to 4096 and joystick is up to 512
  if (speed > p_threshold) {
    speed = speed * 8;
    pwm.setPWM(pinA, 0, 4096);
    pwm.setPWM(pinB, 0, speed);
  }
  else if (speed < n_threshold) {
    speed = speed * -8;
    pwm.setPWM(pinB, 0, 4096);
    pwm.setPWM(pinA, 0, speed);
  }
  else {
    pwm.setPWM(pinA, 4096, 0);
    pwm.setPWM(pinB, 4096, 0);
  }

  // pwm.setPWM(pin, 4096, 0);       // turns pin fully on
  // pwm.setPWM(pin, 0, 4096);       // turns pin fully off
}

void check_battery(void){
  uint16_t battery_read = 0;

  for (uint8_t i = 0; i < ADC_AVERAGE_SAMPLES; i++) {
    uint16_t battery_sample = analogRead(BATTERY_INPUT);
    battery_read = battery_read + battery_sample;
  }

  battery_read = battery_read / ADC_AVERAGE_SAMPLES;
  Serial.print(">Battery read: ");
  Serial.println(battery_read);
  voltage = battery_read * (MAX_VOLTAGE / ADC_RES);
  Serial.print(">Battery voltage1: ");
  Serial.println(voltage);
  voltage = voltage * VBAT_DIVIDER_COMP;
  Serial.print(">Battery voltage2: ");
  Serial.println(voltage);
  voltage_percent = (((voltage - BATTERY_OFFSET) / USABLE_BATTERY_RANGE) * 100);
  Serial.print(">Battery percent: ");
  Serial.println(voltage_percent);

  // if (voltage > 6.0) {
  //   battery_flag = true;
  // }
  // else {
  //   battery_flag = false;
  // }

  // Serial.print(">Battery voltage: ");
  // Serial.println(voltage);
}

// Arduino setup function. Runs in CPU 1
void setup() {
  Serial.begin(115200);

  pwm.begin();
  pwm.setPWMFreq(50);

  analogReadResolution(10);
  analogSetClockDiv(255);
  analogSetAttenuation(ADC_11db);

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

  check_battery();
}
