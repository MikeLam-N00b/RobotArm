/**
 * @file ESP32bluetooth.ino
 * @brief Bluetooth RC servo controller for ESP32 (Arduino framework).
 *
 * Copyright (c) 2025 Lâm Kim Bảo
 * SPDX-License-Identifier: MIT
 *
 * @author Lâm Kim Bảo
 * @date   30/11/2025
 * @note   The SPDX identifier above is a compact machine-readable
 *         license marker; include full license text in a separate
 *         LICENSE file if required by your policies.
 *       compact machine-readable license marker; include full license
 *       text in a separate LICENSE file if required by your policies.
 *
 * @warning This sketch interfaces with the Arduino/ESP32 Servo API and
 *          BluetoothClassic. It has been refactored to be more
 *          MISRA-like (typed constants, fixed-width types, explicit
 *          prototypes, limited global visibility). It is not a
 *          guarantee of full MISRA-C:2012 compliance. Use a certified
 *          static analysis tool (PC-lint, QA-C, or similar) to
 *          perform formal compliance checks.
 */

#include "BluetoothSerial.h"
#include <ESP32Servo.h>
#include <stdint.h>

/* Typed Bluetooth serial object */
BluetoothSerial SerialBT;

/* Time tracking for each servo (ms) */
static uint32_t lastUpdate1 = 0u;
static uint32_t lastUpdate2 = 0u;
static uint32_t lastUpdate3 = 0u;
static uint32_t lastUpdate4 = 0u;
static uint32_t lastUpdate5 = 0u;
static uint32_t lastUpdate6 = 0u;

/* Servo pin assignments (typed constants) */
static const uint8_t SERVO1_PIN = 22u;
static const uint8_t SERVO2_PIN = 23u;
static const uint8_t SERVO3_PIN = 32u;
static const uint8_t SERVO4_PIN = 33u;
static const uint8_t SERVO5_PIN = 25u;
static const uint8_t SERVO6_PIN = 26u;

/* Command character read from Bluetooth (read-checked) */
static volatile char dieukhien = '\0';

/* Servo angles (typed, static) */
static uint8_t gocServo1 = 60u;
static uint8_t gocServo2 = 90u;
static uint8_t gocServo3 = 90u;
static uint8_t gocServo4 = 105u;
static uint8_t gocServo5 = 130u;
static uint8_t gocServo6 = 45u;

/* Timing / speed (milliseconds between steps) */
static uint32_t tocdo = 50u;

/* Servo objects (static storage) */
static Servo servo1;
static Servo servo2;
static Servo servo3;
static Servo servo4;
static Servo servo5;
static Servo servo6;

/* Modes (use true/false explicitly) */
static bool Mode1 = false;
static bool Mode2 = false;
static bool Mode3 = false;

/* Named constants for constraints and increments */
static const uint8_t SERVO_MIN = 0u;
static const uint8_t SERVO_MAX = 180u;
static const uint8_t SERVO6_MIN = 20u;
static const uint8_t SERVO6_MAX = 90u;
static const uint8_t INCR_LARGE = 5u;
static const uint8_t INCR_MEDIUM = 3u;
static const uint8_t INCR_SMALL = 2u;

/* Bluetooth device name */
static const char * const btName = "ESP32_BT_RC";

/* Forward declarations (explicit prototypes) */
/**
 * @brief Update active mode flags according to the last received
 *        command character stored in `dieukhien`.
 *
 * @pre `dieukhien` contains the most recently received command byte.
 * @post Mode1/Mode2/Mode3 reflect the requested mode state.
 */
static void setMode(void);

/**
 * @brief Apply movement commands to servos 1 and 2 based on
 *        the current `dieukhien` value and timing checks.
 *
 * @note Timing is governed by `lastUpdate1`/`lastUpdate2` and
 *       `tocdo`. Servo angle variables are clamped using
 *       `SERVO_MIN`/`SERVO_MAX`.
 */
static void robotArm1(void);

/**
 * @brief Apply movement commands to servos 3 and 4 based on
 *        the current `dieukhien` value and timing checks.
 */
static void robotArm2(void);

/**
 * @brief Apply movement commands to servos 5 and 6 based on
 *        the current `dieukhien` value and timing checks.
 */
static void robotArm3(void);

void setup(void)
{
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
  servo5.attach(SERVO5_PIN);
  servo6.attach(SERVO6_PIN);

  Serial.begin(115200);

  /* Initialize Bluetooth and check return explicitly */
  if (SerialBT.begin(btName) == false)
  {
    Serial.println("Khởi động Bluetooth thất bại!");
    for (;;) { /* wait here */ }
  }

  Serial.println("Bluetooth đã sẵn sàng!");
  Serial.print("Tên thiết bị: ");
  Serial.println(btName);
  Serial.println("Mở app Bluetooth RC Car, kết nối và gửi lệnh...");

  /* initialize servos to starting angles */
  servo1.write(gocServo1);
  servo2.write(gocServo2);
  servo3.write(gocServo3);
  servo4.write(gocServo4);
  servo5.write(gocServo5);
  servo6.write(gocServo6);
}

void loop(void)
{
  /* Read a character safely from Bluetooth if available */
  int16_t ch = -1;
  if (SerialBT.available() > 0)
  {
    ch = SerialBT.read();
    if (ch >= 0)
    {
      dieukhien = (char) ch;
      Serial.print("Nhận được: ");
      Serial.println(dieukhien);
      setMode();
      if (Mode1 == true)
      {
        robotArm1();
      }
      else if (Mode2 == true)
      {
        robotArm2();
      }
      else if (Mode3 == true)
      {
        robotArm3();
      }

      Serial.print("  servo1: "); Serial.print(gocServo1);
      Serial.print("  servo2: "); Serial.print(gocServo2);
      Serial.print("  servo3: "); Serial.print(gocServo3);
      Serial.print("  servo4: "); Serial.print(gocServo4);
      Serial.print("  servo5: "); Serial.print(gocServo5);
      Serial.print("  servo6: "); Serial.println(gocServo6);
    }
  }
}

static void setMode(void)
{
  switch (dieukhien)
  {
    case 'W':
    {
      Mode1 = true;
      break;
    }
    case 'U':
    {
      Mode2 = true;
      break;
    }
    case 'w':
    {
      Mode1 = false;
      break;
    }
    case 'u':
    {
      Mode2 = false;
      break;
    }
    case 'V':
    {
      Mode3 = true;
      break;
    }
    case 'v':
    {
      Mode3 = false;
      break;
    }
    default:
    {
      /* no mode change for other characters */
      break;
    }
  }
}

static void robotArm1(void)
{
  uint32_t now1 = millis();

  if ((dieukhien == 'F') && ((now1 - lastUpdate1) >= tocdo))
  {
    lastUpdate1 = now1;
    if (gocServo1 < SERVO_MAX)
    {
      gocServo1 = (uint8_t) (gocServo1 + INCR_LARGE);
      servo1.write(gocServo1);
    }
  }
  else if ((dieukhien == 'B') && ((now1 - lastUpdate1) >= tocdo))
  {
    lastUpdate1 = now1;
    if (gocServo1 > SERVO_MIN)
    {
      gocServo1 = (uint8_t) (gocServo1 - INCR_LARGE);
      servo1.write(gocServo1);
    }
  }

  if ((dieukhien == 'L') && ((now1 - lastUpdate2) >= tocdo))
  {
    lastUpdate2 = now1;
    if (gocServo2 < SERVO_MAX)
    {
      gocServo2 = (uint8_t) (gocServo2 + INCR_LARGE);
      servo2.write(gocServo2);
    }
  }
  else if ((dieukhien == 'R') && ((now1 - lastUpdate2) >= tocdo))
  {
    lastUpdate2 = now1;
    if (gocServo2 > SERVO_MIN)
    {
      gocServo2 = (uint8_t) (gocServo2 - INCR_LARGE);
      servo2.write(gocServo2);
    }
  }
}

static void robotArm2(void)
{
  uint32_t now2 = millis();

  if ((dieukhien == 'F') && ((now2 - lastUpdate3) >= tocdo))
  {
    lastUpdate3 = now2;
    if (gocServo3 < SERVO_MAX)
    {
      gocServo3 = (uint8_t) (gocServo3 + INCR_LARGE);
      servo3.write(gocServo3);
    }
  }
  else if ((dieukhien == 'B') && ((now2 - lastUpdate3) >= tocdo))
  {
    lastUpdate3 = now2;
    if (gocServo3 > SERVO_MIN)
    {
      gocServo3 = (uint8_t) (gocServo3 - INCR_LARGE);
      servo3.write(gocServo3);
    }
  }

  if ((dieukhien == 'L') && ((now2 - lastUpdate4) >= tocdo))
  {
    lastUpdate4 = now2;
    if (gocServo4 < SERVO_MAX)
    {
      gocServo4 = (uint8_t) (gocServo4 + INCR_LARGE);
      servo4.write(gocServo4);
    }
  }
  else if ((dieukhien == 'R') && ((now2 - lastUpdate4) >= tocdo))
  {
    lastUpdate4 = now2;
    if (gocServo4 > SERVO_MIN)
    {
      gocServo4 = (uint8_t) (gocServo4 - INCR_LARGE);
      servo4.write(gocServo4);
    }
  }
}

static void robotArm3(void)
{
  uint32_t now3 = millis();

  if ((dieukhien == 'F') && ((now3 - lastUpdate5) >= tocdo))
  {
    lastUpdate5 = now3;
    if (gocServo5 < SERVO_MAX)
    {
      gocServo5 = (uint8_t) (gocServo5 + INCR_MEDIUM);
      servo5.write(gocServo5);
    }
  }
  else if ((dieukhien == 'B') && ((now3 - lastUpdate5) >= tocdo))
  {
    lastUpdate5 = now3;
    if (gocServo5 > SERVO_MIN)
    {
      gocServo5 = (uint8_t) (gocServo5 - INCR_MEDIUM);
      servo5.write(gocServo5);
    }
  }

  if ((dieukhien == 'L') && ((now3 - lastUpdate6) >= tocdo))
  {
    lastUpdate6 = now3;
    if (gocServo6 < SERVO6_MAX)
    {
      gocServo6 = (uint8_t) (gocServo6 + INCR_SMALL);
      servo6.write(gocServo6);
    }
  }
  else if ((dieukhien == 'R') && ((now3 - lastUpdate6) >= tocdo))
  {
    lastUpdate6 = now3;
    if (gocServo6 > SERVO6_MIN)
    {
      gocServo6 = (uint8_t) (gocServo6 - INCR_SMALL);
      servo6.write(gocServo6);
    }
  }
}

