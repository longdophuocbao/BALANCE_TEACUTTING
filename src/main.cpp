#include <Arduino.h>
#include "esp_task_wdt.h"

#include <SimpleKalmanFilter.h>

// IMU library
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// CANBUS library
#include <SPI.h>
#include <mcp2515.h>
#include "CANbus.h"

// Define Pin
#include "definePin.h"

#include "fuzzy.h"

hw_timer_t *timer = NULL;

SimpleKalmanFilter simpleKalmanFilter(1.5, 1.5, 0.1);

// difine CAN bus
MCP2515 mcp2515(5);
struct can_frame canReceive;

// difine IMU (i2c)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
int speed_freq;
int speed_fuzzy;

float x_angle_f = 0.0;
// float x_angle = 0.0;

float angle_set = 1.2;

float *p_angle_set = &angle_set;

float x_angle = 0;
float *p_x_angle = &x_angle;

int L_pulseCount = 0;
int *ptrL_pulseCount = &L_pulseCount;

int R_pulseCount = 0;
int *ptrR_pulseCount = &R_pulseCount;

const int default_freq = 1000;

void IRAM_ATTR onTimer()
{
  // int Motor_L_Dir = digitalRead(MOTOR_L_DIR_PIN);
  // int Motor_R_Dir = digitalRead(MOTOR_R_DIR_PIN);

  // Serial.println("vvv");
  //(*ptrR_pulseCount)++;
  uint32_t gpioStatus = GPIO.in;
  if (digitalRead(MOTOR_L_STEP_PIN) == 0)
  {
    if (gpioStatus & (1 << MOTOR_L_DIR_PIN))
    {
      (*ptrL_pulseCount)++;
    }
    else
    {
      (*ptrL_pulseCount)--;
    }
  }
  if (digitalRead(MOTOR_R_STEP_PIN) == 0)
  {
    if (gpioStatus & (1 << MOTOR_R_DIR_PIN))
    {
      (*ptrR_pulseCount)++;
    }
    else
    {
      (*ptrR_pulseCount)--;
    }
  }
  timerWrite(timer, 0);
}

void SetFreqMotor(uint16_t freq)
{
  ledcChangeFrequency(L_Motor, freq, 8);
  ledcChangeFrequency(R_Motor, freq, 8);
  timerAlarmWrite(timer, 1000 / freq, true);
}

void Gotoposition(uint16_t setpoint)
{
  Serial.println("Gotoposition");
  digitalWrite(MOTOR_L_DIR_PIN, HIGH);
  digitalWrite(MOTOR_R_DIR_PIN, HIGH);
  timerAlarmEnable(timer);
  ledcWrite(L_Motor, 125);
  ledcWrite(R_Motor, 125);

  SetFreqMotor(1000);
  // delay(1000);
  int pulse_pose = setpoint * 6000;
  int *ptrpulse_pose = &pulse_pose;
  // int aa = setpoint * 40;
  while (*ptrR_pulseCount <= *ptrpulse_pose)
  {
    Serial.println(*ptrR_pulseCount);
  }
  Serial.println("Gotoposition_Completed");
  ledcWrite(L_Motor, 0);
  ledcWrite(R_Motor, 0);
}

void GoHOME()
{
  ledcWrite(ESC, ESC_min);
  ledcChangeFrequency(L_Motor, default_freq, 8);
  ledcChangeFrequency(R_Motor, default_freq, 8);
  digitalWrite(MOTOR_L_DIR_PIN, LOW);
  digitalWrite(MOTOR_R_DIR_PIN, LOW);

  ledcWrite(R_Motor, 125);
  ledcWrite(L_Motor, 125);
  while (1)
  {
    esp_task_wdt_reset();
    if (digitalRead(R_SIGNAL) == LOW)
    {
      ledcWrite(R_Motor, 0);
      Serial.println("R Motor Stopped");
    }
    if (digitalRead(L_SIGNAL) == LOW)
    {
      ledcWrite(L_Motor, 0);
      Serial.println("L Motor Stopped");
    }

    if ((digitalRead(R_SIGNAL) == LOW) && (digitalRead(L_SIGNAL) == LOW))
    {
      *ptrL_pulseCount = 0;
      *ptrR_pulseCount = 0;
      Serial.println("GotoHome_Completed");
      break; // Thoát khỏi vòng lặp khi cả hai động cơ đã dừng
    }
  }
}

void readMPU6050AndCalculateFuzzy(void *parameter)
{
  delay(500);
  Serial.println("readMPU6050AndCalculateFuzzy_CORE");
  while (true)
  {
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    *p_x_angle = orientationData.orientation.pitch;
    *p_x_angle = simpleKalmanFilter.updateEstimate(*p_x_angle);
    Serial.println(*p_x_angle);
    uint8_t system, gyro, accel, mag = 0;
    // bno.getCalibration(&system, &gyro, &accel, &mag);
    // Serial.println();
    // Serial.print("Calibration: Sys=");
    // Serial.print(system);
    // Serial.print(" Gyro=");
    // Serial.print(gyro);
    // Serial.print(" Accel=");
    // Serial.print(accel);
    // Serial.print(" Mag=");
    // Serial.println(mag);
    

    //vTaskDelay(10 / portTICK_PERIOD_MS); // Add a small delay to yield control to other tasks
  }
}
void controlMotors(void *parameter)
{
  // delay(500);
  //  timerAlarmEnable(timer);
  //  Serial.println(timerAlarmEnabled(timer));
  //  timerAlarmEnable(timer);
  Serial.println("controlMotors_CORE");
  bool BalanceRun = false;
  while (1)
  {
    if (mcp2515.readMessage(&canReceive) == MCP2515::ERROR_OK)
    {
      Serial.print("CanID: 0x");
      Serial.println(canReceive.can_id, HEX);
      if (canReceive.can_id == CanID)
      {
        int state = canReceive.data[0];
        uint16_t height_point = 0;
        uint16_t Cutting_Speed = 0;
        switch (state)
        {
        case Gohome_cm:
          Serial.println("Gohome_cm");
          BalanceRun = false;
          GoHOME();
          state = 0;
          break;

        case ControlBalance_cm:
          Serial.println("ControlBalance_cm");
          if (canReceive.data[1] = 0xFF)
            BalanceRun = true;
          else if (canReceive.data[1] = 0)
            BalanceRun = false;

          ledcWrite(ESC, 86);
          state = 0;
          break;

        case CalibIMU_cm:
          ledcWrite(ESC, ESC_min);
          ledcWrite(R_Motor, 0);
          ledcWrite(L_Motor, 0);
          BalanceRun = false;
          Serial.println("CalibIMU_cm");
          // mpu6050.calcGyroOffsets(true);
          delay(500);
          state = 0;
          break;

        case ControlCuting_cm:
          Serial.println("ControlCuting_cm");
          Cutting_Speed = (uint8_t)canReceive.data[1] + ((uint8_t)(canReceive.data[2]) << 8);
          Cutting_Speed = constrain(Cutting_Speed, ESC_min, ESC_max);
          ledcWrite(ESC, Cutting_Speed);
          state = 0;
          break;

        // Gotoposition
        case GoPosition_cm:
          Serial.println("GoPosition_cm");
          BalanceRun = false;
          height_point = (uint8_t)canReceive.data[1] + ((uint8_t)(canReceive.data[2]) << 8);
          height_point = constrain(height_point, 0, 450);

          Serial.println(height_point);
          GoHOME();

          // delay(500);
          Gotoposition(height_point);

          state = 0;
          break;

          // case ControlMotor_cm:
          //   // Serial.println("GoPosition_cm");
          //   if (canCommand.data[1] = 0xFF)
          //     Motor_R = true;
          //   else if (canCommand.data[1] = 0)
          //     Motor_R = false;

          //   if (canCommand.data[5] = 0xFF)
          //     Motor_L = true;
          //   else if (canCommand.data[2] = 0)
          //     Motor_L = false;

          //   break;

        default:
          Serial.println("ReceivedCommendWrong!");
          state = 0;
          break;
        }
      }
    }
    if (BalanceRun == true)
    {
      g_fisInput[0] = abs(*p_x_angle);
      fis_evaluate();
      speed_fuzzy = g_fisOutput[0];
      speed_freq = map(speed_fuzzy, 0, 1400, 0, 300);
      speed_freq = 700 + speed_freq;
      // timerAlarmWrite(timer, speed_freq, true);
      Serial.print(*p_x_angle);
      Serial.print("    ");
      Serial.println(speed_freq);
      SetFreqMotor(speed_freq);
      if ((*p_x_angle) >= (*p_angle_set))
      {
        ledcWrite(R_Motor, 125);
        ledcWrite(L_Motor, 125);
        digitalWrite(MOTOR_R_DIR_PIN, LOW);
        digitalWrite(MOTOR_L_DIR_PIN, HIGH);
      }
      else if ((*p_x_angle) <= -(*p_angle_set))
      {
        ledcWrite(R_Motor, 125);
        ledcWrite(L_Motor, 125);
        digitalWrite(MOTOR_R_DIR_PIN, HIGH);
        digitalWrite(MOTOR_L_DIR_PIN, LOW);
      }
      else if ((*p_x_angle > -(*p_angle_set)) && (*p_x_angle < (*p_angle_set)))
      {
        ledcWrite(R_Motor, 0);
        ledcWrite(L_Motor, 0);
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
void setup()
{
  setCpuFrequencyMhz(240);
  Serial.begin(115200);

  

  // Setup CANBUS
  mcp2515.reset();
  mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  // Setup PWM for ESC
  Serial.println("SETUP_ESC");
  ledcSetup(ESC, 50, 8);
  ledcAttachPin(ESC_PIN, ESC);
  ledcChangeFrequency(ESC, 50, 8);

  ledcWrite(ESC, ESC_max);
  delay(1000);
  ledcWrite(ESC, ESC_min);
  delay(1000);

  Serial.println("SETUP_ESC_Completed");

  // Setup PWM for Step motor
  // Serial.println("SETUP_STEP");
  ledcSetup(L_Motor, default_freq, 8);
  ledcSetup(R_Motor, default_freq, 8);
  ledcAttachPin(MOTOR_L_STEP_PIN, L_Motor);
  ledcAttachPin(MOTOR_R_STEP_PIN, R_Motor);
  ledcChangeFrequency(L_Motor, default_freq, 8);
  ledcChangeFrequency(R_Motor, default_freq, 8);
  // Serial.println("SETUP_STEP_Completed");

  // Cấu hình Timer 1 để tạo ngắt mỗi khi phát ra một xung
  timer = timerBegin(1, 240, true);                  // Timer 1, prescaler 80 (1MHz clock)
  timerAttachInterrupt(timer, &onTimer, true);       // Gán hàm ngắt
  timerAlarmWrite(timer, 1000 / default_freq, true); // Tạo ngắt mỗi xung PWM
  //  timerAlarmEnable(timer);                           // Kích hoạt ngắt

  // Setup PIN mode
  pinMode(R_SIGNAL, INPUT);
  pinMode(L_SIGNAL, INPUT);

  // pinMode(MOTOR_R_STEP_PIN, OUTPUT);
  // pinMode(MOTOR_L_STEP_PIN, OUTPUT);
  pinMode(MOTOR_L_DIR_PIN, OUTPUT);
  pinMode(MOTOR_R_DIR_PIN, OUTPUT);

  // Setup IMU
  
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  bno.setMode(OPERATION_MODE_IMUPLUS);
  // Create tasks for each core
  Serial.println("SETUP_Completed");
  delay(100);
  xTaskCreatePinnedToCore(
      readMPU6050AndCalculateFuzzy, // Function to implement the task
      "MPU6050 and Fuzzy Task",     // Name of the task
      10000,                        // Stack size in words
      NULL,                         // Task input parameter
      1,                            // Priority of the task
      NULL,                         // Task handle
      0                             // Core where the task should run
  );
  delay(100);
  xTaskCreatePinnedToCore(
      controlMotors,        // Function to implement the task
      "Motor Control Task", // Name of the task
      10000,                // Stack size in words
      NULL,                 // Task input parameter
      1,                    // Priority of the task
      NULL,                 // Task handle
      1                     // Core where the task should run
  );
}
void loop()
{
}