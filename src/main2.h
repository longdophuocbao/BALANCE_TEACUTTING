#include <Arduino.h>

#include <MPU6050_tockn.h>
#include "soc/rtc_wdt.h"
#include "esp_task_wdt.h"
hw_timer_t *timer = NULL;

#define MOTOR_L_STEP_PIN 5
#define MOTOR_L_DIR_PIN 23
#define MOTOR_R_STEP_PIN 18
#define MOTOR_R_DIR_PIN 19

const int R_SIGNAL = 15; // Pin connected to the switch
const int L_SIGNAL = 4;  // Pin connected to the switch
const int ledPin = 13;   // Pin của LED

MPU6050 mpu6050(Wire);

bool Motor_L = true;
bool Motor_R = true;
int cc = 0;
int x_angle = 0;
int ff = 350; // min 333
bool L = false;
bool R = false;
// volatile int i = 0;
int Xung_L = 0;
int Xung_R = 0;

bool *pMotor_L = &Motor_L;
bool *pMotor_R = &Motor_R;

int *pXung_L = &Xung_L;
int *pXung_R = &Xung_R;

bool *pL = &L;
bool *pR = &R;

void IRAM_ATTR onTimer()
{ // Defining Inerrupt function with IRAM_ATTR for faster access
  // Serial.println(*pMotor_R);
  static int i = 0;
  if (i == 0)
  {
    if (*pMotor_L)
    {
      digitalWrite(MOTOR_L_STEP_PIN, HIGH);
      if (*pL == true)
        *pXung_L += 1;
      else
        *pXung_L -= 1;
    }
    if (*pMotor_R)
    {
      digitalWrite(MOTOR_R_STEP_PIN, HIGH);
      if (*pR == true)
        *pXung_R += 1;
      else
        *pXung_R -= 1;
    }
    i = 1;
  }
  else
  {
    if (*pMotor_L)
    {
      digitalWrite(MOTOR_L_STEP_PIN, LOW);
    }
    if (*pMotor_R)
    {
      digitalWrite(MOTOR_R_STEP_PIN, LOW);
    }
    i = 0;
  }
}

void setspeed(int _speed)
{

  // Set timer frequency to 1Mhz
  // timer = timerBegin(_speed);
  // timerAlarm(hw_timer_t *timer, uint64_t alarm_value, bool autoreload, uint64_t reload_count)
}

void Gotoposition(int setpoint)
{
  Serial.println("Gotoposition");
  digitalWrite(MOTOR_L_DIR_PIN, HIGH);
  digitalWrite(MOTOR_R_DIR_PIN, HIGH);
  Motor_R = true;
  Motor_L = true;
  *pR = true;
  delay(1000);
  // int aa = setpoint * 40;
  while (Xung_R <= 4000)
  {
    // Serial.println(*pXung_R);
    delay(20);
  }
  Serial.println(Xung_R);
  Serial.println("Gotoposition_Completed");
  Motor_R = false;
  Motor_L = false;
}

void GoHOME()
{
  Serial.println("GotoHome");
  digitalWrite(MOTOR_L_DIR_PIN, LOW);
  digitalWrite(MOTOR_R_DIR_PIN, LOW);
  Motor_R = true;
  Motor_L = true;
  while (1)
  {
    if (digitalRead(R_SIGNAL) == LOW)
    {
      Motor_R = false;
      *pXung_R = 0;
    }
    if (digitalRead(L_SIGNAL) == LOW)
    {
      Motor_L = false;
      *pXung_L = 0;
    }
    if ((Motor_R == false) && (Motor_L == false))
    {
      break;
    }
  }
  *pL = true;
  *pR = true;
  Serial.println("GotoHome_Completed");
}

void setup()
{
  Serial.begin(115200);
  setCpuFrequencyMhz(240);
  rtc_wdt_protect_off();       // Tắt bảo vệ ghi của WDT
  rtc_wdt_disable();           // Tắt WDT
  rtc_wdt_protect_on();        // Bật lại bảo vệ ghi (nếu cần)
                               // Tắt Task Watchdog Timer
  // esp_task_wdt_init(0, false); // Tham số thứ hai là `false` để không kích hoạt Task Watchdog Timer

  // // Tắt Task Watchdog Timer cho tác vụ hiện tại
  // esp_task_wdt_delete(NULL);

  // rtc_wdt_set_length_of_reset_signal(RTC_WDT_SYS_RESET_SIG, RTC_WDT_LENGTH_3_2us);
  // rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_SYSTEM);
  // rtc_wdt_set_time(RTC_WDT_STAGE0, 500000);
  pinMode(R_SIGNAL, INPUT_PULLUP); // Set the button pin as input with an internal pull-up resistor
  pinMode(L_SIGNAL, INPUT_PULLUP); // Set the button pin as input with an internal pull-up resistor

  pinMode(MOTOR_R_STEP_PIN, OUTPUT); // Thiết lập chế độ OUTPUT cho LED
  pinMode(MOTOR_L_STEP_PIN, OUTPUT);

  pinMode(MOTOR_L_DIR_PIN, OUTPUT);
  pinMode(MOTOR_R_DIR_PIN, OUTPUT);

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  // Attach onTimer function to our timer.
  timer = timerBegin(2, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 500, true);
  timerAlarmEnable(timer);
}

int initt = 0;
int cnt = 0;
bool innit = true;
void loop()
{
  //
  // delay(500);
  if (innit == true)
  {
    Serial.println("run");
    innit = false;
    GoHOME();

    delay(20);
    Gotoposition(50);

    Motor_R = true;
    Motor_L = true;
  }

  mpu6050.update();
  x_angle = mpu6050.getAngleX();
  // Khởi tạo timer với chu kỳ 1 giây (1000000 microseconds
  if (x_angle > 2)
  {
    Motor_R = true;
    Motor_L = true;
    digitalWrite(MOTOR_R_DIR_PIN, LOW);
    digitalWrite(MOTOR_L_DIR_PIN, HIGH);
    L = true;
    R = false;
  }
  else if (x_angle < -2)
  {
    Motor_R = true;
    Motor_L = true;
    digitalWrite(MOTOR_R_DIR_PIN, HIGH);
    digitalWrite(MOTOR_L_DIR_PIN, LOW);
    L = false;
    R = true;
  }
  else if ((x_angle > -2) && (x_angle < 2))
  {
    Motor_R = false;
    Motor_L = false;
  }
  Serial.print(x_angle);
  Serial.print("\t");
  Serial.print(Xung_R / 40);
  Serial.print("\t");
  Serial.println(Xung_L / 40);

  
}
