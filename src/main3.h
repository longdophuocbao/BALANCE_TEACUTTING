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

int speed_fuzzy = 0;
int speed_freq = 0;
bool Motor_L = true;
bool Motor_R = true;
int cc = 0;
int x_angle = 0;
int angle_set = 1;
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

//FUZZY
//***********************************************************************
#define FIS_TYPE float
#define FIS_RESOLUSION 101
#define FIS_MIN -3.4028235E+38
#define FIS_MAX 3.4028235E+38
typedef FIS_TYPE (*_FIS_MF)(FIS_TYPE, FIS_TYPE*);
typedef FIS_TYPE (*_FIS_ARR_OP)(FIS_TYPE, FIS_TYPE);
typedef FIS_TYPE (*_FIS_ARR)(FIS_TYPE*, int, _FIS_ARR_OP);
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

// Number of inputs to the fuzzy inference system
const int fis_gcI = 1;
// Number of outputs to the fuzzy inference system
const int fis_gcO = 1;
// Number of rules to the fuzzy inference system
const int fis_gcR = 5;

FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];
//FUZZY

//***********************************************************************

//FUZZY
FIS_TYPE fis_trapmf(FIS_TYPE x, FIS_TYPE* p) {
  FIS_TYPE a = p[0], b = p[1], c = p[2], d = p[3];
  FIS_TYPE t1 = ((x <= c) ? 1 : ((d < x) ? 0 : ((c != d) ? ((d - x) / (d - c)) : 0)));
  FIS_TYPE t2 = ((b <= x) ? 1 : ((x < a) ? 0 : ((a != b) ? ((x - a) / (b - a)) : 0)));
  return (FIS_TYPE)min(t1, t2);
}

// Triangular Member Function
FIS_TYPE fis_trimf(FIS_TYPE x, FIS_TYPE* p) {
  FIS_TYPE a = p[0], b = p[1], c = p[2];
  FIS_TYPE t1 = (x - a) / (b - a);
  FIS_TYPE t2 = (c - x) / (c - b);
  if ((a == b) && (b == c)) return (FIS_TYPE)(x == a);
  if (a == b) return (FIS_TYPE)(t2 * (b <= x) * (x <= c));
  if (b == c) return (FIS_TYPE)(t1 * (a <= x) * (x <= b));
  t1 = min(t1, t2);
  return (FIS_TYPE)max(t1, 0);
}

FIS_TYPE fis_prod(FIS_TYPE a, FIS_TYPE b) {
  return (a * b);
}

FIS_TYPE fis_probor(FIS_TYPE a, FIS_TYPE b) {
  return (a + b - (a * b));
}

FIS_TYPE fis_sum(FIS_TYPE a, FIS_TYPE b) {
  return (a + b);
}

FIS_TYPE fis_array_operation(FIS_TYPE* array, int size, _FIS_ARR_OP pfnOp) {
  int i;
  FIS_TYPE ret = 0;

  if (size == 0) return ret;
  if (size == 1) return array[0];

  ret = array[0];
  for (i = 1; i < size; i++) {
    ret = (*pfnOp)(ret, array[i]);
  }

  return ret;
}


//***********************************************************************
// Data for Fuzzy Inference System
//***********************************************************************
// Pointers to the implementations of member functions
_FIS_MF fis_gMF[] = {
  fis_trapmf, fis_trimf
};

// Count of member function for each Input
int fis_gIMFCount[] = { 5 };

// Count of member function for each Output
int fis_gOMFCount[] = { 3 };

// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1[] = { -100, -80, -15, -10 };
FIS_TYPE fis_gMFI0Coeff2[] = { -6, 0, 6 };
FIS_TYPE fis_gMFI0Coeff3[] = { 10, 15, 80, 100 };
FIS_TYPE fis_gMFI0Coeff4[] = { -17, -10, -5 };
FIS_TYPE fis_gMFI0Coeff5[] = { 5, 10, 17 };
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3, fis_gMFI0Coeff4, fis_gMFI0Coeff5 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff };

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1[] = { 0, 0 };
FIS_TYPE fis_gMFO0Coeff2[] = { 0, 1200 };
FIS_TYPE fis_gMFO0Coeff3[] = { 0, 1400 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff };

// Input membership function set
int fis_gMFI0[] = { 0, 1, 0, 1, 1 };
int* fis_gMFI[] = { fis_gMFI0 };

// Output membership function set

int* fis_gMFO[] = {};

// Rule Weights
FIS_TYPE fis_gRWeight[] = { 1, 1, 1, 1, 1 };

// Rule Type
int fis_gRType[] = { 1, 1, 1, 1, 1 };

// Rule Inputs
int fis_gRI0[] = { 1 };
int fis_gRI1[] = { 4 };
int fis_gRI2[] = { 2 };
int fis_gRI3[] = { 5 };
int fis_gRI4[] = { 3 };
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3, fis_gRI4 };

// Rule Outputs
int fis_gRO0[] = { 3 };
int fis_gRO1[] = { 2 };
int fis_gRO2[] = { 1 };
int fis_gRO3[] = { 2 };
int fis_gRO4[] = { 3 };
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3, fis_gRO4 };

// Input range Min
FIS_TYPE fis_gIMin[] = { -80 };

// Input range Max
FIS_TYPE fis_gIMax[] = { 80 };

// Output range Min
FIS_TYPE fis_gOMin[] = { 0 };

// Output range Max
FIS_TYPE fis_gOMax[] = { 1 };

//***********************************************************************
// Data dependent support functions for Fuzzy Inference System
//***********************************************************************
// None for Sugeno

//***********************************************************************
// Fuzzy Inference System
//***********************************************************************
void fis_evaluate() {
  FIS_TYPE fuzzyInput0[] = { 0, 0, 0, 0, 0 };
  FIS_TYPE* fuzzyInput[fis_gcI] = {
    fuzzyInput0,
  };
  FIS_TYPE fuzzyOutput0[] = { 0, 0, 0 };
  FIS_TYPE* fuzzyOutput[fis_gcO] = {
    fuzzyOutput0,
  };
  FIS_TYPE fuzzyRules[fis_gcR] = { 0 };
  FIS_TYPE fuzzyFires[fis_gcR] = { 0 };
  FIS_TYPE* fuzzyRuleSet[] = { fuzzyRules, fuzzyFires };
  FIS_TYPE sW = 0;

  // Transforming input to fuzzy Input
  int i, j, r, o;
  for (i = 0; i < fis_gcI; ++i) {
    for (j = 0; j < fis_gIMFCount[i]; ++j) {
      fuzzyInput[i][j] =
        (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]);
    }
  }

  int index = 0;
  for (r = 0; r < fis_gcR; ++r) {
    if (fis_gRType[r] == 1) {
      fuzzyFires[r] = 1;
      for (i = 0; i < fis_gcI; ++i) {
        index = fis_gRI[r][i];
        if (index > 0)
          fuzzyFires[r] = fis_prod(fuzzyFires[r], fuzzyInput[i][index - 1]);
        else if (index < 0)
          fuzzyFires[r] = fis_prod(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
        else
          fuzzyFires[r] = fis_prod(fuzzyFires[r], 1);
      }
    } else {
      fuzzyFires[r] = 0;
      for (i = 0; i < fis_gcI; ++i) {
        index = fis_gRI[r][i];
        if (index > 0)
          fuzzyFires[r] = fis_probor(fuzzyFires[r], fuzzyInput[i][index - 1]);
        else if (index < 0)
          fuzzyFires[r] = fis_probor(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
        else
          fuzzyFires[r] = fis_probor(fuzzyFires[r], 0);
      }
    }

    fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r];
    sW += fuzzyFires[r];
  }

  if (sW == 0) {
    for (o = 0; o < fis_gcO; ++o) {
      g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
    }
  } else {
    for (o = 0; o < fis_gcO; ++o) {
      FIS_TYPE sWI = 0.0;
      for (j = 0; j < fis_gOMFCount[o]; ++j) {
        fuzzyOutput[o][j] = fis_gMFOCoeff[o][j][fis_gcI];
        for (i = 0; i < fis_gcI; ++i) {
          fuzzyOutput[o][j] += g_fisInput[i] * fis_gMFOCoeff[o][j][i];
        }
      }

      for (r = 0; r < fis_gcR; ++r) {
        index = fis_gRO[r][o] - 1;
        sWI += fuzzyFires[r] * fuzzyOutput[o][index];
      }

      g_fisOutput[o] = sWI / sW;
    }
  }
}

//FUZZY

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

    //FUZZY
  g_fisInput[0] = abs(x_angle);

  //g_fisOutput[0] = 0;
  speed_fuzzy = g_fisOutput[0];
  speed_freq = map(speed_fuzzy, 0, 1400, 0, 300);
  speed_freq = 700 - speed_freq;
  // Serial.println(speed_freq);
  timerAlarmWrite(timer, speed_freq, true);
  fis_evaluate();

  //Serial.println(speed);
  //FUZZY


  if (x_angle > angle_set)
  {
    Motor_R = true;
    Motor_L = true;
    digitalWrite(MOTOR_R_DIR_PIN, LOW);
    digitalWrite(MOTOR_L_DIR_PIN, HIGH);
    L = true;
    R = false;
  }
  else if (x_angle < -angle_set)
  {
    Motor_R = true;
    Motor_L = true;
    digitalWrite(MOTOR_R_DIR_PIN, HIGH);
    digitalWrite(MOTOR_L_DIR_PIN, LOW);
    L = false;
    R = true;
  }
  else if ((x_angle > -angle_set) && (x_angle < angle_set))
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

