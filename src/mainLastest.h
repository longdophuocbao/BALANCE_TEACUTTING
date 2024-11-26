#include <Arduino.h>

#include <SimpleKalmanFilter.h>

#include <MPU6050_tockn.h>
#include "soc/rtc_wdt.h"
#include "esp_task_wdt.h"

// CANBUS library
#include <SPI.h>
#include <mcp2515.h>


// define canbus frame

#define CanID 0x20
#define Gohome_cm 0xA0
#define GoPosition_cm 0xA1
#define ControlCuting_cm 0xA2
#define ControlBalance_cm 0xA3
#define ControlMotor_cm 0xA4
#define CalibIMU_cm 0xA5

#define ESCpin 14
struct can_frame canCommand;
struct can_frame canReply;
MCP2515 mcp2515(5);

hw_timer_t *timer = NULL;

#define MOTOR_L_STEP_PIN 26 // 26-5
#define MOTOR_L_DIR_PIN 25  // 25-23
#define MOTOR_R_STEP_PIN 33 // 33-18
#define MOTOR_R_DIR_PIN 32  // 32-19

const int R_SIGNAL = 15; // Pin connected to the switch
const int L_SIGNAL = 4;  // Pin connected to the switch
//const int ledPin = 13;   // Pin của LED

MPU6050 mpu6050(Wire);

SimpleKalmanFilter simpleKalmanFilter(1.5, 1.5, 0.1);

int speed_fuzzy = 0;
int speed_freq = 0;
bool Motor_L = false;
bool Motor_R = false;
int cc = 0;
float x_angle_f = 0.0;
float x_angle = 0.0;
float angle_set = 1.2;
bool L = false;
bool R = false;
// volatile int i = 0;
long Xung_L = 0;
long Xung_R = 0;

bool *pMotor_L = &Motor_L;
bool *pMotor_R = &Motor_R;

long *pXung_L = &Xung_L;
long *pXung_R = &Xung_R;

bool *pL = &L;
bool *pR = &R;

// FUZZY BEGIN
// ####################################################################
#define FIS_TYPE float
#define FIS_RESOLUSION 101
#define FIS_MIN -3.4028235E+38
#define FIS_MAX 3.4028235E+38
typedef FIS_TYPE (*_FIS_MF)(FIS_TYPE, FIS_TYPE *);
typedef FIS_TYPE (*_FIS_ARR_OP)(FIS_TYPE, FIS_TYPE);
typedef FIS_TYPE (*_FIS_ARR)(FIS_TYPE *, int, _FIS_ARR_OP);
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
// FUZZY
//*********************************************************************
FIS_TYPE fis_trapmf(FIS_TYPE x, FIS_TYPE *p)
{
  FIS_TYPE a = p[0], b = p[1], c = p[2], d = p[3];
  FIS_TYPE t1 = ((x <= c) ? 1 : ((d < x) ? 0 : ((c != d) ? ((d - x) / (d - c)) : 0)));
  FIS_TYPE t2 = ((b <= x) ? 1 : ((x < a) ? 0 : ((a != b) ? ((x - a) / (b - a)) : 0)));
  return (FIS_TYPE)min(t1, t2);
}

// Triangular Member Function
FIS_TYPE fis_trimf(FIS_TYPE x, FIS_TYPE *p)
{
  FIS_TYPE a = p[0], b = p[1], c = p[2];
  FIS_TYPE t1 = (x - a) / (b - a);
  FIS_TYPE t2 = (c - x) / (c - b);
  if ((a == b) && (b == c))
    return (FIS_TYPE)(x == a);
  if (a == b)
    return (FIS_TYPE)(t2 * (b <= x) * (x <= c));
  if (b == c)
    return (FIS_TYPE)(t1 * (a <= x) * (x <= b));
  t1 = min(t1, t2);
  return (FIS_TYPE)max(t1, 0);
}

FIS_TYPE fis_prod(FIS_TYPE a, FIS_TYPE b)
{
  return (a * b);
}

FIS_TYPE fis_probor(FIS_TYPE a, FIS_TYPE b)
{
  return (a + b - (a * b));
}

FIS_TYPE fis_sum(FIS_TYPE a, FIS_TYPE b)
{
  return (a + b);
}

FIS_TYPE fis_array_operation(FIS_TYPE *array, int size, _FIS_ARR_OP pfnOp)
{
  int i;
  FIS_TYPE ret = 0;

  if (size == 0)
    return ret;
  if (size == 1)
    return array[0];

  ret = array[0];
  for (i = 1; i < size; i++)
  {
    ret = (*pfnOp)(ret, array[i]);
  }

  return ret;
}

//***********************************************************************
// Data for Fuzzy Inference System
//***********************************************************************
// Pointers to the implementations of member functions
_FIS_MF fis_gMF[] = {
    fis_trapmf, fis_trimf};

// Count of member function for each Input
int fis_gIMFCount[] = {5};

// Count of member function for each Output
int fis_gOMFCount[] = {3};

// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1[] = {-100, -80, -15, -10};
FIS_TYPE fis_gMFI0Coeff2[] = {-6, 0, 6};
FIS_TYPE fis_gMFI0Coeff3[] = {10, 15, 80, 100};
FIS_TYPE fis_gMFI0Coeff4[] = {-17, -10, -5};
FIS_TYPE fis_gMFI0Coeff5[] = {5, 10, 17};
FIS_TYPE *fis_gMFI0Coeff[] = {fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3, fis_gMFI0Coeff4, fis_gMFI0Coeff5};
FIS_TYPE **fis_gMFICoeff[] = {fis_gMFI0Coeff};

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1[] = {0, 0};
FIS_TYPE fis_gMFO0Coeff2[] = {0, 1200};
FIS_TYPE fis_gMFO0Coeff3[] = {0, 1400};
FIS_TYPE *fis_gMFO0Coeff[] = {fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3};
FIS_TYPE **fis_gMFOCoeff[] = {fis_gMFO0Coeff};

// Input membership function set
int fis_gMFI0[] = {0, 1, 0, 1, 1};
int *fis_gMFI[] = {fis_gMFI0};

// Output membership function set

int *fis_gMFO[] = {};

// Rule Weights
FIS_TYPE fis_gRWeight[] = {1, 1, 1, 1, 1};

// Rule Type
int fis_gRType[] = {1, 1, 1, 1, 1};

// Rule Inputs
int fis_gRI0[] = {1};
int fis_gRI1[] = {4};
int fis_gRI2[] = {2};
int fis_gRI3[] = {5};
int fis_gRI4[] = {3};
int *fis_gRI[] = {fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3, fis_gRI4};

// Rule Outputs
int fis_gRO0[] = {3};
int fis_gRO1[] = {2};
int fis_gRO2[] = {1};
int fis_gRO3[] = {2};
int fis_gRO4[] = {3};
int *fis_gRO[] = {fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3, fis_gRO4};

// Input range Min
FIS_TYPE fis_gIMin[] = {-80};

// Input range Max
FIS_TYPE fis_gIMax[] = {80};

// Output range Min
FIS_TYPE fis_gOMin[] = {0};

// Output range Max
FIS_TYPE fis_gOMax[] = {1};

//***********************************************************************
// Data dependent support functions for Fuzzy Inference System
//***********************************************************************
// None for Sugeno
//***********************************************************************
// Fuzzy Inference System
//***********************************************************************
void fis_evaluate()
{
  FIS_TYPE fuzzyInput0[] = {0, 0, 0, 0, 0};
  FIS_TYPE *fuzzyInput[fis_gcI] = {
      fuzzyInput0,
  };
  FIS_TYPE fuzzyOutput0[] = {0, 0, 0};
  FIS_TYPE *fuzzyOutput[fis_gcO] = {
      fuzzyOutput0,
  };
  FIS_TYPE fuzzyRules[fis_gcR] = {0};
  FIS_TYPE fuzzyFires[fis_gcR] = {0};
  FIS_TYPE *fuzzyRuleSet[] = {fuzzyRules, fuzzyFires};
  FIS_TYPE sW = 0;

  // Transforming input to fuzzy Input
  int i, j, r, o;
  for (i = 0; i < fis_gcI; ++i)
  {
    for (j = 0; j < fis_gIMFCount[i]; ++j)
    {
      fuzzyInput[i][j] =
          (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]);
    }
  }

  int index = 0;
  for (r = 0; r < fis_gcR; ++r)
  {
    if (fis_gRType[r] == 1)
    {
      fuzzyFires[r] = 1;
      for (i = 0; i < fis_gcI; ++i)
      {
        index = fis_gRI[r][i];
        if (index > 0)
          fuzzyFires[r] = fis_prod(fuzzyFires[r], fuzzyInput[i][index - 1]);
        else if (index < 0)
          fuzzyFires[r] = fis_prod(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
        else
          fuzzyFires[r] = fis_prod(fuzzyFires[r], 1);
      }
    }
    else
    {
      fuzzyFires[r] = 0;
      for (i = 0; i < fis_gcI; ++i)
      {
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

  if (sW == 0)
  {
    for (o = 0; o < fis_gcO; ++o)
    {
      g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
    }
  }
  else
  {
    for (o = 0; o < fis_gcO; ++o)
    {
      FIS_TYPE sWI = 0.0;
      for (j = 0; j < fis_gOMFCount[o]; ++j)
      {
        fuzzyOutput[o][j] = fis_gMFOCoeff[o][j][fis_gcI];
        for (i = 0; i < fis_gcI; ++i)
        {
          fuzzyOutput[o][j] += g_fisInput[i] * fis_gMFOCoeff[o][j][i];
        }
      }

      for (r = 0; r < fis_gcR; ++r)
      {
        index = fis_gRO[r][o] - 1;
        sWI += fuzzyFires[r] * fuzzyOutput[o][index];
      }

      g_fisOutput[o] = sWI / sW;
    }
  }
}
// ######################################################################
//  FUZZY END

void setCANreplyframe(uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7)
{
  canReply.can_id = CanID;
  canReply.can_dlc = 8;
  canReply.data[0] = data0;
  canReply.data[1] = data1;
  canReply.data[2] = data2;
  canReply.data[3] = data3;
  canReply.data[4] = data4;
  canReply.data[5] = data5;
  canReply.data[6] = data6;
  canReply.data[7] = data7;
}

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

void Gotoposition(uint16_t setpoint)
{
  Serial.println("Gotoposition");
  digitalWrite(MOTOR_L_DIR_PIN, HIGH);
  digitalWrite(MOTOR_R_DIR_PIN, HIGH);
  Motor_R = true;
  Motor_L = true;
  *pR = true;
  delay(1000);
  uint16_t pulse_pose = setpoint*40;
      // int aa = setpoint * 40;
  while (Xung_R <= pulse_pose)
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
  timerAlarmWrite(timer, 600, true);
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
  Serial.
}


int initt = 0;
int cnt = 0;
bool innit = true;

void readMPU6050AndCalculateFuzzy(void *parameter)
{
  while (true)
  {
    //ledcWrite(0, 17);
    mpu6050.update();
    x_angle_f = mpu6050.getAngleX();
    x_angle = simpleKalmanFilter.updateEstimate(x_angle_f);

    vTaskDelay(10 / portTICK_PERIOD_MS); // Add a small delay to yield control to other tasks
  }
}

void controlMotors(void *parameter)
{
  // Motor control loop
  Serial.println("run");
  
  // Motor_R = true;
  // Motor_L = true;
  bool BalanceRun = false;
  bool sta =false;
  while (true)
  {
    // Serial.println("AAA");
    //  This loop is now handled within the timer interrupt.
    if (mcp2515.readMessage(&canCommand) == MCP2515::ERROR_OK) 
    {
      delay(200);
      sta =~sta;
      Serial.println("Received!");
      if ((canCommand.can_id == CanID)&&(sta == true))
        {
          int state = canCommand.data[0];
          uint16_t height_point = 0;
          uint16_t Cutting_Speed = 0; 
          switch (state)
          {
          // goto home
          case Gohome_cm:
            Serial.println("Gohome_cm");
            GoHOME();
            state = 0;
            break;

          case ControlBalance_cm:
            Motor_R = false;
            Motor_L = false;
            BalanceRun = false;
            Serial.println("ControlBalance_cm");
            if (canCommand.data[1] = 0xFF)
              BalanceRun = true;
            else if (canCommand.data[1] = 0)
              BalanceRun = false;

            state = 0;
            break;

          case CalibIMU_cm:
            Motor_R = false;
            Motor_L = false;
            BalanceRun = false;
            Serial.println("CalibIMU_cm");
            mpu6050.calcGyroOffsets(true);
            mcp2515.reset();
            mcp2515.setBitrate(CAN_500KBPS);
            mcp2515.setNormalMode();
            delay(500);

            state = 0;

            break;

          case ControlCuting_cm:
            Serial.println("ControlCuting_cm");
            Cutting_Speed = (uint8_t)canCommand.data[1] + ((uint8_t)(canCommand.data[2]) << 8);
            Cutting_Speed = constrain(Cutting_Speed, 1000, 2000);
            state = 0;
            // esc.writeMicroseconds(Cutting_Speed);
            break;

          // Gotoposition
          case GoPosition_cm:

            Serial.println("GoPosition_cm");
            Motor_R = false;
            Motor_L = false;
            BalanceRun = false;
            height_point = (uint8_t)canCommand.data[1] + ((uint8_t)(canCommand.data[2]) << 8);
            height_point = constrain(height_point, 0, 450);

            Serial.println(height_point);
            GoHOME();

            delay(500);
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
            Serial.println();
            state = 0;
            break;
          }
        
      }
      mcp2515.reset();
      mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
      mcp2515.setNormalMode();
      //mcp2515.clearMERR();
    }
    if (BalanceRun == true)
    {
      g_fisInput[0] = abs(x_angle);
      fis_evaluate();
      speed_fuzzy = g_fisOutput[0];
      speed_freq = map(speed_fuzzy, 0, 1400, 0, 300);
      speed_freq = 700 - speed_freq;
      timerAlarmWrite(timer, speed_freq, true);
      if (x_angle >= angle_set)
      {
        Motor_R = true;
        Motor_L = true;
        digitalWrite(MOTOR_R_DIR_PIN, LOW);
        digitalWrite(MOTOR_L_DIR_PIN, HIGH);
        L = true;
        R = false;
      }
      else if (x_angle <= -angle_set)
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
    }
    // Serial.println(speed_freq);
    // Serial.print(x_angle_f,3);
    // Serial.print("\t");
    // Serial.print(x_angle,3);
    // Serial.print("\t");
    // Serial.print(Xung_R / 40);
    // Serial.print("\t");
    // Serial.println(Xung_L / 40);
    // vTaskDelay(10 / portTICK_PERIOD_MS); // Add a small delay to yield control to other tasks
  }
}

void setup()
{
  Serial.begin(115200);
  setCpuFrequencyMhz(240);
  rtc_wdt_protect_off();
  rtc_wdt_disable();
  rtc_wdt_protect_on();

  // Setup CANBUS
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  setCANreplyframe(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);

  
  // Setup ESC
  ledcSetup(0, 50, 8);
  
  // ledcAttachChannel(ESCpin, 50, 8, 0);
  //ledcAttachPin(ESCpin,0);
  
  // ledcChangeFrequency(0,50,8);
  // delay(1000);
  // ledcWrite(0, 23);
  // delay(5000);
  // ledcWrite(0, 15);
  // delay(5000);

  // myservo.setPeriodHertz(50);// Standard 50hz servo
  // myservo.attach(servoPin, 500, 2400);
  

  pinMode(R_SIGNAL, INPUT_PULLUP);
  pinMode(L_SIGNAL, INPUT_PULLUP);

  pinMode(MOTOR_R_STEP_PIN, OUTPUT);
  pinMode(MOTOR_L_STEP_PIN, OUTPUT);
  pinMode(MOTOR_L_DIR_PIN, OUTPUT);
  pinMode(MOTOR_R_DIR_PIN, OUTPUT);

  Wire.begin();
  mpu6050.begin();

  // Attach onTimer function to our timer.
  timer = timerBegin(3, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 600, true);
  timerAlarmEnable(timer);
  Motor_R = false;
  Motor_L = false;

  // GoHOME();

  // delay(500);

  // Gotoposition(100);
  // delay(500);

  // mpu6050.calcGyroOffsets(true);

  // mcp2515.reset();
  // mcp2515.setBitrate(CAN_500KBPS);
  // mcp2515.setNormalMode();

  // Create tasks for each core
  xTaskCreatePinnedToCore(
      readMPU6050AndCalculateFuzzy, // Function to implement the task
      "MPU6050 and Fuzzy Task",     // Name of the task
      10000,                        // Stack size in words
      NULL,                         // Task input parameter
      1,                            // Priority of the task
      NULL,                         // Task handle
      0                             // Core where the task should run
  );

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
  // Empty loop as all tasks are handled by individual cores
}
