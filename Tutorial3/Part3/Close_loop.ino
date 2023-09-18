#include "Arduino.h"



#define PRESSURE_SENSOR (A1)             //  MPX5100 Series Integrated Silicon Pressure Sensor analog input (0 to 100 kPa)

//Arduino PWM Speed Control：
int E1 = 3;    ///<Pump 1 Speed
int M1 = 4;    ///<Pump 1 Direction

int E2 = 11;   ///<Valve 1 Enable
int M2 = 12;  ///<Valve  1 State

const int E3 = 5; ///<Pump 2 Speed
const int M3 = 8; ///<Pump 2 Direction

const int E4 = 6; ///<Valve 2 Enable
const int M4 = 7; ///<Valve 1 Direction

int timecounter = 1;  // Auxiliar variable for controlling the time of the process
bool lock=false;
int stateprocess=0;

//Pressure sensor calibration factors  MPX5100 Series Integrated Silicon Pressure Sensor analog input (0 to 100 kPa)  Vout=Vs(P * 0.009 + 0.04),  Vs=5V = 1024,  P = 
 
const float SensorOffset = 4.44;  //pressure sensor offset
const float SensorGain = 0.109;   // pressure sensor proportional relation

//Pressure filter
float pressure=0;
float pressure_f=0;
float pressure_a=0;
float alpha=0.2;


void setup()
{
  //Motor control shield  
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(M3, OUTPUT);
    pinMode(M4, OUTPUT);

    pinMode(PRESSURE_SENSOR, INPUT);  // Defining sensor inputs for ADC (Analog digital converter)
    
    Serial.begin(115200);             // Starting Serial communication with computer baudrate 115200 bps

//Print legend of parameters of the system

    Serial.print("Process_Status");    
    Serial.print(",");
    Serial.print("Pressure_sensor_Value");    // pressure data in kpa
    Serial.print(",");
    Serial.print("Filtered_Pressure");    // pressure data in kpa
    Serial.println(",");
}

void loop()
{
timecounter++;
int motorspeed=100;      
float Setpoint=20;

// filtering sensing pressure status
float  pressure_sensorValue = (analogRead(PRESSURE_SENSOR)*SensorGain-SensorOffset); //Do maths for calibration
pressure=pressure_sensorValue;
pressure_f=pressure_f+alpha*(pressure-pressure_a);
pressure_a=pressure_f;

//Controlling the time of the process
if (lock == false )     // Inflation process until reach 20 kPa
{
  // Introducion air to the PneuNets
     motor_1_on(250);
     valve_1_on();
     stateprocess=1;     
  }
 if ( pressure_f >= 19 )
 {
  lock=true;
  valve_1_on();
  motor_1_off();
  stateprocess=2;
  }
  if ( timecounter > 70 and timecounter <= 120 )
  {
  motor_1_off();
  valve_1_off();
  stateprocess=3;
    }
  if ( timecounter >= 120)
  {
    timecounter=0;
    lock=false;
    stateprocess=4;
    }


    //Print parameters of the system

    Serial.print(stateprocess);    // pressure data in kpa
    Serial.print(",");
    Serial.print(pressure_sensorValue);    // pressure data in kpa
    Serial.print(",");
    Serial.print(pressure_f);    // pressure data in kpa
    Serial.println(",");

    
    delay(100);   // defining sample time = 100 miliseconds

   
    
}


// Motor control functions 

void motor_1_on(int motorspeed)
{
  analogWrite(E1, motorspeed);   //PWM Speed Control   value
  digitalWrite(M1,HIGH);
  }

void motor_1_off(void)
{
  analogWrite(E1, 0);   //PWM Speed Control   value
  digitalWrite(M1,HIGH);
  }

  void valve_1_on(void)
{
  analogWrite(E2, 255);   //PWM Speed Control   value
  digitalWrite(M2,HIGH);
  }

void valve_1_off(void)
{
  analogWrite(E2, 0);   //PWM Speed Control   value
  digitalWrite(M2,HIGH);
  }

void motor_2_on(int motorspeed)
{
  analogWrite(E3, motorspeed);   //PWM Speed Control   value
  digitalWrite(M3,HIGH);
  }

void motor_2_off(void)
{
  analogWrite(E3, 0);   //PWM Speed Control   value
  digitalWrite(M3,HIGH);
  }

  void valve_2_on(void)
{
  analogWrite(E4, 255);   //PWM Speed Control   value
  digitalWrite(M4,HIGH);
  }

void valve_2_off(void)
{
  analogWrite(E4, 0);   //PWM Speed Control   value
  digitalWrite(M4,HIGH);
  }
