#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

int L1 = 69;
int L2 = 47;
int x_ini = -10;
int y_ini = 85;
int pos_step = 2;

void setServoPulse(uint8_t n, double pulse);
void cinematica(double x, double y, int* pos, int servo[]);
void use_servo_4( int servo[8], int final_pos[8] );
void ponta_de_pe();
void walk();

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150
#define SERVOMAX  600
#define USMIN  600
#define USMAX  2400
#define SERVO_FREQ 50

uint8_t servonum = 15;
int servo_aux[2];

int position_history[12] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };

void setup() {
  Serial.begin(9600);

  pwm.begin();

  pwm.setOscillatorFrequency(25700000);
  pwm.setPWMFreq(SERVO_FREQ);

  pwm.writeMicroseconds(0, 1500);
  pwm.writeMicroseconds(1, 1500);
  pwm.writeMicroseconds(2, 1500);
  pwm.writeMicroseconds(3, 1500);
  pwm.writeMicroseconds(4, 1500);
  pwm.writeMicroseconds(5, 1500);
  pwm.writeMicroseconds(10, 1500);
  pwm.writeMicroseconds(11, 1500);
  pwm.writeMicroseconds(12, 1500);
  pwm.writeMicroseconds(13, 1500);
  pwm.writeMicroseconds(14, 1500);
  pwm.writeMicroseconds(15, 1500);

  Serial.println("8 channel Servo test!");
  ponta_de_pe();
}

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;

  pulselength = 1000000;
  pulselength /= SERVO_FREQ;
  Serial.print(pulselength); Serial.println(" us per period");
  pulselength /= 4096;
  Serial.print(pulselength); Serial.println(" us per bit");
  pulse *= 1000000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

// ***** valores de referência *****
// centro 1500
// -90 = 500
// +90 = 2500


// x -> distancia
// y -> altura
// theta1 -> motor de cima
// theta2 -> motor de baixo
void cinematica(double x, double y, int* pos, int servo[]) 
{ 
  double theta2_rad = acos((pow(x, 2) + pow(y, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2));
  double theta1_rad = atan(y / x) - atan(L2 * sin(theta2_rad) / (L1 + L2 * cos(theta2_rad)));

  double theta1;
  double theta2;

  if (servo[0] == 1 || servo[0] == 14)
    theta1 = -1 * (90 - (theta1_rad * 180 / M_PI));
  else
    theta1 = 90 - (theta1_rad * 180 / M_PI);

  if (servo[1] == 0 || servo[1] == 15)
    theta2 = -1 * (90 - (theta2_rad * 180 / M_PI));
  else
    theta2 = 90 - (theta2_rad * 180 / M_PI);

  if (x < 0) {
    if (theta1 < 0)
      theta1 += 180;
    else
      theta1 -= 180;
  }
//  Serial.print("ANGULOS -> ");
//  Serial.print(theta1);
//  Serial.print("; ");
//  Serial.println(theta2);

  int pos_motor1 = (int)((theta1 * 100 / 9) + 1500);
  int pos_motor2 = (int)((theta2 * 100 / 9) + 1500);
//  Serial.print("POSICOES -> ");
//  Serial.print(pos_motor1);
//  Serial.print("; ");
//  Serial.println(pos_motor2);

  pos[0] = pos_motor2;
  pos[1] = pos_motor1;
}

void use_servo_4 ( int servo[8], int final_pos[8] )
{
  // Mapeia o servo na posicao do vetor de servos
  int servo_i[8];
  for(int i = 0; i < 8; i++){
    if (servo[i] > 5)
      servo_i[i] = servo[i] - 4;
    else 
      servo_i[i] = servo[i];
  }

  // Pega posicao atual do servo
  int initial_pos[8];
  for(int i = 0; i < 8; i++)
    initial_pos[i] = position_history[servo_i[i]];

  // Calcula o quanto tem que andar
  int amnt[8];
  int amnt_aux[8];
  int max_amnt = 0;
  for(int i = 0; i < 8; i++) {
    amnt[i] = final_pos[i] - initial_pos[i];
    amnt_aux[i] = abs(amnt[i]);
    if(amnt_aux[i] >= max_amnt)
      max_amnt = amnt_aux[i];
  }
  // Move o servo
  for (int i = 0; i < max_amnt; i += pos_step)
  {
    for(int j = 0; j < 8; j++)
    {
      if (amnt[j] > 0) {
        if (amnt_aux[j] > 0) {
          if(amnt_aux[j] >= pos_step) {
            pwm.writeMicroseconds(servo[j], (initial_pos[j] + i));
            amnt_aux[j] -= pos_step;
          }
          else {
            pwm.writeMicroseconds(servo[j], final_pos[j]);
            amnt_aux[j] = 0;
          }
        }
      } else if (amnt[j] < 0) {
        if (amnt_aux[j] > 0) {
          if(amnt_aux[j] >= pos_step) {
            pwm.writeMicroseconds(servo[j], (initial_pos[j] - i));
            amnt_aux[j] -= pos_step;
          }
          else {
            pwm.writeMicroseconds(servo[j], final_pos[j]);
            amnt_aux[j] = 0;
          }
        }
      }
//      Serial.print(servo[j]);
//      Serial.print(" = ");
//      Serial.println(amnt_aux[j]);
    }
  }
  for (int i = 0; i < 8; i++)
    position_history[servo_i[i]] = final_pos[i];
}

void use_servo_1 ( int servo[2], int final_pos[2] )
{
  // Mapeia o servo na posicao do vetor de servos
  int servo_i[2];
  for(int i = 0; i < 2; i++){
    if (servo[i] > 5)
      servo_i[i] = servo[i] - 4;
    else 
      servo_i[i] = servo[i];
  }

  // Pega posicao atual do servo
  int initial_pos[2];
  for(int i = 0; i < 2; i++)
    initial_pos[i] = position_history[servo_i[i]];

  // Calcula o quanto tem que andar
  int amnt[2];
  int amnt_aux[2];
  int max_amnt = 0;
  for(int i = 0; i < 2; i++) {
    amnt[i] = final_pos[i] - initial_pos[i];
    amnt_aux[i] = abs(amnt[i]);
    if(amnt_aux[i] >= max_amnt)
      max_amnt = amnt_aux[i];
  }
  // Move o servo
  for (int i = 0; i < max_amnt; i += pos_step)
  {
    for(int j = 0; j < 2; j++)
    {
      if (amnt[j] > 0) {
        if (amnt_aux[j] > 0) {
          if(amnt_aux[j] >= pos_step) {
            pwm.writeMicroseconds(servo[j], (initial_pos[j] + i));
            amnt_aux[j] -= pos_step;
          }
          else {
            pwm.writeMicroseconds(servo[j], final_pos[j]);
            amnt_aux[j] = 0;
          }
        }
      } else if (amnt[j] < 0) {
        if (amnt_aux[j] > 0) {
          if(amnt_aux[j] >= pos_step) {
            pwm.writeMicroseconds(servo[j], (initial_pos[j] - i));
            amnt_aux[j] -= pos_step;
          }
          else {
            pwm.writeMicroseconds(servo[j], final_pos[j]);
            amnt_aux[j] = 0;
          }
        }
      }
//      Serial.print(servo[j]);
//      Serial.print(" = ");
//      Serial.println(amnt_aux[j]);
    }
  }
  for (int i = 0; i < 2; i++)
    position_history[servo_i[i]] = final_pos[i];
}

void ponta_de_pe ()
{
  int servo[8];
  int pos[8];
  int servo_aux[2];
  int* pos_aux = (int*) malloc(2 * sizeof(int));

  servo[0] = 0;
  servo[1] = 1;
  servo[2] = 4;
  servo[3] = 5;
  servo[4] = 10;
  servo[5] = 11;
  servo[6] = 14;
  servo[7] = 15;

  servo_aux[0] = 1;
  servo_aux[1] = 0;
  cinematica(x_ini, y_ini, pos_aux, servo_aux); 
  pos[0] = pos_aux[0];                  // servo 0 - pata   - 0
  pos[1] = pos_aux[1];                  // servo 1 - perna  - 0

  servo_aux[0] = 4;
  servo_aux[1] = 5;
  cinematica(x_ini, y_ini, pos_aux, servo_aux);
  pos[2] = pos_aux[1];                  // servo 4 - perna  - 1
  pos[3] = pos_aux[0];                  // servo 5 - pata   - 1

  servo_aux[0] = 11;
  servo_aux[1] = 10;
  cinematica(x_ini, y_ini, pos_aux, servo_aux);
  pos[4] = pos_aux[0];                  // servo 10 - pata  - 0
  pos[5] = pos_aux[1];                  // servo 11 - perna - 0

  servo_aux[0] = 14;
  servo_aux[1] = 15;
  cinematica(x_ini, y_ini, pos_aux, servo_aux);
  pos[6] = pos_aux[1];                  // servo 14 - perna - 1
  pos[7] = pos_aux[0];                  // servo 15 - pata  - 1

  use_servo_4(servo, pos);
}

void walk() {
  int servo_4[8];
  int pos_4[8];
  int servo_aux[2];
  int* pos_aux = (int*) malloc(2 * sizeof(int));
  int servo_1[2];
  int pos_1[2];

  servo_4[0] = 0;
  servo_4[1] = 1;
  servo_4[2] = 4;
  servo_4[3] = 5;
  servo_4[4] = 10;
  servo_4[5] = 11;
  servo_4[6] = 14;
  servo_4[7] = 15;

  int x_ref = 30;
  int x_d = 0;
  int y_d = 50;

  for(int j = 0; j < 10; j++) {
    for (int i = 0; i < 4; i++) {
      if (i == 0) {
        servo_aux[0] = 1;
        servo_aux[1] = 0;
        servo_1[0] = 0;
        servo_1[1] = 1;
      } else if (i == 1) {
        servo_aux[0] = 4;
        servo_aux[1] = 5;
        servo_1[0] = 4;
        servo_1[1] = 5;
      } else if (i == 2) {
        servo_aux[0] = 11;
        servo_aux[1] = 10;
        servo_1[0] = 10;
        servo_1[1] = 11;
      } else {
        servo_aux[0] = 14;
        servo_aux[1] = 15;
        servo_1[0] = 14;
        servo_1[1] = 15;
      }
  
      if (servo_1[0] >= 10)
        x_d = -1 * x_ref;
      else
        x_d = x_ref;
      
      cinematica(x_ini, y_ini - y_d, pos_aux, servo_aux);
      if (i % 2 != 0) {
        pos_1[0] = pos_aux[1];
        pos_1[1] = pos_aux[0];
      } else {
        pos_1[0] = pos_aux[0];
        pos_1[1] = pos_aux[1];
      }
      use_servo_1(servo_1, pos_1);
      delay(10);
    
      cinematica(x_ini + x_d, y_ini - y_d, pos_aux, servo_aux); 
      if (i % 2 != 0) {
        pos_1[0] = pos_aux[1];
        pos_1[1] = pos_aux[0];
      } else {
        pos_1[0] = pos_aux[0];
        pos_1[1] = pos_aux[1];
      }
      use_servo_1(servo_1, pos_1);
      delay(10);
    
      cinematica(x_ini + x_d, y_ini, pos_aux, servo_aux);
      if (i % 2 != 0) {
        pos_1[0] = pos_aux[1];
        pos_1[1] = pos_aux[0];
      } else {
        pos_1[0] = pos_aux[0];
        pos_1[1] = pos_aux[1];
      }
      use_servo_1(servo_1, pos_1);
      delay(10);
    }
    ponta_de_pe();
    delay(10);
  }
}

void finge_de_morto() {
  int servo[2] = {2,12};
  int pos[2] = {1500 - 750,1500 + 750};
  use_servo_1(servo, pos);
  pos[0] = 1500;
  pos[1] = 1500;
  use_servo_1(servo, pos);
}

void levanta() {
  int servo[8];
  int pos[8];
  int servo_aux[2];
  int* pos_aux = (int*) malloc(2 * sizeof(int));

  servo[0] = 0;
  servo[1] = 1;
  servo[2] = 4;
  servo[3] = 5;
  servo[4] = 10;
  servo[5] = 11;
  servo[6] = 14;
  servo[7] = 15;

  servo_aux[0] = 1;
  servo_aux[1] = 0;
  cinematica(L1+L2, 0, pos_aux, servo_aux); 
  pos[0] = pos_aux[0];                  // servo 0 - pata   - 0
  pos[1] = pos_aux[1];                  // servo 1 - perna  - 0

  servo_aux[0] = 4;
  servo_aux[1] = 5;
  cinematica(L1+L2, 0, pos_aux, servo_aux);
  pos[2] = pos_aux[1];                  // servo 4 - perna  - 1
  pos[3] = pos_aux[0];                  // servo 5 - pata   - 1

  servo_aux[0] = 11;
  servo_aux[1] = 10;
  cinematica(L1+L2, 0, pos_aux, servo_aux);
  pos[4] = pos_aux[0];                  // servo 10 - pata  - 0
  pos[5] = pos_aux[1];                  // servo 11 - perna - 0

  servo_aux[0] = 14;
  servo_aux[1] = 15;
  cinematica(L1+L2, 0, pos_aux, servo_aux);
  pos[6] = pos_aux[1];                  // servo 14 - perna - 1
  pos[7] = pos_aux[0];                  // servo 15 - pata  - 1

  use_servo_4(servo, pos);

  servo_aux[0] = 2;
  servo_aux[1] = 12;
  int pos_2[2] = {1500-250,1500+250};
  use_servo_1(servo_aux,pos_2);
  servo_aux[0] = 3;
  servo_aux[1] = 13;
  pos_2[0] = 1500+250;
  pos_2[1] = 1500-250;
  use_servo_1(servo_aux,pos_2);
  pos_2[0] = 1500;
  pos_2[1] = 1500;
  use_servo_1(servo_aux,pos_2);
  servo_aux[0] = 2;
  servo_aux[1] = 12;
  use_servo_1(servo_aux,pos_2);
  
  ponta_de_pe();
}

void senta() {
  int servo[2] = {11,14};
  int pos[2] = {1500-750,1500+750};
  use_servo_1(servo,pos);

  servo[0] = 0;
  servo[1] = 1;
  pos[0] = 1500-300;
  pos[1] = 1500-150;
  use_servo_1(servo,pos);
  servo[0] = 5;
  servo[1] = 4;
  pos[0] = 1500+300;
  pos[1] = 1500+150;
  use_servo_1(servo,pos);
}

void loop() { 
  int controle;
  while (true)
  {
    if (Serial.available() > 0)
    {
      controle = Serial.parseInt();
      Serial.println(controle);
      if (controle == 0)
        ponta_de_pe();
      if (controle == 1)
        walk();
      if (controle == 2)
        senta();
      if (controle == 3)
        finge_de_morto();
      if (controle == 4)
        levanta();
    }
  }
}
