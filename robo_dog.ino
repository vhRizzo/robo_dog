#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

int L1 = 69;
int L2 = 47;
int x_ini = -10;
int y_ini = 85;

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

int position_history[12] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1400, 1500, 1500, 1500 };

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
  pwm.writeMicroseconds(12, 1400);
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
  int pos_step = 1;
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
  
  int shift = 15;
  
  for (int i = 0; i < shift; i++) {
    servo_aux[0] = 1;
    servo_aux[1] = 0;
    cinematica(x_ini + shift, y_ini + shift, pos_aux, servo_aux); 
    pos[0] = pos_aux[0];                  // servo 0 - pata   - 0
    pos[1] = pos_aux[1];                  // servo 1 - perna  - 0
  
    servo_aux[0] = 4;
    servo_aux[1] = 5;
    cinematica(x_ini - shift, y_ini - shift, pos_aux, servo_aux);
    pos[2] = pos_aux[1];                  // servo 4 - perna  - 1
    pos[3] = pos_aux[0];                  // servo 5 - pata   - 1
  
    servo_aux[0] = 11;
    servo_aux[1] = 10;
    cinematica(x_ini - shift, y_ini - shift, pos_aux, servo_aux);
    pos[4] = pos_aux[0];                  // servo 10 - pata  - 0
    pos[5] = pos_aux[1];                  // servo 11 - perna - 0
  
    servo_aux[0] = 14;
    servo_aux[1] = 15;
    cinematica(x_ini + shift, y_ini + shift, pos_aux, servo_aux);
    pos[6] = pos_aux[1];                  // servo 14 - perna - 1
    pos[7] = pos_aux[0];                  // servo 15 - pata  - 1
  
    use_servo_4(servo, pos);
  }
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
        walk();
    }
  }
}
