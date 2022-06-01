#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// L1 = 6.9 cm
// L2 = 4.7 cm

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150
#define SERVOMAX  600
#define USMIN  600
#define USMAX  2400
#define SERVO_FREQ 50

uint8_t servonum = 15;
int servo_aux[2];

int position_history[12] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };
typedef struct {
  int servo;
  int final_pos;
} servo_t;

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();

  pwm.setOscillatorFrequency(25200000);
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
  
  delay(10);
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
void cinematica(double x, double y, int* pos) {
  // int *pos_motor1, int *pos_motor2
  
  //float gamma = atan(x/y);
  //float h = sqrt((x*x)+(y*y));
  //float beta = acos((h/2)/60);
  //float alpha = asin((h/2)/60);
  
  //float theta1_deg = 10*(gamma + beta)/M_PI;
  //float theta2_deg = 10*(M_PI - 2*alpha)/M_PI;

  // L1 = 69
  // L2 = 47

  double theta2_rad = acos((x*x + y*y - 69*69 - 47*47)/(2*69*47));
  double theta1_rad = atan(y/x) - atan(47*sin(theta2_rad)/(69 + 47*cos(theta2_rad)));
  double theta1 = 90 - (theta1_rad * 180/M_PI);
  double theta2 = 90 - (theta2_rad * 180/M_PI);
  Serial.print("ANGULOS -> ");
  Serial.print(theta1);
  Serial.print("; ");
  Serial.println(theta2);

  int pos_motor1 = (int)((theta1 * 100/9) + 1500);
  int pos_motor2 = (int)((theta2 * 100/9) + 1500);
  Serial.print("POSICOES -> ");
  Serial.print(pos_motor1);
  Serial.print("; ");
  Serial.println(pos_motor2);

  pos[0] = pos_motor1;
  pos[1] = pos_motor2;
}


void use_servo(int servo, int final_pos) {
  // Mapeia o servo na posicao do vetor de servos
  int servo_i = servo;
  if (servo > 5)
    servo_i -= 4;

  Serial.print(servo);
  Serial.print(" - ");

  Serial.print(servo_i);
  Serial.print(" - ");
  
  // Pega posicao atual do servo
  int initial_pos = position_history[servo_i];

  // Calcula o quanto tem que andar
  int amnt = final_pos - initial_pos;
  Serial.print("amnt = ");
  Serial.print(amnt);
  Serial.print(" final = ");
  
  // Move o servo
  if (amnt > 0) {
    for(int i = 0; i < amnt; i += 2) {
      pwm.writeMicroseconds(servo, (initial_pos + i));
      delay(1);
    }
    position_history[servo_i] = final_pos;
    Serial.println(position_history[servo_i]);
    //vTaskDelete(NULL);
    
  } else if (amnt < 0) {
    amnt = amnt*-1;
    for(int i = 0; i < amnt; i += 2) {
      pwm.writeMicroseconds(servo, (initial_pos - i));
      delay(1);
    }
    position_history[servo_i] = final_pos;
    Serial.println(position_history[servo_i]);
    //vTaskDelete(NULL);
    
  } else {
      Serial.println("");
      //vTaskDelete(NULL);
  }
}

void ponta_de_pe() {
  servo_t servo_aux;
  
//  use_servo(0, 1300);
//  use_servo(5, 1700);
//  use_servo(10, 1700);
//  use_servo(15, 1300);
  
  /* Pernas */
//  use_servo(1, 1250);
//  use_servo(4, 1850);
//  use_servo(11, 1900);
//  use_servo(14, 1250);
  
}

void walk() {
  /* Primeiro passo */
//  use_servo(0, position_history[0] - 300);
//  use_servo(15, position_history[15 - 4] - 300);
//  use_servo(5, position_history[5] - 200);
//  use_servo(10, position_history[10 - 4] + 200);
//  
//  use_servo(11, position_history[11 - 4] + 300);
//  use_servo(4, position_history[4] - 250);
//  
//  use_servo(10, position_history[10 - 4] - 200);
//  use_servo(5, position_history[5] + 200);
//
//  use_servo(0, position_history[0] + 300);
//  use_servo(15, position_history[15 - 4] + 300);
//
//  /* Recentraliza */
//  use_servo(0, position_history[0] + 300);
//  use_servo(15, position_history[15 - 4] + 300);
//
//  use_servo(5, position_history[5] + 300);
//  use_servo(4, position_history[4] + 250);
//  use_servo(11, position_history[11 - 4] - 300);
}

void teste() {
  int* pos = (int*)malloc(sizeof(int)*2);
  cinematica(20, 70, pos);
  
  use_servo(11, pos[0]);
  use_servo(10, pos[1]);
}

void loop() {
  int controle;
  
  if (Serial.available() > 0) {
    controle = Serial.parseInt();
    Serial.println(controle);
    
    if(controle == 0) {
      ponta_de_pe();
    }
    if(controle == 1) {
      walk();
    }
    if(controle == 2) {
      teste();
    }
  }
}
