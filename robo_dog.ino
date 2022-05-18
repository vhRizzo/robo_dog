#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150
#define SERVOMAX  600
#define USMIN  600
#define USMAX  2400
#define SERVO_FREQ 50

uint8_t servonum = 15;

int position_history[12] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };

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
    return;
    
  } else if (amnt < 0) {
    amnt = amnt*-1;
    for(int i = 0; i < amnt; i += 2) {
      pwm.writeMicroseconds(servo, (initial_pos - i));
      delay(1);
    }
    position_history[servo_i] = final_pos;
    Serial.println(position_history[servo_i]);
    return;
    
  } else {
      Serial.println("");
      return;
  }
}

void ponta_de_pe() {
  use_servo(0, 1150);
  use_servo(1, 1150);
  use_servo(4, 1850);
  use_servo(5, 1850);
  use_servo(11, 1850);
  use_servo(10, 1850);
  use_servo(14, 1150);
  use_servo(15, 1150);
}

void walk() {
  use_servo(14, 1000);
  use_servo(11, 2000);
  use_servo(4, 1600);
  use_servo(1, 1300);
  
  //ponta_de_pe();
  //use_servo(4, 2000);
  //use_servo(11, 1650);
  
  //use_servo(0, 1300);
  //use_servo(15, 1700);

  //ponta_de_pe();
  //use_servo(1, 2000);
  //use_servo(10, 1650);
  
  //use_servo(5, 1300);
  //use_servo(10, 1700);
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
  }
}
