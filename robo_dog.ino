#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <math.h>

int L1 = 69;
int L2 = 47;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150
#define SERVOMAX  600
#define USMIN  600
#define USMAX  2400
#define SERVO_FREQ 50

uint8_t servonum = 15;
int servo_aux[2];

int position_history[12] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };

void use_servo_0 ( void* arg );
void use_servo_1 ( void* arg );
void use_servo_2 ( void* arg );
void use_servo_3 ( void* arg );
void use_servo_4 ( void* arg );
void use_servo_5 ( void* arg );
void use_servo_10 ( void* arg );
void use_servo_11 ( void* arg );
void use_servo_12 ( void* arg );
void use_servo_13 ( void* arg );
void use_servo_14 ( void* arg );
void use_servo_15 ( void* arg );

void setup() {
  Serial.begin(9600);

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

  Serial.println("8 channel Servo test!");
  
  xTaskCreate(loop_task, "Loop_Task", 10 * 1024, NULL, 25, NULL);
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
  Serial.print("ANGULOS -> ");
  Serial.print(theta1);
  Serial.print("; ");
  Serial.println(theta2);

  int pos_motor1 = (int)((theta1 * 100 / 9) + 1500);
  int pos_motor2 = (int)((theta2 * 100 / 9) + 1500);
  Serial.print("POSICOES -> ");
  Serial.print(pos_motor1);
  Serial.print("; ");
  Serial.println(pos_motor2);

  pos[0] = pos_motor1;
  pos[1] = pos_motor2;
}

void ponta_de_pe()
{
  int* pos = (int*)malloc(sizeof(int) * 2);
  int servo[2];

  servo[0] = 11;
  servo[1] = 10;
  cinematica(20, 90, pos, servo);

  xTaskCreate (use_servo_11, "Use_Servo_11", 4 * 1024, (void*)pos[0], 0, NULL);
  xTaskCreate (use_servo_10, "Use_Servo_10", 4 * 1024, (void*)pos[1], 0, NULL);

  servo[0] = 1;
  servo[1] = 0;
  cinematica(20, 90, pos, servo);

  xTaskCreate (use_servo_1, "Use_Servo_1", 4 * 1024, (void*)pos[0], 0, NULL);
  xTaskCreate (use_servo_0, "Use_Servo_0", 4 * 1024, (void*)pos[1], 0, NULL);

  servo[0] = 4;
  servo[1] = 5;
  cinematica(20, 90, pos, servo);

  xTaskCreate (use_servo_4, "Use_Servo_4", 4 * 1024, (void*)pos[0], 0, NULL);
  xTaskCreate (use_servo_5, "Use_Servo_5", 4 * 1024, (void*)pos[1], 0, NULL);

  servo[0] = 14;
  servo[1] = 15;
  cinematica(20, 90, pos, servo);

  xTaskCreate (use_servo_14, "Use_Servo_14", 4 * 1024, (void*)pos[0], 0, NULL);
  xTaskCreate (use_servo_15, "Use_Servo_15", 4 * 1024, (void*)pos[1], 0, NULL);
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

void loop_task ( void* arg ) {
  int controle;
  while (true)
  {
    if (Serial.available() > 0) {
      controle = Serial.parseInt();
      Serial.println(controle);
  
      if (controle == 0) {
        ponta_de_pe();
      }
      if (controle == 1) {
        walk();
      }
      //    if(controle == 2) {
      //      teste();
      //    }
    }
  }
}

void loop() { }

void use_servo_0 ( void* arg ) {
  int servo = 0;
  int final_pos = (int)arg;
  
  // Mapeia o servo na posicao do vetor de servos
  int servo_i = servo;
  if (servo > 5)
    servo_i -= 4;

  // Pega posicao atual do servo
  int initial_pos = position_history[servo_i];

  // Calcula o quanto tem que andar
  int amnt = 0;

  // Move o servo
  if (amnt > 0) {
    for (int i = 0; i < amnt; i += 5) {
      pwm.writeMicroseconds(servo, (initial_pos + i));
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

  } else if (amnt < 0) {
    amnt = amnt * -1;
    for (int i = 0; i < amnt; i += 5) {
      pwm.writeMicroseconds(servo, (initial_pos - i));
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

  } else {
    Serial.println();
    vTaskDelete(NULL);
  }
}

void use_servo_1 ( void* arg ) {
  int servo = 1;
  int final_pos = (int)arg;
  
  // Mapeia o servo na posicao do vetor de servos
  int servo_i = servo;
  if (servo > 5)
    servo_i -= 4;

  // Pega posicao atual do servo
  int initial_pos = position_history[servo_i];

  // Calcula o quanto tem que andar
  int amnt = 0;

  // Move o servo
  if (amnt > 0) {
    for (int i = 0; i < amnt; i += 5) {
      pwm.writeMicroseconds(servo, (initial_pos + i));
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

  } else if (amnt < 0) {
    amnt = amnt * -1;
    for (int i = 0; i < amnt; i += 5) {
      pwm.writeMicroseconds(servo, (initial_pos - i));
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

  } else {
    Serial.println();
    vTaskDelete(NULL);
  }
}

void use_servo_4 ( void* arg ) {
  int servo = 4;
  int final_pos = (int)arg;
  
  // Mapeia o servo na posicao do vetor de servos
  int servo_i = servo;
  if (servo > 5)
    servo_i -= 4;

  // Pega posicao atual do servo
  int initial_pos = position_history[servo_i];

  // Calcula o quanto tem que andar
  int amnt = 0;

  // Move o servo
  if (amnt > 0) {
    for (int i = 0; i < amnt; i += 5) {
      pwm.writeMicroseconds(servo, (initial_pos + i));
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

  } else if (amnt < 0) {
    amnt = amnt * -1;
    for (int i = 0; i < amnt; i += 5) {
      pwm.writeMicroseconds(servo, (initial_pos - i));
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

  } else {
    Serial.println();
    vTaskDelete(NULL);
  }
}

void use_servo_5 ( void* arg ) {
  int servo = 5;
  int final_pos = (int)arg;
  
  // Mapeia o servo na posicao do vetor de servos
  int servo_i = servo;
  if (servo > 5)
    servo_i -= 4;

  // Pega posicao atual do servo
  int initial_pos = position_history[servo_i];

  // Calcula o quanto tem que andar
  int amnt = 0;

  // Move o servo
  if (amnt > 0) {
    for (int i = 0; i < amnt; i += 5) {
      pwm.writeMicroseconds(servo, (initial_pos + i));
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

  } else if (amnt < 0) {
    amnt = amnt * -1;
    for (int i = 0; i < amnt; i += 5) {
      pwm.writeMicroseconds(servo, (initial_pos - i));
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

  } else {
    Serial.println();
    vTaskDelete(NULL);
  }
}

void use_servo_10 ( void* arg ) {
  int servo = 10;
  int final_pos = (int)arg;

  // Mapeia o servo na posicao do vetor de servos
  int servo_i = servo;
  if (servo > 5)
    servo_i -= 4;

  // Pega posicao atual do servo
  int initial_pos = position_history[servo_i];

  // Calcula o quanto tem que andar
  int amnt = 0;

  // Move o servo
  if (amnt > 0) {
    for (int i = 0; i < amnt; i += 5) {
      pwm.writeMicroseconds(servo, (initial_pos + i));
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

  } else if (amnt < 0) {
    amnt = amnt * -1;
    for (int i = 0; i < amnt; i += 5) {
      pwm.writeMicroseconds(servo, (initial_pos - i));
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

  } else {
    Serial.println();
    vTaskDelete(NULL);
  }
}

void use_servo_11 ( void* arg ) {
  int servo = 11;
  int final_pos = (int)arg;
  
  // Mapeia o servo na posicao do vetor de servos
  int servo_i = servo;
  if (servo > 5)
    servo_i -= 4;

  // Pega posicao atual do servo
  int initial_pos = position_history[servo_i];

  // Calcula o quanto tem que andar
  int amnt = 0;

  // Move o servo
  if (amnt > 0) {
    for (int i = 0; i < amnt; i += 5) {
      pwm.writeMicroseconds(servo, (initial_pos + i));
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

  } else if (amnt < 0) {
    amnt = amnt * -1;
    for (int i = 0; i < amnt; i += 5) {
      pwm.writeMicroseconds(servo, (initial_pos - i));
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

  } else {
    Serial.println();
    vTaskDelete(NULL);
  }
}

void use_servo_14 ( void* arg ) {
  int servo = 14;
  int final_pos = (int)arg;
  
  // Mapeia o servo na posicao do vetor de servos
  int servo_i = servo;
  if (servo > 5)
    servo_i -= 4;

  // Pega posicao atual do servo
  int initial_pos = position_history[servo_i];

  // Calcula o quanto tem que andar
  int amnt = 0;

  // Move o servo
  if (amnt > 0) {
    for (int i = 0; i < amnt; i += 5) {
      pwm.writeMicroseconds(servo, (initial_pos + i));
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

  } else if (amnt < 0) {
    amnt = amnt * -1;
    for (int i = 0; i < amnt; i += 5) {
      pwm.writeMicroseconds(servo, (initial_pos - i));
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

  } else {
    Serial.println();
    vTaskDelete(NULL);
  }
}

void use_servo_15 ( void* arg ) {
  int servo = 15;
  int final_pos = (int)arg;
  
  // Mapeia o servo na posicao do vetor de servos
  int servo_i = servo;
  if (servo > 5)
    servo_i -= 4;

  // Pega posicao atual do servo
  int initial_pos = position_history[servo_i];

  // Calcula o quanto tem que andar
  int amnt = 0;

  // Move o servo
  if (amnt > 0) {
    for (int i = 0; i < amnt; i += 5) {
      pwm.writeMicroseconds(servo, (initial_pos + i));
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

  } else if (amnt < 0) {
    amnt = amnt * -1;
    for (int i = 0; i < amnt; i += 5) {
      pwm.writeMicroseconds(servo, (initial_pos - i));
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

  } else {
    Serial.println();
    vTaskDelete(NULL);
  }
}
