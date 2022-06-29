#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

int L1 = 69;      // Tamanho da perna
int L2 = 47;      // Tamanho da pata
int x_ini = -10;  // Coordenada x para a posicao "ponta de pe"
int y_ini = 100;  // Coordenada y para a posicao "ponta de pe"
int pos_step = 1; // Incrementacao do loop para movimentacao dos servos, aumentar esse valor aumentara a velocidade de movimento do robo

/* Prototipos de funcoes */
void setServoPulse(uint8_t n, double pulse);
void cinematica(double x, double y, int* pos, int servo[]);
void use_servo_4( int servo[8], int final_pos[8] );
void ponta_de_pe();
void walk();
void senta();
void finge_de_morto();
void levanta();
void deita();

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150
#define SERVOMAX  600
#define USMIN  600
#define USMAX  2400
#define SERVO_FREQ 50

uint8_t servonum = 15;
int servo_aux[2];

int position_history[12] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1400, 1500, 1500, 1500 };  // Valores iniciais para a posicao de 0° dos servos
// O servo 12 esta em 1400 pois este passou por uma leve descalibracao, provavelmente causado pelo clima

void setup() {
  /* Inicia o seriale os parametros de pwm dos servos */
  Serial.begin(9600);

  pwm.begin();

  pwm.setOscillatorFrequency(25700000);
  pwm.setPWMFreq(SERVO_FREQ);

  /* Faz todos os servos ficarem em 0° */
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
  ponta_de_pe();  // Deixa o robo na posicao padrao para que fique de pe
//  walk();       
                  /* Programar o Arduino com esta linha descomentada, fara
                   * que o robo comece a caminhar sem receber nenhum comando.
                   * Util para faze-lo andar sem necessidade de conectar o 
                   * Arduino por um cabo de dados.
                   */
}

/*
 * Funcao padrao da biblioteca pwm.
 */
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

/* ***** valores de referencia *****
 * centro (0°)  = 1500
 *      -90°    = 500
 *      +90°    = 2500
 *
 * x -> distancia
 * y -> altura
 * theta1 -> motor de cima
 * theta2 -> motor de baixo
 */

/**
 * Faz o calculo da cinematica inversa de uma perna (coxa e pata), onde atraves de coordenadas
 * x e y, ira calcular o angulo que a coxa e a pata deverao assumir para se posicionar naquele
 * ponto. O valor do angulo entao e convertido para microssegundos para uso na biblioteca PWM.
 * @param x Coordenada x do ponto de destino.
 * @param y Coordenada y do ponto de destino.
 * @param pos Ponteiro de retorno onde ficarao armazanadas os valores de microssegundos da
 *            pata [0] e coxa[1].
 * @param servo Vetor contendo os indices dos servos de uma perna com coxa na posicao 0 e pata na posicao 1.
 */
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

  int pos_motor1 = (int)((theta1 * 100 / 9) + 1500);
  int pos_motor2 = (int)((theta2 * 100 / 9) + 1500);

  pos[0] = pos_motor2;
  pos[1] = pos_motor1;
}

/**
 * Move 4 pernas (8 servos) de uma so vez e de forma gradual.
 * @param servo Vetor com os indices dos servos que serao movidos
 * @param final_pos Vetor com os valores de microssegundos para realizar a movimentacao dos servos.
 *                  Cada posicao deste vetor e referente a um servo na mesma posicao
 *                  (e.g. final_pos[3] possui o valor referente ao indice servo[3]).
 */
void use_servo_4 ( int servo[8], int final_pos[8] )
{
  // Mapeia o servo na posicao do vetor de servos
  /*
   * Como a montagem fisica do robo foi feita conectando-se a frente do robo
   * nas 6 primeiras coneccoes e a traseira do robo nas 6 ultimas, a conexao
   * esta como [0,1,2,3,4,5,10,11,12,13,14,15], portanto, para salvar
   * corretamente em um vetor de indices, a segunda metade do vetor devera
   * ter seu indice decrementado em 4 unidades.
   */
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
  /*
   * Armazena a quantidade que cada servo ira movimentar no vetor amnt.
   * Armazena tambem o modulo dessa quantidade em um vetor auxiliar que
   * podera ser decrementado para fazer o controle de quanto ja andou.
   * E armazena a quantidade maxima que um servo ira se movimentar para
   * utilizar como parametro para o loop de movimentacao.
   */
  int amnt[8];
  int amnt_aux[8];
  int max_amnt = 0;
  for(int i = 0; i < 8; i++) {
    amnt[i] = final_pos[i] - initial_pos[i];
    amnt_aux[i] = abs(amnt[i]);
    if(amnt_aux[i] >= max_amnt)
      max_amnt = amnt_aux[i];
  }
  
  /* Move o servo */
  // Loop que ira ate o valor maximo que um servo deve andar, e ira incrementar baseado na largura de passo definida na variavel global auxiliar.
  // Note que "largura de passo" nao tem nada a ver com o quanto o robo ira se locomover. Tomando o servo como um relogio, a largura de passo
  // se trata de quanto o ponteiro ira se movimentar a cada incrementacao, ou seja, um valor maior fara com que o servo se movimente mais rapidamente.
  for (int i = 0; i < max_amnt; i += pos_step)
  {
    // loop para movimentar todos os servos no vetor.
    for(int j = 0; j < 8; j++)
    {
      if (amnt[j] > 0) {                                          // verifica se a movimentacao e pra frente
        if (amnt_aux[j] > 0) {                                    // verifica o servo ainda necessita se movimentar
          if(amnt_aux[j] >= pos_step) {                           // verifica se o passo nao e maior que a quantidade restante para se movimentar
            pwm.writeMicroseconds(servo[j], (initial_pos[j] + i));// movimenta o servo baseado na largura do passo definida
            amnt_aux[j] -= pos_step;                              // decrementa o vetor auxiliar
          }
          else {                                                  /* se a largura do passo era maior do que a quantidade restante que o robo devia andar,
                                                                   * movimenta apenas ate a posicao final e zera o vetor auxiliar, pois este servo
                                                                   * finalizou sua movimentacao
                                                                   */
            pwm.writeMicroseconds(servo[j], final_pos[j]);
            amnt_aux[j] = 0;
          }
        }
      } else if (amnt[j] < 0) {                                     // mesmo comportamento acima, mas com movimentacao para tras
        if (amnt_aux[j] > 0) {
          if(amnt_aux[j] >= pos_step) {                             // em vez de movimentar incrementando a largura de passo a partir da posicao inicial
            pwm.writeMicroseconds(servo[j], (initial_pos[j] - i));  // movimenta decrementando a largura de passo
            amnt_aux[j] -= pos_step;
          }
          else {
            pwm.writeMicroseconds(servo[j], final_pos[j]);
            amnt_aux[j] = 0;
          }
        }
      }
    }
  }
  // Atualiza a posicao atual do servo no historico
  for (int i = 0; i < 8; i++)
    position_history[servo_i[i]] = final_pos[i];
}

/**
 * Move 1 perna (2 servos) de uma so vez e de forma gradual.
 * @see use_servo_4()
 * @param servo Vetor com os indices dos servos que serao movidos
 * @param final_pos Vetor com os valores de microssegundos para realizar a movimentacao dos servos.
 *                  Cada posicao deste vetor e referente a um servo na mesma posicao
 *                  (e.g. final_pos[1] possui o valor referente ao indice servo[1]).
 */
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
    }
  }
  // Atualiza a posicao atual do servo no historico
  for (int i = 0; i < 2; i++)
    position_history[servo_i[i]] = final_pos[i];
}

/**
 * Deixa o robo de pe em sua posicao padrao.
 */
void ponta_de_pe ()
{
  int servo[8];                                 // indice dos servos que serao movidos
  int pos[8];                                   // indice que indica quanto cada servo deve ser movido
  int servo_aux[2];                             // vetor auxiliar que carregara apenas 1 perna por vez para calcular sua cinematica
  int* pos_aux = (int*) malloc(2 * sizeof(int));// ponteiro para receber o indice de movimentacao de cada servo de uma perna

  // Seleciona os servos que serao movidos
  servo[0] = 0;
  servo[1] = 1;
  servo[2] = 4;
  servo[3] = 5;
  servo[4] = 10;
  servo[5] = 11;
  servo[6] = 14;
  servo[7] = 15;

  // Calcula a cinematica de perna por perna
  servo_aux[0] = 1;
  servo_aux[1] = 0;
  cinematica(x_ini, y_ini, pos_aux, servo_aux); 
  pos[0] = pos_aux[0];                  // servo 0 - pata   - 0
  pos[1] = pos_aux[1];                  // servo 1 - coxa  - 0

  servo_aux[0] = 4;
  servo_aux[1] = 5;
  cinematica(x_ini, y_ini, pos_aux, servo_aux);
  pos[2] = pos_aux[1];                  // servo 4 - coxa  - 1
  pos[3] = pos_aux[0];                  // servo 5 - pata   - 1

  servo_aux[0] = 11;
  servo_aux[1] = 10;
  cinematica(x_ini, y_ini, pos_aux, servo_aux);
  pos[4] = pos_aux[0];                  // servo 10 - pata  - 0
  pos[5] = pos_aux[1];                  // servo 11 - coxa - 0

  servo_aux[0] = 14;
  servo_aux[1] = 15;
  cinematica(x_ini, y_ini, pos_aux, servo_aux);
  pos[6] = pos_aux[1];                  // servo 14 - coxa - 1
  pos[7] = pos_aux[0];                  // servo 15 - pata  - 1

  // Move todos os servos de uma vez
  use_servo_4(servo, pos);
}

/**
 * Faz o robo caminhar
 */
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

  int x_ref = 30; // quantidade de referencia em x que uma perna deve se deslocar
  int x_d = 0;    // quantidade em x que uma perna deve se deslocar (atualizado e explicado mais abaixo)
  int y_d = 30;   // quantidade em y que uma perna deve se deslocar
  int n = 10;     // quantidade de passos que o robo dara
  
//  for(int j = 0; j < n; j++) {  // Utilize esta linha caso queira que o robo de n passos
  for( ; ; ) {                    // Ou esta se quiser que ele ande infinitamente
    for (int i = 0; i < 4; i++) { // Move as 4 pernas, uma perna de cada vez
      if (i == 0) {               // faz o controle de qual perna sera calculada a cinematica e movimentada
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
      
      /*
       * Se for uma perna frontal (de 0 a 10), a posicao em x devera se deslocar para frente (em sentido positivo).
       * Se for uma perna traseira (de 10 a 15), a posicao em x devera se deslocar para tras (em sentido negativo).
       */
      if (servo_1[0] >= 10)
        x_d = -1 * x_ref;
      else
        x_d = x_ref;
      
      // Calcula a cinematica de movimentacao apenas se deslocando em y inicialmente
      cinematica(x_ini, y_ini - y_d, pos_aux, servo_aux);
      /*
       * Pega corretamente qual servo e coxa e qual servo e pata.
       * O robo foi montado da seguinte forma: pata esq -> coxa esq -> ombro esq -> ombro dir -> coxa dir -> pata dir
       * Portanto, em uma certa perna, em ordem numerica, os pares serao em um momento [pata,coxa] e em outros [coxa,pata]
       * (e.g. na perna 0,1 o servo 0 e a pata e o servo 1 e a coxa, ja na perna 4,5 o servo 4 e a coxa e o servo 5 e a pata),
       * mas os parametros de movimentacao sempre devem ser passados como [pata,coxa].
       */
      if (i % 2 != 0) {
        pos_1[0] = pos_aux[1];
        pos_1[1] = pos_aux[0];
      } else {
        pos_1[0] = pos_aux[0];
        pos_1[1] = pos_aux[1];
      }
      // Faz o deslocamento
      use_servo_1(servo_1, pos_1);
      delay(10);
    
      // Agora calcula a cinematica com y ainda deslocado, mas tambem deslocando x
      cinematica(x_ini + x_d, y_ini - y_d, pos_aux, servo_aux); 
      if (i % 2 != 0) {
        pos_1[0] = pos_aux[1];
        pos_1[1] = pos_aux[0];
      } else {
        pos_1[0] = pos_aux[0];
        pos_1[1] = pos_aux[1];
      }
      // Faz o deslocamento
      use_servo_1(servo_1, pos_1);
      delay(10);
    
      // Calcula a cinematica do mantendo o x deslocado, mas retornando y para a posicao padrao
      cinematica(x_ini + x_d, y_ini, pos_aux, servo_aux);
      if (i % 2 != 0) {
        pos_1[0] = pos_aux[1];
        pos_1[1] = pos_aux[0];
      } else {
        pos_1[0] = pos_aux[0];
        pos_1[1] = pos_aux[1];
      }
      // Faz o deslocamento
      use_servo_1(servo_1, pos_1);
      delay(10);
    }
    
    /*
     * Retorna o robo para a posicao inicial.
     * Como todas as patas ainda estao deslocadas em x, quando o robo retornar para sua posicao inicial,
     * ele ira na verdade levar todo o seu corpo para a frente, fazendo com que ele conclua um passo.
     */
    ponta_de_pe();
    delay(10);
  }
}

/**
 * Faz com que o robo caia de lado no chao.
 */
void finge_de_morto() {
  int servo[2] = {2,12};
  int pos[2] = {1500 - 750,1500 + 750};
  use_servo_1(servo, pos);  // Desloca dois ombros de um mesmo lado em sentido interno para que o robo perca o equilibrio e caia de lado
  delay(500);               // Aguarda um breve momento antes que se corrija, caso contrario ele ira se corrigir antes de ter caido, e nao tombara para o lado
  pos[0] = 1500;
  pos[1] = 1400;
  use_servo_1(servo, pos);  // Retorna os ombros para suas posicoes padrao
}

/**
 * Faz com que o robo se levante caso tenha tombado de lado (nao funciona caso tenha caido de cabeca para baixo)
 */
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
  
  // Estica todas as pernas no chao sendo feito atraves do calculo da cinematica 
  // onde o deslocamento em y = 0, e em x sera a soma dos comprimentos da coxa e da pata
  servo_aux[0] = 1;
  servo_aux[1] = 0;
  cinematica(L1+L2, 0, pos_aux, servo_aux); 
  pos[0] = pos_aux[0];                  // servo 0 - pata   - 0
  pos[1] = pos_aux[1];                  // servo 1 - perna  - 0

  // Faz o mesmo para todas as pernas
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

  // Semelhante a finge_de_morto(), os ombros de ambas as laterais (garantindo com que ele consiga se levantar independente do lado que tiver caido) irao
  // se deslocar levemente em sentido interno para que o robo caia sobre sua barriga
  servo_aux[0] = 2;
  servo_aux[1] = 12;
  int pos_2[2] = {1500-250,1500+250};
  use_servo_1(servo_aux,pos_2);
  servo_aux[0] = 3;
  servo_aux[1] = 13;
  pos_2[0] = 1500+250;
  pos_2[1] = 1500-250;
  delay(500);
  use_servo_1(servo_aux,pos_2);
  
  // Em seguida retorna os ombros para suas posicoes padrao
  pos_2[0] = 1500;
  pos_2[1] = 1500;
  use_servo_1(servo_aux,pos_2);
  servo_aux[0] = 2;
  servo_aux[1] = 12;
  pos[1] = 1400;
  use_servo_1(servo_aux,pos_2);
  
  // E retorna todas as patas para suas posicoes padrao
  ponta_de_pe();
}

/**
 * Faz com que o robo se estique no chao.
 * @see levanta().
 */
void deita() {
  // Da mesma forma que foi feita na funcao levanta(), o robo ira esticar todas as suas pernas no chao
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
}

/**
 * Faz com que o robo se posicione de uma forma semelhante a um cachorro sentado.
 */
void senta() {
  // Desloca as coxas traseiras para dentro
  int servo[2] = {11,14};
  int pos[2] = {1500-750,1500+750};
  use_servo_1(servo,pos);

  // Estica levemente as patas frontais
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
  // Faz a leitura do serial do Arduino, onde o usuario podera selecionar qual acao o robo devera realizar
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
      if (controle == 5)
        deita();
    }
  }
}
