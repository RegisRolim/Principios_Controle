#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#define MPU_ADDR 0x68
#define MEM_START_ADDR 0x6E
#define MEM_R_W 0x6F
// NÃO ALTERAR
    long lastRead = 0;
    byte processed_packet[8];
    byte readd_packet[50];
    byte temp = 0;
    byte fifoCountL = 0;
    byte fifoCountL2 = 0;
    byte packetCount = 0x00;
    boolean longPacket = false;
    boolean firstPacket = true;
    float q[4];
    float Euler[3]; // psi, theta, phi
    float hq[4];
    float a[4];
    float b[4];
    int first = 0;
    boolean reset_coord = false;
    boolean ok_coord = false;
// INSIRA SUAS VARIÁVEIS ------
float set_point = 0;
float angulo_atual = 0;
int erro = 0;


int velocidade1 = 155;
int velocidade2 = 155;
int dados = 0;
int ang = 0;
// Motor 1 (Dianteiros);
int M1P1 = 13;
int M1P2 = 12;

// Motor 2 (Traseiros);
int M2P1 = 8;
int M2P2 = 7;


void setup(){
  pinMode(M1P1, OUTPUT);
  pinMode(M1P2, OUTPUT);
  pinMode(M2P1, OUTPUT);
  pinMode(M2P2, OUTPUT);
  
  Serial.begin(115200);
  sensor_setup();
    while(first <= 400){
    sensor_loop();
  }
}

void loop(){
  sensor_loop();
  ang = Euler[0];

    if(ang > 0 ){  // Se o angulo pesar para um dos lados (no caso o lado positivo).    
        velocidade1 = velocidade1 - 1;  
        velocidade2 = velocidade2 + 1;
      }
      if(ang < 0){
        velocidade1 = velocidade1 + 1;  
        velocidade2 = velocidade2 - 1;
      }
      Controle();
  
}//Fim do loop
  void Controle(){
    digitalWrite(M1P1, HIGH); // 5v
    digitalWrite(M1P2, LOW);  // 0v

  
    digitalWrite(M2P1, HIGH); // 5v
    digitalWrite(M2P2, LOW);  // 0v
    analogWrite(5, velocidade1); 
    analogWrite(6, velocidade2);

}

void writeQuat(){
  q[0] = (long) ((((unsigned long) processed_packet[0]) << 8) + ((unsigned long) processed_packet[1]));
  q[1] = (long) ((((unsigned long) processed_packet[2]) << 8) + ((unsigned long) processed_packet[3]));
  q[2] = (long) ((((unsigned long) processed_packet[4]) << 8) + ((unsigned long) processed_packet[5]));
  q[3] = (long) ((((unsigned long) processed_packet[6]) << 8)  + ((unsigned long) processed_packet[7]));
  for(int i = 0; i < 4; i++ ) {
    if( q[i] > 32767 ) {
      q[i] -= 65536;
    }
    q[i] = ((float) q[i]) / 16384.0f;
  }

  if (first >= 400 ){
    if(first == 400 || reset_coord == true){
      a[0] = q[0];
      a[1] = -q[1];
      a[2] = -q[2];
      a[3] = -q[3];
      reset_coord = false;
    }
    b[0] = q[0];
    b[1] = q[1];
    b[2] = q[2];
    b[3] = q[3];
    q[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    q[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    q[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    q[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
    }
    if(first <= 400){
      first++;
    }
  
  Euler[0] = (atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1))*180/3.1415926; // psi
  //Euler[1] = (-asin(2 * q[1] * q[3] + 2 * q[0] * q[2]))*180/3.1415926; // theta
  //Euler[2] = (atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1))*180/3.1415926; // phi

  //Serial.print("Angulo Atual: ");Serial.print(Euler[0]); Serial.print(" , ");
  //Serial.print("Erro: ");Serial.print(erro);Serial.print(" , ");
  //Serial.print("Set Point: ");Serial.println(set_point);
  //Serial.print("X: ");Serial.print(x);Serial.print(" , ");
  //Serial.print("Y: ");Serial.println(y);
}
