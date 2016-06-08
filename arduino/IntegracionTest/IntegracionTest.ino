/****************************
 IntegracionTest

  Este sketch comprueba el funcionamiento basico de un sensor mixto acelerometro
  y magnetometro MPU6050

  Author: @goyoregalado
  Esta version incluye una referencia temporal basada en el contador
  de milisegundos del propio arduino.
  No debe considerarse una referencia fiable.
  
  Este codigo se deriva del que se encuentra en:
  http://playground.arduino.cc/Main/MPU-6050#sketch
  
******************************/
#include "MPU6050Constants.h"
#include "HMC5883LConstants.h"
#include "BH1750FVIConstants.h"
#include <Wire.h>
#include <RH_ASK.h>
#include <SPI.h>

//Tamaño maximo buffer trasnmision.
// 61 caracteres como maximo y el caracter null \0 solo salida IMU
//const int MAX_BUFFER = 62;
// Si sumamos la salida del magnetometro:
const int MAX_BUFFER = 100;

int temp;
// Magnitudes de los sensores.
int AcX, AcY, AcZ, GyX, GyY, GyZ, magX, magY, magZ, lux;
// Valor de los sensores LDR.
int ls1, ls2, ls3, ls4;
int valor = 0;
unsigned int packetNumber = 0;
unsigned int milsec, milsecOffset = 0;
String out = "";
// Pin de recepcion 8, pin de transmision 7
RH_ASK driver(2000, 8, 7, 10, false);
//RH_ASK driver;
char msg[MAX_BUFFER];
//char *msg;


void setup() {
  
  AcX = 0;
  AcY = 0;
  AcZ = 0;
  temp = 0;
  GyX = 0;
  GyY = 0;
  GyZ = 0;
  magX = 0;
  magY = 0;
  magZ = 0;
  lux = 0;
  ls1 = 0;
  ls2 = 0;
  ls3 = 0;
  ls4 = 0;
  
  Serial.begin(9600);

  if (driver.init()) {
    // Si el enlace de radio funciona, encendemos el led L de la placa.
    digitalWrite(13, HIGH);
  }

  Wire.begin();
  
  Serial.println("Configurando MPU6050");
  configure_MPU6050();
  Serial.println("Configurando HMC5883L");
  configure_hmc5883l();
  Serial.println("Configurando BH1750FVI");
  configure_bh1750FVI();

  

  
  Serial.println("Obteniendo marca temporal de referencia");
  milsecOffset = millis();
  
  // Formato de linea.
  // nPaquete;milis;AcX;AcY;AcZ;T;GyX;GyY;GyZ;magX;magY;magZ
   
}

void loop() {
  out = "";
  milsec = millis();
  milsec = milsec - milsecOffset;

  read_MPU6050();
  hmc5883l_singleread();
  read_bh1750FVI();
  ls1 = analogRead(A0);
  ls2 = analogRead(A1);
  ls3 = analogRead(A2);
  ls4 = analogRead(A4);
  
  out += String(packetNumber);
  out += ";";
  out += String(milsec);
  out += ";";
  out += String(AcX);
  out += ";";
  out += String(AcY);
  out += ";";
  out += String(AcZ);
  out += ";";
  out += String(temp);
  out += ";";
  out += String(GyX);
  out += ";";
  out += String(GyY);
  out += ";";
  out += String(GyZ);
  out += ";";
  out += String(magX);
  out += ";";
  out += String(magY);
  out += ";";
  out += String(magZ);
  out += ";";
  out += String(lux);
/*  out += ";";
  if (ls1 >= 500) {
    out += "1;"; 
  }
  else {
    out += "0;"; 
  }
  if (ls2 >= 500) {
    out += "1;"; 
  }
  else {
    out += "0;"; 
  }
  if (ls3 >= 500) {
    out += "1;"; 
  }
  else {
    out += "0;"; 
  }
  if (ls4 >= 500) {
    out += "1"; 
  }
  else {
    out += "0"; 
  }*/
  out += "\n";
  //Serial.println("Temperatura: ");
  //Serial.println(temp/340.00+36.53);

  Serial.println(out);
  out.toCharArray(msg, out.length() +1 );
  //out.toCharArray(msg, out.length() );

  
  //msg = "25;12762;-524;516;17360;-3136;212;131;32;409;-256;-217;226;1;1;1;1\0";
  Serial.print("\t Enviando: [");
  Serial.print(msg);
  Serial.println("]");
  driver.send((uint8_t *)msg, strlen(msg));
  driver.waitPacketSent();
  
  packetNumber += 1;
  delay(500); 
}

void configure_MPU6050() {
  Wire.beginTransmission(MPU);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission(true);  
}

void read_MPU6050() {
  Wire.beginTransmission(MPU);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);
  AcX = Wire.read() <<8| Wire.read();
  AcY = Wire.read() <<8| Wire.read();
  AcZ = Wire.read() <<8| Wire.read();
  temp = Wire.read() <<8| Wire.read();
  GyX = Wire.read() <<8| Wire.read();
  GyY = Wire.read() <<8| Wire.read();
  GyZ = Wire.read() <<8| Wire.read();
  Wire.endTransmission(true);  
}

void configure_hmc5883l() {
  Wire.beginTransmission(HMC5883L);
  // Se configura 0 00 001 00
  // El bit mas significativo siempre debe ser 0
  // Los siguientes dos bits se ponen a cero para que no haga
  // media de lecturas.
  // Los siguientes tres bits toman el valor 100 y eso provoca
  // que la velocidad de medida sea de 15 Hz
  // Los dos ultimos bits establecen que la medida se haga sin sesgo, por eso toman valor 00
  Wire.write(HMC5883L_CONFIG_A);
  Wire.write(0x04);
  Wire.endTransmission(true);
  
  Wire.beginTransmission(HMC5883L);
  // Se configura 000 00000
  // Los tres primeros bits configuran la ganancia de las salidas. Toman un valor 000
  // este valor implica que la ganancia sea de 1370 LSB/Gauss
  Wire.write(HMC5883L_CONFIG_B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  Wire.beginTransmission(HMC5883L);
  // Se configura con el valor 000000 01
  // Los seis primeros bits tienen que configurarse a cero obligatoriamente.
  // Los dos ultimos bits se configuran con el valor 01 que representa el modo de lectura unica.
  // Si queremos usar lecturas continuas entonces se programa con 00
  // Dado que en modo de lectura unica el chip permanece en reposo hasta la siguiente lectura, parece
  // el modo mas adecuado para el modelo de cubesat.
  Wire.write(HMC5883L_MODE);
  Wire.write(0x01);
  Wire.endTransmission(true);
}

void hmc5883l_singleread() {
  Wire.beginTransmission(HMC5883L);
  // Se configura con el valor 000000 01
  // Los seis primeros bits tienen que configurarse a cero obligatoriamente.
  // Los dos ultimos bits se configuran con el valor 01 que representa el modo de lectura unica.
  // Si queremos usar lecturas continuas entonces se programa con 00
  // Dado que en modo de lectura unica el chip permanece en reposo hasta la siguiente lectura, parece
  // el modo mas adecuado para el modelo de cubesat.
  Wire.write(HMC5883L_MODE);
  Wire.write(0x01);
  Wire.endTransmission(true);  
  
  Wire.beginTransmission(HMC5883L);
  Wire.write(HMC5883L_DATAX_H);
  Wire.endTransmission(false);
  Wire.requestFrom(HMC5883L, 6, true);
  magX = Wire.read() <<8| Wire.read();
  magZ = Wire.read() <<8| Wire.read();
  magY = Wire.read() <<8| Wire.read();

}

void configure_bh1750FVI() {
  Wire.beginTransmission(BH1750_L);
  Wire.write(Power_On);
  Wire.endTransmission(true);
  
  Wire.beginTransmission(BH1750_L);
  Wire.write(Continuous_H_resolution_Mode);
  Wire.endTransmission(true);
}

void read_bh1750FVI() {
  Wire.beginTransmission(BH1750_L);
  Wire.requestFrom(BH1750_L, 2);
  lux = Wire.read() <<8| Wire.read();
  
}

