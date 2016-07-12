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

  Todo el cálculo de los ángulos yrp se deriva de 
  http://www.geekmomprojects.com/mpu-6050-redux-dmp-data-fusion-vs-complementary-filter/#

  Este cálculo requiere del uso de la entrada de interrupción de arduino, situada en el pin 2
  en el caso del Arduino Uno. Debe conectarse a la salida INT del MPU6050.
  
******************************/
#include "MPU6050Constants.h"
#include "HMC5883LConstants.h"
#include "BH1750FVIConstants.h"
#include <Wire.h>
#include <RH_ASK.h>
#include <SPI.h>

// Librerías requeridas para el cálculo de la orientación.
// Las librerías I2Cdev and MPU6050 deben instalarse en el IDE como librerías,
// o situarlas en el path de includes del proyecto.

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

// Objeto que representa a la unidad de medición inercial.
MPU6050 mpu;




//Tamaño maximo buffer trasnmision.
// 61 caracteres como maximo y el caracter null \0 solo salida IMU
//const int MAX_BUFFER = 62;
// Si sumamos la salida del magnetometro:
const int MAX_BUFFER = 100;


// Magnitudes de los sensores.
int temp;
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

// Variables requeridas para el cálculo de la orientación.
bool dmpReady = false;  // True si la inicialización del DMP es correcta.
uint8_t mpuIntStatus;   // Almacena el byte de status de la interrupción de la MPU.
uint8_t devStatus;      // Devuelve el estado del dispositivo MPU tras cada operación (0 = OK, !0 = error)
uint16_t packetSize;    // Tamaño de paquete DMP (default is 42 bytes)
uint16_t fifoCount;     // Contador del número de bytes almacenados en FIFO.
uint8_t fifoBuffer[64]; // Buffer de almacenamiento FIFO.

// Orientation/motion variables
Quaternion q;
VectorFloat gravity;
float euler[3];
float ypr[3];

// This global variable tells us how to scale gyroscope data
float    GYRO_FACTOR;

// This global varible tells how to scale acclerometer data
float    ACCEL_FACTOR;


// Variables to store the values from the sensor readings
int16_t ax, ay, az;
int16_t gx, gy, gz;

const float RADIANS_TO_DEGREES = 57.2958;


/************************************************************************
 * Rutina de detección de interrupción.
 * 
 */
 
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

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
  
  Serial.begin(57600);

  if (driver.init()) {
    // Si el enlace de radio funciona, encendemos el led L de la placa.
    digitalWrite(13, HIGH);
  }

  Wire.begin();
  
  Serial.println("Configurando MPU6050");
  //configure_MPU6050();
  mpu.initialize();

  // verify connection
  Serial.println(F("Probando conexion al dispositivo..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 conexion correcta") : F("MPU6050 fallo en la conexion"));

  Serial.println(F("Inicializando DMP..."));
  devStatus = mpu.dmpInitialize();

  // Si el valor es 0 todo ha ido como se esperaba
  if (devStatus == 0) {
      // Habilitamos el DMP.
      Serial.println(F("Habilitando DMP..."));
      mpu.setDMPEnabled(true);
  
      // Habilitamos la detección de interrupciones.
      Serial.println(F("Habilitando detección de interrupciones (Arduino external interrupt 0)..."));
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
  
      // Ponemos a true el flag dmpReady
      Serial.println(F("DMP list, esperando primera interrupción..."));
      dmpReady = true;
  
      // Establecemos el rango max de escala para el giroscopio.
      uint8_t FS_SEL = 0;

  
      // Obtenemos el valor de rango maximo de escala para el giroscopio.
      // Devuelve valores entre 0 y 3
      uint8_t READ_FS_SEL = mpu.getFullScaleGyroRange();
      Serial.print("FS_SEL = ");
      Serial.println(READ_FS_SEL);
      GYRO_FACTOR = 131.0/(FS_SEL + 1);
      
  
      // Obtenemos rango maximo de escala del acelerometro.
      uint8_t READ_AFS_SEL = mpu.getFullScaleAccelRange();
      Serial.print("AFS_SEL = ");
      Serial.println(READ_AFS_SEL);
  
      // Se obtiene el tamanno maximo de paquete DMP
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = Carga inicial de memoria fallida
      // 2 = Actualizacion de configuracion DMP fallida.
      // (lo habitual es que el error sea 1)
      Serial.print(F("Inicializacion DMP fallida (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }


  
  
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

  //read_MPU6050();
  mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

  // Si se ha producido la interrupción
  if (mpuInterrupt) {
    // Reseteamos el flag de interrupción
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
  
    // Calculamos el número de bytes en FIFO.
    fifoCount = mpu.getFIFOCount();
  
    // Comprobamos si hay overflow (Esto no debería pasar)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // Reseteamos la FIFO.
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
  
    // En cualquier otro caso, comprobamos que existan datos.
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        
        // Obtain YPR angles from buffer
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

/*
       Serial.print("DMP:");
       Serial.print(ypr[2]*RADIANS_TO_DEGREES, 2);
       Serial.print(":");
       Serial.print(-ypr[1]*RADIANS_TO_DEGREES, 2);
       Serial.print(":");
       Serial.println(ypr[0]*RADIANS_TO_DEGREES, 2);
 */      
    }
  }

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
  // Primer valor, rotación sobre eje X.
  // Segundo valor, rotación sobre eje Y.
  // Tercer valor, rotación sobre eje Z.
  out += String(ypr[2] * RADIANS_TO_DEGREES);
  out += ";";
  out += String(ypr[1] * RADIANS_TO_DEGREES);
  out += ";";
  out += String(ypr[0] * RADIANS_TO_DEGREES);
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

  out += "\n";
  //Serial.println("Temperatura: ");
  //Serial.println(temp/340.00+36.53);

  Serial.println(out);
  
  // El codigo de envio utilizando RF es demasiado lento y provoca
  // el overflow del buffer FIFO para el calculo de la orientacion.
  //out.toCharArray(msg, out.length() +1 );

  
  //msg = "25;12762;-524;516;17360;-3136;212;131;32;409;-256;-217;226;1;1;1;1\0";
/*  Serial.print("\t Enviando: [");
  Serial.print(msg);
  Serial.println("]");
  driver.send((uint8_t *)msg, strlen(msg));
  driver.waitPacketSent();
  
  packetNumber += 1;*/
  //delay(500); 
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

