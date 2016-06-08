const int MPU=0x68;

/*
Este registro se utiliza para establecer la direccion I2C del dispositivo.
Por defecto, tiene el valor 0x68.
*/
const int WHOAMI=0x75;

/*
PWR_MGMT_1: Este registro permite configurar el modo de alimentacion y el
origen de la señal de reloj.
Poniendo el bit 6 (SLEEP) a 1 pasa a modo sleep.
Si no esta en modo sleep y se habilita el bit 5 (CYCLE) el dispositivo
hace ciclos de una ´nica lectura del acelerometro a un ritmo determinado por
LP_WAKE_CTRL en el registro PWR_MGMT_2
Los bits del 0 al 2 CLKSEL[2:0] establecen la fuente de reloj. A cero usa
oscilador interno a 8MHz.

*/
const int PWR_MGMT_1=0x6B;


/*
PWR_MGMT_2: Este registro permite configurar la frecuencia de wake-ups en modo de bajo
consumo solo acelerometro.
Permite poner ejes del acelerometro y giroscopo en modo standby.
Por defecto, tiene el valor 0x40
*/
const int PWR_MGMT_2=0x6C;

/*
Registros relativos a los acelerometros y y giroscopos.
*/
const int ACCEL_XOUT_H=0x3B;
const int ACCEL_XOUT_L=0x3C;
const int ACCEL_YOUT_H=0x3D;
const int ACCEL_YOUT_L=0x3E;
const int ACCEL_ZOUT_H=0x3F;
const int ACCEL_ZOUT_L=0x40;
const int TEMP_OUT_H=0x41;
const int TEMP_OUT_L=0x42;
const int GYRO_XOUT_H=0x43;
const int GYRO_XOUT_L=0x44;
const int GYRO_YOUT_H=0x45;
const int GYRO_Y_OUT_L=0x46;
const int GYRO_Z_OUT_H=0x47;
const int GYRO_Z_OUT_L=0x48;

/*
  ACCEL_CONFIG: Este registro contiene la configuracion del acelerometro. Sirve para disparar el self-test de cada eje
  y para configurar el valor AFS_SEL que se codifica en los bits 4 y 3. El contenido de este campo define el rango
  maximo del sensor.
  AFS_SEL = 0 --> +-2g
  AFS_SEL = 1 --> +-4g
  AFS_SEL = 2 --> +-8g
  AFS_SEL = 3 --> +-16g
*/
const int ACCEL_CONFIG=0x1C;

/*
  GYRO_CONFIG: Este registro contiene la configuracion del giroscopo. Sirve para configurar la resolucion y sensibilidad
  del giroscopo segun los valores del registro FS_SEL que se codifica en los bits 4 y 3. El contenido de este campo define
  el rango maximo del sensor y su sensibilidad.
  
  Rango maximo de medida.
  FS_SEL = 0 --> +- 250º/s 
  FS_SEL = 1 --> +- 500º/s
  FS_SEL = 2 --> +- 1000º/s 
  FS_SEL = 3 --> +- 2000º/s
  
  Sensibilidad.
  FS_SEL = 0 --> 131 LSB/(º/s)
  FS_SEL = 1 --> 65.5 LSB/(º/s)
  FS_SEL = 2 --> 32.8 LSB/(º/s) 
  FS_SEL = 3 --> 16.4 LSB/(º/s)

*/
const int GYRO_CONFIG=0x1B;

/*
Mascara a aplicar para poder mostrar los valores de configuracion de acelerometro y giroscopo.
*/
const int FULL_SCALE_MASK=0x18;
