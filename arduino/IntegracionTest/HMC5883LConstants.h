/*
  Direccion I2C base del sensor.
*/
const int HMC5883L=0x1E;

/*
  Direcciones de los registros del sensor.
*/
#define HMC5883L_CONFIG_A        0x00
#define HMC5883L_CONFIG_B        0x01
#define HMC5883L_MODE            0x02
#define HMC5883L_DATAX_H         0x03
#define HMC5883L_DATAX_L         0x04
#define HMC5883L_DATAZ_H         0x05
#define HMC5883L_DATAZ_L         0x06
#define HMC5883L_DATAY_H         0x07
#define HMC5883L_DATAY_L         0x08
#define HMC5883L_STATUS          0x09
#define HMC5883L_ID_A            0x0A
#define HMC5883L_ID_B            0x0B
#define HMC5883L_ID_C            0x0C

/* 
  Valores de configuracion de la ganancia
  del sensor.
  Influyen en la resolucion de las salidas.
  El valor que vamos a seleccionar, a priori es
  HMC5883L_GAIN_1370.
*/
#define HMC5883L_GAIN_1370          0x00
#define HMC5883L_GAIN_1090          0x01
#define HMC5883L_GAIN_820           0x02
#define HMC5883L_GAIN_660           0x03
#define HMC5883L_GAIN_440           0x04
#define HMC5883L_GAIN_390           0x05
#define HMC5883L_GAIN_330           0x06
#define HMC5883L_GAIN_220           0x07

#define HMC5883L_MODEREG_BIT        1
#define HMC5883L_MODEREG_LENGTH     2

#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01
#define HMC5883L_MODE_IDLE          0x02

#define HMC5883L_STATUS_LOCK_BIT    1
#define HMC5883L_STATUS_READY_BIT   0
