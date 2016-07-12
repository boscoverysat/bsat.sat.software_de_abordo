# Arduino

## Resumen

Este directorio contiene los códigos fuente empleados para gestionar la electrónica de abordo del satélite, mediante el empleo
de Arduino.

## Contenido del repositorio

#### Directorio [IntegracionTest](./IntegracionTest/)

Esta carpeta contine el código del programa instalado en el Arduino del satélite así como las librerías necesarias para poder
operar con los sensores seleccionados.

#### Directorio [libs](./libs/)

Contiene una copia estática de las librerías utilizadas para el cálculo de la
orientación por si en el futuro pudieran verse alteradas o desaparecieran.

El contenido principal es la librería [i2cdevlib](https://github.com/jrowberg/i2cdevlib) que pueden encontrar en su última versión en el enlace anterior.

Para utilizar el enlace de RF se debe incluir también la librería [RadioHead](http://www.airspayce.com/mikem/arduino/RadioHead/index.html) concretamente el driver RH_ASK

## Licencia

[![Creative Commons 4.0 logo](img/cc40.png)](http://creativecommons.org/licenses/by-nc-sa/4.0/)
