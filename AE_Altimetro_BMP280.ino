/*
NOTA IMPORTANTE: Tanto este programa como la electrónica asociada han sido diseñadas por
                 Jesús Lourés Lamas con la única intención de investigar sobre el uso de
                 la plataforma Arduino y el sensor BMP280. No es un instrumento suficientemente
                 probado para utilizarlo en una aeronave.
                    Tanto el progama como la electrónica pueden tener fallos que podrían
                 provocar indicaciones erróneas.
                    Ni el autor ni la Asociación de Constructores Amateur de Aeronaves aconseja
                 su uso para otros fines que no sean educativos no asumiendo por tanto ninguna
                 responsabilidad derivada de la utilización de este diseño.

                 Como se muestra más abajo se usan librerías de terceros, consultar en sus
                 respectivas páginas las posibles restricciones a su utilización.

  Universal 8bit Graphics Library, https://github.com/olikraus/u8glib/
  
  Copyright (c) 2012, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
  
*/


#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "U8glib.h"
#include <Wire.h>
#include <EEPROM.h>
#include <Stepper.h>

//#define LED 13            //Led verde en la placa arduino. Solo para pruebas, este pin se usa para el zumbador
#define ALT_OXIGENO 3500.0  //Alarma de oxígeno si se superan los 3.500m (11.500 ft)
#define TMP_ALARM 900000    //Tiempo en milisegundos para rehabilitar alarma de altitud después de haberla anulado (900000 = 15 min)
#define TMP_AJ_PRES 10000 //20000   //Tiempo que es necesario mantener pulsado para que entre en modo ajuste del offset de presión (20 seg)
#define STEPS 720
#define F_ESCALA_C 10000.0  //Valor correspondiente a una vuelta completa de la aguja corta.
const long F_ESCALA_L = 1000.0;   //Valor correspondiente a una vuelta completa de la aguja larga.
const double FACTOR_M = 3.28084;  //La altitud se calcula en metros, este valor es el factor por el que se multiplica para pasr a pies
                                  //Si la escala se va a usar en metros poner 1 en esta definición
const int LECS_MEDIA = 15;  //Lecturas que se harán para obtener la media de presión y temp.
#define TMP_LEC_SENSOR 50  //Espera entre lecturas del sensor. Según la datasheet en ultra high resolution
                             //pueden ser necesarios hasta 43,2mS                                
                          
#define ENC_CLK 2         //Señal CLK de encoder para seleccionar QNH
#define ENC_DT 3          //Señal DT de encoder
#define ENC_SW 12         //Pulsador de eje encoder, no tiene resistencia de pull up, habrá que usar la del micro
#define ZUMBA 13          //Zumbador. Alerta de oxígeno, suena por encima de 3.500 m (11.500 pies), se quita pulsando botón
#define S_HALL_L 16       //Pin 16 (A2) sensor e. Hall aguja larga
#define S_HALL_C 17       //Pin 17 (A3) sensor e. Hall aguja corta
#define DIR_QNH 500       //Dirección en EEPROM para QNH, se toma una intermedia por si alguna librería
                          //hiciese uso de las primeras o últimas posiciones
#define DIR_AJ_CORTA 504  //Dirección en EEPROM para ajuste de aguja corta                      
#define DIR_AJ_LARGA 506  //Dirección en EEPROM para ajuste de aguja larga  
#define DIR_AJ_PRES 508   //Dirección en EEPROM para offset de presión, ojo ocupará al menos 4 bytes, no 2 como los anteriores

//Modelo VID28-05. Las conexiones de los dos motores visto de frente en vertical y con los ejes arriba son:
// - Motor superior, el más cercano al eje (eje interior, aguja larga):
//   pin superior dcha a 10 arduino
//   pin inferior dcha a 11 arduino
//   pin superior izq. a 4 arduino
//   pin inferior izq. a 5 arduino
// - Motor inferior, el más alejado del eje (eje exterior, aguja corta):
//   pin superior dcha a 6 arduino
//   pin inferior dcha a 7 arduino
//   pin superior izq. a 8 arduino
//   pin inferior izq. a 9 arduino

Stepper step_larga(STEPS, 10, 11, 4, 5);  //Aguja Larga
Stepper step_corta(STEPS, 6, 7, 8, 9); //Aguja corta

//Uso U8GLIB, he probado Adafruit pero parece usar más memoria.
//OJO en el circuito de la pantalla marca la dirección 0x78. En las pruebas con la librería
//Adafruit ha sido necesario usar 0x3c y en u8glib aparece: #define I2C_SLA (0x3c*2) en el
//fichero u8g_com_arduino_ssd_i2c.c

// setup u8g object, please remove comment from one of the following constructor calls
// IMPORTANT NOTE: The following list is incomplete. The complete list of supported 
// devices with all constructor calls is here: https://github.com/olikraus/u8glib/wiki/device
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);	// I2C / TWI 
//U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_NO_ACK|U8G_I2C_OPT_FAST);	// Fast I2C / TWI 
//U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK);	// display which does not send AC

//Se declara una instancia de la librería
Adafruit_BMP280 bme; // I2C

//volatile porque se usan en interrupción
volatile long tmp_reb = 250; //250;  //temporización para evitar rebotes de encoder (en mS) 
volatile float QNH;
volatile char ajustando_presion;
volatile char ajustando_corta;
volatile int offset_corta = 0;   //offset para aguja corta sobre los 180 grados teóricos, volatile porque se usa en interrupciones
volatile int offset_larga = 0;   //offset para aguja larga sobre los 180 grados teóricos
volatile int ajus_corta;
volatile int ajus_larga;

long msegs, msec_QNH;
float grab_QNH;  //QNH grabado en EEPROM
//Arduino no da mayor precisión con double que con float, no tiene sentido usar double,
//pero la librería del BMP180 exige tipo double
double Presion = 0;
double Altitud = 0;
double Temperatura = 0;
int error_ini = 0;
boolean modoTest = false;
byte anul_zumba = 0;  //Si distinto de 0 se ha anulado la alarma de oxígeno, no sonará el zumbador
byte sonando = 0;     //A uno cuando se active la alarma de oxígeno
byte ciclo_son;       //Indica si está en ciclo activado o desactivado de intermitencia de sonido
long rehab_zumba;     //Se almacena lectura de millis() para rehabilitar alarma de altitud oxígeno al cabo de TMP_ALARM
long ms_interm;       //Lectura millis() para sonido intermitente
long ms_habAajPres;   //Se almacena lectura de millis() para entrar en modo ajuste de presión si se mantiene pulsado más de 20 segundos
int interm;           //Para desplazar intermitentemente punto por pantalla mostrando que está vivo
int pos_corta = 0;  //posición de la aguja corta en número de pasos (0 a 720)
int pos_larga = 0;  //posición larga
double offset_Pres = 0;  //offset para presión en milibares
int numLectMedia = 0;    //número de lecturas para hacer la media
long ms_LecSensor;  //Se almacena lectura de millis() para espera de siguiente lectura de sensor
int nueva_lectura;  //indica que se ha completado una nueva lectura promedio
double sumTemp, parcTemp, maxTemp, minTemp;  //Variables globales para promedio de lecturas
double sumPres, parcPres, maxPres, minPres;

//Sólo para pruebas
long cont = 0;
double PresionInicial = -999;
double AltitudInicial = -999;
double TemperaturaInicial = -999;


/********************************************************************************************************
drawTest
      Muestra una pantalla pensada para test.
      Aparecen en fuente pequeña la temperatura, presión, QNH y altitud en metros y pies
*********************************************************************************************************/
void drawTest(void) {
  char buf1[16];
  double adoub;
  // graphic commands to redraw the complete screen should be placed here  
  u8g.firstPage();  
  do
  {  
    u8g.setFont(u8g_font_helvB12r);
    //u8g.setFontPosTop();
    dtostrf(Temperatura, 5, 1, buf1);
    strcat(buf1, " C");
    u8g.drawStr(1, 12, buf1);
    dtostrf(Presion, 5, 1, buf1);
    strcat(buf1, " mb");
    u8g.drawStr(60, 12, buf1);
    dtostrf(QNH, 5, 1, buf1);
    u8g.drawStr(1, 28, buf1);
    strcpy(buf1, "QNH");
    u8g.drawStr(60, 28, buf1);
    dtostrf(Altitud, 5, 1, buf1);
    strcat(buf1, " m");
    u8g.drawStr(1, 44, buf1);
    adoub = Altitud * 3.28084;  //Aquí no se pone FACTOR_M porqeu en la pantalla aparece simultáneamoente en metros y pies
    dtostrf(adoub, 5, 1, buf1);
    strcat(buf1, " ft");
    u8g.drawStr(1, 60, buf1);
  } while( u8g.nextPage() );
}

/********************************************************************************************************
drawPpal
      Muestra la pantalla que aparecerá normalmente.
      Aparecen en fuente pequeña la altitud, en metros en la primera línea y
      pies en la tercera en la línea central y fuente grande aparece el QNH.
*********************************************************************************************************/
void drawPpal(void) {
  char buf1[12];
  char *pbuf;
  double adoub;
  int i;
  
  // graphic commands to redraw the complete screen should be placed here 
  interm++;
  if (interm > 10)
    interm = 0;
  u8g.firstPage();  
  do
  {  
    u8g.setFont(u8g_font_helvB12r);
    //Mostrar en primera línea altitud en metros en fuente 12
    dtostrf(Altitud, 5, 0, buf1);
    strcat(buf1, " m");
    //La función dtostrf deja espacios por la izquierda si hay menos dígitos de los indicados
    //Quitar los espacios
    i = 0;
    while((char)*(&buf1[i]) == 0x20)
      i++;
    pbuf = &buf1[i];
    //En lugar del esto se puede usar el método trim de la clase String, pero en vez de array char
    //hay que usar String que ocupa más memoria
    u8g.drawStr(1, 12, pbuf);
    //mostrar temperatura
    //No se muestra, demasiada info. Se puede pulsar botón para verla
    //dtostrf(Temperatura, 2, 1, buf1);
    //strcat(buf1, " C");
    //u8g.drawStr(80, 12, buf1);
    //Mostrat QNH en línea central en fuente 24 pero texto "QNH" en 12
    //u8g.drawFrame(0,14,127,32);
    //u8g.drawFrame(50,14,77,32);
    u8g.drawHLine(0, 18, 50);
    u8g.drawHLine(0, 41, 50);
    u8g.drawHLine(50, 12, 77);
    u8g.drawHLine(50, 47, 77);
    u8g.drawLine(0, 18, 0, 41);
    u8g.drawLine(50, 18, 50, 12);
    u8g.drawLine(50, 41, 50, 47);
    u8g.drawLine(127, 12, 127, 47);
    strcpy(buf1, "QNH");
    u8g.drawStr(5, 36, buf1);
    u8g.setFont(u8g_font_helvB24n);
    //Mostrar en tercera línea altitud en pies en fuente 12
    dtostrf(QNH, 4, 0, buf1);
    u8g.drawStr(53, 42, buf1);
    u8g.setFont(u8g_font_helvB12r);
//    strcpy(buf1, "QNH");
//    u8g.drawStr(84, 42, buf1);
    adoub = Altitud * 3.28084;  //Aquí no se pone FACTOR_M porqeu en la pantalla aparece simultáneamoente en metros y pies
    dtostrf(adoub, 5, 0, buf1);
    strcat(buf1, " f");
    i = 0;
    while((char)*(&buf1[i]) == 0x20)
      i++;
    pbuf = &buf1[i];
    u8g.drawStr(1, 60, pbuf);
    nueva_lectura = 0;
    //dibujar punto que se desplaza para indicar que está muestreando
    for (i = 0; i <= interm; i++)
      buf1[i] = ' ';
    buf1[interm] = '-';
    buf1[interm+1] = 0x00;
    u8g.drawStr(65, 60, buf1);
    
    //Solo para pruebas mostrar contador para ver velocidad de refresco
//    dtostrf(cont, 5, 0, buf1);
//    u8g.drawStr(80, 60, buf1);
//    //Solo para pruebas mostrar posición aguja corta
//    dtostrf(pos_corta, 4, 0, buf1);
//    u8g.drawStr(80, 12, buf1);
//    //Solo para pruebas mostrar posición aguja larga
//    dtostrf(pos_larga, 4, 0, buf1);
//    u8g.drawStr(80, 12, buf1);
    
  } while( u8g.nextPage() );
}

/********************************************************************************************************
ajusteOffset
      Permite ajustar el cero de las agujas, se espera durante 2 segundos a que el usuario pulse el
      botón del encoder. Se ha ampiado a 2 segundos ya que este tiempo sirve para comprobar que las
      agujas no han sufrido desviación.
      Si el parámetro o_corta es distinto de ceros e ajusta la corta, si 0 se ajusta la larga
*********************************************************************************************************/
void ajusteOffset(char o_corta)
{  
  long lact;
  int aint;
  
  //mostrar pantalla ofreciendo ajuste durante 2 seg
   if (o_corta)
    display(0x0100);  //no usar error_ini ya que realmente no es un error
   else
    display(0x0400);
   msegs = millis();
   lact = msegs;
   while ((lact < (msegs + 2000)) && digitalRead(ENC_SW))
   {  //esperar 2 sg. o a que se pulse
    lact = millis();
   }
   delay(150);  //evitar rebotes
   if (!digitalRead(ENC_SW))
   {  //se ha pulsado
    if (o_corta)
      display(0x0200);
    else
      display(0x0800);
    delay(150);
    while (!digitalRead(ENC_SW));  //esperar a que se deje de pulsar
    if (o_corta)
      ajustando_corta = 1;  //indicador para que la interrupción procese el offset adecuado
    else
      ajustando_corta = 0;
    attachInterrupt(0, ajCortaEncoder, FALLING);  //interrupción 0 por flanco - pin 2
    while (digitalRead(ENC_SW))  //esperar a que se pulse para grabar y salir
    {
      if (o_corta)
      {
        display(0x0200);
        if (ajus_corta != 0)
        {
          //noInterrupts(); //deberían inhibirse las interrupciones, pero se bloquea
          step_corta.step(ajus_corta);
          ajus_corta = 0;
          //interrupts();
        }
      }
      else
      {
        display(0x0800);
        if (ajus_larga != 0)
        {
          step_larga.step(ajus_larga);
          ajus_larga = 0;
        }
      }
//      interrupts();
    }
    detachInterrupt(0);
    if (o_corta)
    {
      while (offset_corta > STEPS)
        offset_corta = offset_corta - STEPS;
      while (offset_corta < -STEPS)
        offset_corta = offset_corta + STEPS;
      aint = offset_corta;
      EEPROM.put(DIR_AJ_CORTA, aint);  //si se usa directamente "offset_corta" da un "warning" el compilador
    }   
    else
    {
      while (offset_larga > STEPS)
        offset_larga = offset_larga - STEPS;
      while (offset_larga < -STEPS)
        offset_larga = offset_larga + STEPS;
      aint = offset_larga;
      EEPROM.put(DIR_AJ_LARGA, aint);
    }
  }
}

/********************************************************************************************************
iniMotores
      Inicializa las agujas a su posición cero.
      Si alguna de ellas no s detectada por el sensor de efecto Hall mostrará una pantalla de error.
      Primero ajusta la aguja pequeña y después la larga.
*********************************************************************************************************/
void iniMotores(void)
{
  int pasos;
  int aint;
  
  //Segun la web de VID la velocidad máx. son 600º/seg que equivalen a 100 RPMs.
  //Utilizo 50 para la corta y 40 para la larga porque habrá que contar también con la inercia de las agujas,
  //en el prototipo a 100 no se mueve la aguja corta y con solo 70 a veces falla la larga.
  step_corta.setSpeed(50);
  step_larga.setSpeed(40);
  //Buscar cero de aguja corta
  error_ini = 0;
  display(0x40);  //no usar error_ini ya que realmente no es un error
  if (digitalRead(S_HALL_C) == LOW)
  {
    step_corta.step(-180);  //Si el sensor está activo mover la aguja 90 grados para empezar el recorrido. 
    if (digitalRead(S_HALL_C) == LOW)
    {  //Sige activo, es un error
      error_ini = 0x60;
      display(error_ini);
      return;
    }
  }
  //Avanzar paso a paso hasta detectar aguja corta
  pasos = 0;
  while (pasos < 1440)
  {
    pasos++;
    step_corta.step(-1);
    if (digitalRead(S_HALL_C) == 0)
    {
      pasos = 2000;  //para salir del while 
      EEPROM.get(DIR_AJ_CORTA, aint);   
       if (isnan(aint) || (aint > STEPS) || (aint < (-1 * STEPS)))
        offset_corta = 0;
      else
        offset_corta = aint;
      delay(150); //esperar debido a inercia ya que cambia de sentido
      step_corta.step(offset_corta - 60); //Ir a posición cero 
    }
  }  //while
  if (pasos < 2000)
  { //No se encontró cero de corta
    error_ini = 0x60;
    display(error_ini);
    return;
  }
  //Ajuste de cero para la corta si el usuario lo decide pulsando el botón del encoder
  ajusteOffset(1);
  
  //Buscar cero de aguja larga
  error_ini = 0;
  display(0x10);  //no usar error_ini ya que realmente no es un error
  if (digitalRead(S_HALL_L) == LOW)
  {
    step_larga.step(-180);  //Si el sensor está activo mover la aguja 90 grados para empezar el recorrido. 
    if (digitalRead(S_HALL_L) == LOW)
    {  //Sige activo, es un error
      error_ini = 0x30;
      display(error_ini);
      return;
    }
  }
  //Avanzar paso a paso hasta detectar aguja larga
  pasos = 0;
  while (pasos < 1440)
  {
    pasos++;
    step_larga.step(-1);
    if (digitalRead(S_HALL_L) == 0)
    {
      pasos = 2000;  //para salir del while
      EEPROM.get(DIR_AJ_LARGA, aint);   
       if (isnan(aint) || (aint > STEPS) || (aint < (-1 * STEPS)))
        offset_larga = 0;
      else
        offset_larga = aint;
      delay(150); //esperar debido a inercia ya que cambia de sentido
      step_larga.step(offset_larga + 120); //Volver 90 grados (algo memos) para dejar libre sensor y regresar aproximadamente al cero
    }
  }  //while
  if (pasos < 2000)
  { //No se encontró cero de larga
    error_ini = 0x30;
    display(error_ini);
    return;
  }
  //Ajuste de cero para la larga si el usuario lo decide pulsando el botón del encoder
  ajusteOffset(0);
}

/********************************************************************************************************
sensorStart
      Inicializa el sensor.
      Si no respondde adecuadamente mostrará una pantalla de error.
      Se lee de la EEPROM el QNH, si es mayor de 2000 (probable EEPOM en blanco) o si menor de 900
      se pone el valor por defecto a 1013.
*********************************************************************************************************/
void sensorStart() {
  float aflo;
  double adoub;

  EEPROM.get(DIR_QNH, aflo);   
  if (isnan(aflo) || //Si la memoria está en blanco será todo 0xFF y aflo dará error "NAN"
     (aflo > 1200.0) || (aflo < 900.0))
    QNH = 1013.0; //1013 siempre qeu no esté entre 900 y 1200 mb
  else
    QNH = aflo;
  grab_QNH = QNH;

  EEPROM.get(DIR_AJ_PRES, adoub);   
  if (isnan(adoub) || (adoub < -30.0) || (adoub > 30.0))
    offset_Pres = 0.0;
  else
    offset_Pres = (double)adoub;
  
  // Inicio BMP280
  //No hay parámetro para oversampling, parece que por defecto usa
  //el mínimo para la temperatura y el máximo para la presión (valor 0x3F en registro F4
  //Dirección de la placa sensor de Thinary Electronic (aliexpress)
  if (!bme.begin(0x76)) 
  {  
    error_ini = 0x80;
    display(error_ini);
    return;
  }
}

/********************************************************************************************************
display
      Función para mostrar pantallas completas.
      Pensada para mostrar códigos de error, aunque también se usa para mostrar las líneas de
      información de búsqueda de cero que puede acabar en error (ver llamadas desde iniMotor.
*********************************************************************************************************/
void display(unsigned int cod_err)
{ 
  char buf1[16];
  
  if (cod_err != 0)
  {
    u8g.firstPage();  
    do
    {  
      u8g.setFont(u8g_font_helvB12r); //Si se usa solo esta lib se ahorran más de 6000 bytes
      //u8g.setFont(u8g_font_unifont);
      //u8g.setFontPosTop();
//      if (cod_err & 0x0001) //NO existe en BMP280
//        u8g.drawStr(1, 12, "ERROR Pres.");
//      if (cod_err & 0x0002) //NO existe en BMP280
//        u8g.drawStr(1, 28, "ERROR i. pres.");
//      if (cod_err & 0x0004) //NO existe en BMP280
//        u8g.drawStr(1, 48, "ERROR tmp.");
//      if (cod_err & 0x0008) //NO existe en BMP280
//        u8g.drawStr(1, 60, "ERROR i. tmp.");
      if (cod_err & 0x0010)
      {
        u8g.drawStr(1, 12, "Buscando cero");
        u8g.drawStr(1, 28, "Aguja larga.");
      }
      if (cod_err & 0x0020)
        u8g.drawStr(1, 48, "** ERROR **");
      if (cod_err & 0x0040)
      {
        u8g.drawStr(1, 12, "Buscando cero");
        u8g.drawStr(1, 28, "Aguja corta.");
      }
      if (cod_err & 0x0080)
      {
        u8g.drawStr(1, 12, "** ERROR **");
        u8g.drawStr(1, 36, "EN SENSOR");
        u8g.drawStr(1, 60, "DE PRESION");
      }
      if (cod_err & 0x0100)
      {
        u8g.drawStr(1, 12, "Pulsa para");
        u8g.drawStr(1, 28, "ajustar aguja");
        u8g.drawStr(1, 44, "corta, off:");
        //sprintf(buf1, "offset %d", offset_corta); //usando sprintf se necesitan 1500 bytes más
        dtostrf(offset_corta, 4, 0, buf1);
        u8g.drawStr(80, 44, buf1);
      }
      if (cod_err & 0x0200)
      {
        u8g.drawStr(1, 12, "Gira para");
        u8g.drawStr(1, 28, "ajustar corta");
        u8g.drawStr(1, 44, "y pulsa para");
        u8g.drawStr(1, 60, "grabar ");
        dtostrf(offset_corta, 4, 0, buf1);
        u8g.drawStr(50, 60, buf1);
      }
      if (cod_err & 0x0400)
      {
        u8g.drawStr(1, 12, "Pulsa para");
        u8g.drawStr(1, 28, "ajustar aguja");
        u8g.drawStr(1, 44, "larga, off:");
        //sprintf(buf1, "offset %d", offset_corta); //usando sprintf se necesitan 1500 bytes más
        dtostrf(offset_larga, 4, 0, buf1);
        u8g.drawStr(80, 44, buf1);
      }
      if (cod_err & 0x0800)
      {
        u8g.drawStr(1, 12, "Gira para");
        u8g.drawStr(1, 28, "ajustar larga");
        u8g.drawStr(1, 44, "y pulsa para");
        u8g.drawStr(1, 60, "grabar:");
        dtostrf(offset_larga, 4, 0, buf1);
        u8g.drawStr(50, 60, buf1);
      }
      if (cod_err & 0x1000)
      {
        dtostrf(Presion, 5, 1, buf1);
        strcat(buf1, " mb");
        u8g.drawStr(30, 12, buf1);
        u8g.drawStr(1, 28, "Activado el");
        u8g.drawStr(1, 44, "ajuste de offset");  //no cabe "del"
        u8g.drawStr(1, 60, "presion:");
        dtostrf(offset_Pres, 4, 1, buf1);
        u8g.drawStr(75, 60, buf1);
      }
      if (cod_err & 0x2000)
      {
        dtostrf(Presion, 5, 1, buf1);
        strcat(buf1, " mb");
        u8g.drawStr(30, 12, buf1);
        u8g.drawStr(1, 28, "Ajusta offset");
        u8g.drawStr(1, 44, "y pulsa para");
        u8g.drawStr(1, 60, "grabar:");
        dtostrf(offset_Pres, 4, 1, buf1);
        u8g.drawStr(75, 60, buf1);
      }
    } while( u8g.nextPage() );
  }
}  

/********************************************************************************************************
readSensor
      Función para lectura de presión y temperatura.
      Se hacen 17 lecturas, se desprecian los máximos y mínimos y se devuelve la media de las 15 restantes.
      Hay que establecer un compromiso entre estabilidad y retardo / inercia
      Ver const TMP_LEC_SENSOR y LECS_MEDIA
*********************************************************************************************************/
void readSensor()
{
  double adoub;
  
  if (millis() > (ms_LecSensor + TMP_LEC_SENSOR))
  {
    //Han transcurrido al menos TMP_LEC_SENSOR (50 ms)
    //Lecturas de presión y temperatura y se calcula la Altitud
    //Se hacen 17 lecturas y se promedian los 15 valores centrales (se excluye el máximo y ínimo)
    //Se inicia la lectura de temperatura
    parcTemp = (double)bme.readTemperature();
    parcPres = (double)bme.readPressure();
    ms_LecSensor = millis();
    if (numLectMedia == 0)
    {  //Primera lectura: tomar los primeros valores como máximo y minimo par aluego ir sustituyendo si procede
      sumTemp = parcTemp;
      sumPres = parcPres;
      minTemp = parcTemp;
      maxTemp = parcTemp;
      minPres = parcPres;
      maxPres = parcPres;  
    } else
    { //No es primera lectura, ver si máximo o mínimo
      sumPres = sumPres + parcPres;
      sumTemp = sumTemp + parcTemp;
      if (parcTemp > maxTemp)
        maxTemp = parcTemp;
      if (parcTemp < minTemp)
        minTemp = parcTemp;
      if (parcPres > maxPres)
        maxPres = parcPres;
      if (parcPres < minPres)
        minPres = parcPres;
      }
     numLectMedia++;
     if (numLectMedia >= (LECS_MEDIA+2))
     {
      numLectMedia = 0;
      nueva_lectura = 1;
      //Eliminar el máximo y el mínimo y dividir los totales por 15 para obtener la media acotada
      Temperatura = (sumTemp - maxTemp - minTemp) / LECS_MEDIA;
      
      Presion = (sumPres - maxPres - minPres) / 100.0;
      Presion = (Presion / LECS_MEDIA) + offset_Pres;
      //Se hace el cálculo de la Altitud según QNH
      //Altitud = 44330 * (1.0 - pow(Presion / QNH, 0.1902949));
      //El cálculo anterior se hace por partes por si hay desbordamientos o conversiones automáticas en cálculos intermedios
      adoub = Presion / QNH;
      adoub = pow(adoub, 0.1902949f); 
      adoub = 1.0f - adoub;
      Altitud = adoub * 44330.0f;
      //No se usa la siguiente instrucción, hace una nueva medida de presión por lo que no usaría la media anterior 
      //Altitud = bme.readAltitude(QNH + offset_Pres); 
     }
   }
 }

/********************************************************************************************************
actualizAgujas
      Coloca las dos agujas según el valor de la altitud en pies.
*********************************************************************************************************/
void actualizAgujas(void)
{
  double adoub;
  long along, al2;
  
  //La aguja corta marca 10000 pies en una vuelta completa, 720 pasos 
  if (Altitud < 0)
    adoub = 0;  //No procesar altitudes negativas
  else  
    adoub = (Altitud * FACTOR_M * STEPS) / F_ESCALA_C;
  along = round(adoub);
  while (along > STEPS)
    along = along - STEPS;  //no dar más de una vuelta
  //ir a posición correspondiente a aguja corta
  along = along - pos_corta;
  if (along != 0)
  {
    pos_corta = pos_corta + along;
    //para que estando ente 9 y 0 no de toda la vuelta `para ir a posición entre 0 y 1 y viceversa
    //calcular el camino más corto
    al2 = abs(along);
    if (abs(along) > abs(STEPS - al2))
    {
      al2 = STEPS - al2;
      if (along > 0)
        along = -1 * al2;
      else
        along = al2;
    }
    step_corta.step(along);
  }
  
  //La larga marca el resto de miles
  if (Altitud < 0)
    adoub = 0;  //No procesar altitudes negativas
  else  
    adoub = Altitud * FACTOR_M;
  along = round(adoub);
  along = along % F_ESCALA_L;
  along = (along * STEPS) / F_ESCALA_L;
  //ir a posición correspondiente
  along = along - pos_larga;
  if (along != 0)
  {
    pos_larga = pos_larga + along;
    //para que estando ente 9 y 0 no de toda la vuelta `para ir a posición entre 0 y 1 y viceversa
    //calcular el camino más corto
    al2 = abs(along);
    if (abs(along) > abs(STEPS - al2))
    {
      al2 = STEPS - al2;
      if (along > 0)
        along = -1 * al2;
      else
        along = al2;
    }

    step_larga.step(along);
  }
}

/***************************************************************************************
****************************************************************************************
Interrupciones encoder
****************************************************************************************
***************************************************************************************/

/**************************************************************************************
 * Interrupción para ajuste de offset aguja corta o larga según sea el valor de la
 * variable ajustando_corta
 *************************************************************************************/
void ajCortaEncoder() 
{
  int i; //, lecnva, lecant, lecDT;
  
  // Si CLK y DT son iguales está girando en sentido antihorario,
  // si son distintos gira a derecha. OJO, encoder ebay va al revés
  if (millis() > tmp_reb) //200 ms entre lecturas para evitar rebotes
  {   
    tmp_reb = millis();
    //esperar a a que se estabilice la señal, pequeño retardo con for, durante
    //interrupción no se actualiza millis()
    for (i = 0; i < 5000; i++)
      __asm__("nop\n\t");
    if (digitalRead(ENC_CLK) == digitalRead(ENC_DT))
    {
      if (ajustando_corta)
      {
        offset_corta--;
        ajus_corta--;  //No se llama a .step desde aquí, demasiado lento para llamar desde interrupción
      } else
      {
        offset_larga--;
        ajus_larga--;
      }
    }  
    else 
    {
      if (ajustando_corta)
      {
        offset_corta++;
        ajus_corta++;
      } else
      {
        offset_larga++;
        ajus_larga++;
      }
    }
  }
}
 
/**************************************************************************************
 * Interrupción para variación de QNH con encoder
 *************************************************************************************/
void qnhEncoder() 
{  
  int i; //, lecnva, lecant, lecDT;
  
  // Si CLK y DT son iguales está girando en sentido horario,
  // si son distintos gira a izquierda.
  //OJO con encoder de BOURNS cambia el sentido respecto al prototipo
  //100 ms entre lecturas para evitar rebotes y comprobar que la señal que ha provocado
  //la interrupción (ENC-CLK) sigue a nivel bajo, la interrupción es por flanco de bajada
  if ((millis() > tmp_reb) && (digitalRead(ENC_CLK) == LOW))                             
  {   
    tmp_reb = millis();
    //esperar a a que se estabilice la señal, pequeño retardo con for, durante
    //interrupción no se actualiza millis()
    for (i = 0; i < 5000; i++)
      __asm__("nop\n\t");
    if (digitalRead(ENC_CLK) == digitalRead(ENC_DT))
    { 
      if (ajustando_presion)
      {  //Se está ajustando la presión, no el QNH, limitar a +-30 mb
        offset_Pres = offset_Pres - 0.1;
        if (offset_Pres < -30.0)
          offset_Pres = -30.0;
      }
      else
      { //Limitar QNH al rango 900 a 1200
        QNH--;
        if (QNH < 900.0)
          QNH = 900.0;          
      }
    }
    else 
    {
      if(ajustando_presion)
      {  //Se está ajustando la presión, no el QNH, limitar a +-30 mb
        offset_Pres = offset_Pres + 0.1;
        if (offset_Pres > 30.0)
          offset_Pres = 30.0;
      }
      else
      { //Limitar QNH al rango 900 a 1200
        QNH++;
        if (QNH > 1200.0)
          QNH = 1200.0;          
      }
    }
  }
  msec_QNH = millis();  //anotar cambio de QNH
}
/***************************************************************************************
 * Fin interrupciones
 **************************************************************************************/

/***************************************************************************************
 funciones predefinidas setup() y loop()
***************************************************************************************/

void setup(void) 
{  
  //Solo para pruebas
  //Serial.begin(9600);
  
  pinMode(ZUMBA, OUTPUT);
  pinMode(S_HALL_L, INPUT_PULLUP); //Pull up en sensores para evitar resistencia externa
  pinMode(S_HALL_C, INPUT_PULLUP); 
  pinMode(ENC_CLK, INPUT); //En el circuito hay resistencia de pull up de 10k
  pinMode(ENC_DT, INPUT);  //En el circuito hay resistencia de pull up de 10k
  pinMode(ENC_SW, INPUT_PULLUP);  //No hay resistencia de pull up en el circuito
  digitalWrite(ZUMBA, LOW);  //Zumbador apagado
    
  // Rotar pantalla, va colocada al revés para que el conector no quede al borde del eje
  u8g.setRot180();
  // assign default color value
  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) 
    u8g.setColorIndex(255);     // white
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT )
    u8g.setColorIndex(3);         // max intensity
  else if ( u8g.getMode() == U8G_MODE_BW )
    u8g.setColorIndex(1);         // pixel on

  //Se inicia el sensor y se hace una lectura inicial
  sensorStart();
  if (error_ini)
   return; //no continuar si error en sensor
    
  iniMotores();
  pos_corta = 0;
  pos_larga = 0;
  interm = 0;  

  if ((!error_ini))
  {
    drawPpal();
    //Admitir interrupciones (las provocadas por el encoder) cuando ya se vea 
    //en pantalla el QNH que se estaría modificando
    attachInterrupt(0, qnhEncoder, FALLING);  //interrupción 0 por flanco - pin 2
  }
  rehab_zumba = millis();
  ms_habAajPres = millis();
  ms_LecSensor = millis();
  ajustando_presion = 0;
  nueva_lectura = 0;
  ciclo_son = 1;  //intermitencia de sonido activado para la primera vez
}

void loop(void) 
{  
  double adoub;
  
  if (!error_ini)
  {
    //Se hace lectura del sensor
    readSensor();   
    //cont++;
    if (!sonando) //Si está sonando el botón anula la alarma en lugar de entrar en pantalla de test
      modoTest = !digitalRead(ENC_SW); 
    if ((modoTest) || ajustando_presion)
    {
      if (ajustando_presion || ((millis() > (ms_habAajPres + TMP_AJ_PRES))))
      {  //En modo ajuste del offset de presión
        if (!ajustando_presion)
        {
          ajustando_presion = 1;
          display(0x1000);  //Mostrar, "Activado el ajuste", no usar error_ini ya que realmente no es un error
        }
        switch (ajustando_presion)
        {
          case 1:
            if (digitalRead(ENC_SW))  //esperar a que se deje de pulsar
            {
              ajustando_presion = 2;  //indicador de que se ha soltado, ahora se espera a que ajuste y pulse para garabar
              display(0x2000);  //Mostrar, "ajustar y pulsar para grabar", no usar error_ini ya que realmente no es un error
            }    
            break;
            
          case 2:
            display(0x2000);  //Mostrar, "ajustar y pulsar para grabar", actualizar offset por si se está girando el encoder
            if (!digitalRead(ENC_SW)) //esperar 2 lecturas iguales seguidas continuando en case 3, para evitar rebotes
              ajustando_presion = 3;
            break;
            
          case 3:  
            display(0x2000);  //Mostrar, "ajustar y pulsar para grabar",  actualizar offset por si se está girando el encoder
            if (!digitalRead(ENC_SW))
            {
              EEPROM.put(DIR_AJ_PRES, offset_Pres);
              ajustando_presion = 0; //Fin del proceso de ajuste de presión
              ms_habAajPres = millis();
            }
            break;
        } //switch
      }
      else
        drawTest();
    }
    else
    {
    ms_habAajPres = millis();
     if (nueva_lectura == 1)
      {
        drawPpal();

        //PRUEBAS CON SERIAL PLOTER
//        if ((PresionInicial == -999) && (PresionInicial == -999) && (PresionInicial == -999))
//        { //Primera lectura
//          PresionInicial = Presion*10.0;
//          AltitudInicial = Altitud;
//          TemperaturaInicial= Temperatura;
//        }      
//        adoub = Temperatura - TemperaturaInicial;
//        Serial.print(adoub+1);  //azul
//        Serial.print(" ");
//        adoub = Presion * 10.0 - PresionInicial;
//        Serial.print(adoub); //naranja
//        Serial.print(" ");
//        adoub = Altitud - AltitudInicial;
//        Serial.println(adoub-1); //rojo
        //FIN PRUEBAS
  
      }
    }
    //Actualizar estado agujas
    actualizAgujas();      
    //Si se ha cambiado el QNH grabar en EEPROM si permanece al menos 5 seg.
    if (grab_QNH != QNH)
    {  //El QNH ha cambiado, grabarlo si ha permanecido al menos 10 seg
      if (millis() > (msec_QNH  + 10000))  //ver si 10 seg
      {
        grab_QNH = QNH;
        EEPROM.put(DIR_QNH, grab_QNH);
      }
    }
    if (!anul_zumba)
    {  //La señal del zumbador no se ha desactivado, comprobar si se supera ALT_OXIGENO
      if (Altitud > ALT_OXIGENO)
      {
        if (sonando == 0)
          ms_interm = millis();  //es la primera vez, tomar referencia de tiempo
        if (millis() < (ms_interm + 750)) //sonido intermitente
        {
          digitalWrite(ZUMBA, HIGH);
        }
        else
        {
          if (ciclo_son == 1)  //pasado el tiempo y todavía en ciclo 1. Es porque no ha pasado en los 750 ms
          {
            digitalWrite(ZUMBA, LOW);

            ciclo_son = 0; //Es necesario "ciclo_son" ya que puede no pasar en 750 y 250 mS debido a que
                         //drawppal y otras funciones son lentas
          }
          
          if (millis() < (ms_interm + 1000))
          {
            digitalWrite(ZUMBA, LOW);
          }
          else
          {
            ms_interm = millis();  //iniciar ciclo sonido 0,75 + 0,25 seg
            if (ciclo_son == 0)  //pasado el tiempo y todavía en ciclo 0. Es porque no ha pasado en los 250 ms
            {
              digitalWrite(ZUMBA, HIGH);
              ciclo_son = 1;
            }
            else
              ciclo_son = 0;
          }
        }
        sonando = 1;
      }
      else
      {
        digitalWrite(ZUMBA, LOW);
        sonando = 0;
      }
      if ((sonando) && !digitalRead(ENC_SW))
      { //Si está sonando el botón anula la alarma en lugar de entrar en pantalla de test
        //Se tomaa el valor de millis() para reiniciar la alarma al cabo de 15 ó 30 min.
        rehab_zumba = millis();
        sonando = 0;
        anul_zumba = 1;  
        ciclo_son = 1;
        digitalWrite(ZUMBA, LOW);
      }
    }  //if (!anul_zumba)
    //Si han transcurrido TMP_ALARM mS volver a habilitar detección de altitud oxígeno
    if (millis() > (rehab_zumba + TMP_ALARM))
      anul_zumba = 0;
  }  //if (!error_ini)

//  //Prueba sensor e. hall
//  if (digitalRead(S_HALL_L) == HIGH)
//    digitalWrite(LED, HIGH);
//  else
//    digitalWrite(LED, LOW);
}
