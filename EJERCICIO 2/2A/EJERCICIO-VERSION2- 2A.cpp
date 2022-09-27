// GRUPO 3 -SENSORES Y ACTUADORES  punto 2A VERSION 2

// se utiliza una libreria -para la configuracion del motor Dc y su posterior control de velocidad y posicion -direccion con ENCODER y PWM  : ATOMIC_BLOCK macro.
#include <util/atomic.h>
// defino los pines de conexion del encoder y el pulsador
#define ENCODER_A       2 // 
#define ENCODER_B       3 // 
#define PULSADOR     4
// CONSTANTE ENTERA ASOCIADA AL Pin del Potenciometro PARA LECTURA DEL VALOR ANALOGICO QUE PERMITIRA CONTROLAR LA VELOCIDAD Y LA POSICION POR PWM
const int pot = A0;
//DEFINO CONSTANTES ENTERAS
// Pines de Control QUE SE CONTECTAN A ld293d
const int E1Pin = 10; // CONTROL PWM DE LA VELOCIDAD
const int M1Pin = 12; // PIN DE CONTROL DE LA DIRECCION Y POSICION 

//Variable global de posición compartida con la interrupción
volatile int theta = 0;
//Variable global de pulsos compartida con la interrupción
volatile int pulsos = 0;
unsigned long timeold;
// VARIABLE DE TIPO REAL PARA LA CALCULO DE LA VELOCIDAD EN RPM A PARTIR DE LOS PULSOS MEDIDOS 
float resolucion = 374.22;
//Variable Global Velocidad DONDE SE ALMACENARA LA VELOCIDAD A LA QUE VA A GIRAR EL MOTOR
int vel = 0;
//Variable Global Posicion DONDE SE ALMACENARA LA VELOCIDAD A LA QUE VA A GIRAR EL MOTOR
int ang = 0;
//Variable Global MODO DE TRABAJO SI ES CONTROL DE VELOCIDAD O POSICION 
bool modo = false;
//Estructura del Motor ASOCIADA A LA LIBRERIA
typedef struct{
  byte enPin;// PARA CONFIGRURAR LA VELOCIDAD DEL MOTOR 
  byte directionPin; // PARA CONFIGURAR LA DIRECCION DEL MOTOR
}Motor;
//Creo el OBJETO motor A PARTIR DE LA LIBRERIA UTILIZADA
const Motor motor = {E1Pin, M1Pin};
//Constantes de dirección del Motor
const int ADELANTE = LOW;
const int ATRAS = HIGH;
void setup(){
  // establezca el divisor del temporizador 1 en 1024 para una frecuencia PWM de 30,64 Hz
  TCCR1B = TCCR1B & B11111000 | B00000101;
  // CONFIGURO LA VELOCIDAD DE COMUNICACION SERIAL Y LA INICIALIZO
  Serial.begin(9600);
  //DEFINO LOS PINES DEL Encoders como entradas
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  //DEFINO EL PIN DEL PULSADOR COMO ENTRADA
  pinMode(PULSADOR, INPUT);
  //Configura Motor, SUS PINES COMO SALIDA
  pinMode(motor.enPin, OUTPUT);
  pinMode(motor.directionPin, OUTPUT);
  //Configurar Interrupción CON EL PIN DEL DEL ENCODER
  timeold = 0;
  attachInterrupt(digitalPinToInterrupt(ENCODER_A),leerEncoder,RISING);
}
void loop(){
  // DEFINO VARIABLES LOCALES DE LA FUNCION LOOP
  float posicion;// VARIABLE QUE ALMACENA LA POSICION
  float rpm; // VARIABLE QUE ALMACENA LA VELOCIDAD EN RPM
  int valor ,dir=true;// VARIABLES QUE SE USAN PARA ALMACENAR LA LECTURA DEL VALOR ANALOGICO DEL POTENCIOMETRO Y LA OTRA QUE DEFINE LA DIRECCION 
  //Lee el Valore del Potenciometro
  valor = analogRead(pot);
  
  //Cambia de Modo PARA HACER CONTROL DE LA Velociadad o DE LA  Posición CUANDO SE PRESIONA EL PULSADOR
  if(debounce(PULSADOR)){
    modo = !modo;// CAMBIO EL MODO DE TRABAJO CADA VEZ QUE SE PRESIONE EL PULSADOR
    theta = 0;// VARIABLE INICIALIZADA EN 0
  }
  // SEGUN LA CONDICION DEFINO EL MODO DE TRABAJO - PARA CONTROL DE VELOCIDAD O DE POSICION 
  if(modo){
    //Transforma el valor del Pot a velocidad  A TRAVES DE LA FUNCION MAPEO ENTRE UNA ENTRADA ANALOGICA Y UNA SALIDA ANALOGICA PWM
    vel = map(valor,0,1023,0,255);
    //Activa el motor dirección ADELANTE con la velocidad
    setMotor(motor, vel, false);
    //Espera un segundo para el calculo de las RPM
    if (millis() - timeold >= 1000)
    {
      //Modifica las variables de la interrupción forma atómica
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        //rpm = float(pulsos * 60.0 / 374.22); //RPM
        rpm = float((60.0 * 1000.0 / resolucion ) / (millis() - timeold) * pulsos);
        timeold = millis();
        pulsos = 0;
      }
      // SE ENVIA LOS VALORES DE VELOCIDAD EN RPM Y DE PWM POR EL MONITOR SERIE
      Serial.print("RPM: ");
      Serial.println(rpm);
      Serial.print("PWM: ");
      Serial.println(vel);
    }
  }
  
  else{
    //Transforma el valor del Pot a ángulo A TRAVES DE LA FUNCION MAPEO ENTRE UNA ENTRADA ANALOGICA Y EL ANGULO DE GIRO DEL MOTOR DE 0° A 360°
    ang = map(valor,0,1023,0,360);  
    //Modifica las variables de la interrupción forma atómica
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      // CALCULAR LA POSICION DEL MOTOR 
      posicion = (float(theta * 360.0 /resolucion));
    }
    //Posiciona el ángulo con tolerancia +- 2 , segun la condicion que se cumpla
    if(ang > posicion+2){
      vel = 255;// VELOCIDAD MAXIMA
      dir = true;
    }
    else if(ang < posicion-2){
      vel = 255;// VELOCIDAD MAXIMA
      dir = false;
    }
    else{
      vel = 0;
    }
    setMotor(motor, vel, dir);
  }
}
//Función para dirección y velocidad del Motor
void setMotor(const Motor motor, int vel, bool dir){
  analogWrite(motor.enPin, vel);// envio la velocidad que debe girar el motor
  if(dir)
    digitalWrite(motor.directionPin, ADELANTE);// envio la direccion del motor adelante
  else
    digitalWrite(motor.directionPin, ATRAS);// envio la direccion del motor atras
}
//Función anti-rebote para asegurar el nivel de lectura, alto o bajo
bool debounce(byte input){
  bool estado = false;
  if(! digitalRead(input)){
    delay(200);
    while(! digitalRead(input));
    delay(200);
    estado = true;
  }      
  return estado;   
}
//Función para la lectura del encoder
void leerEncoder(){
  //Lectura de Velocidad
  if(modo)
    pulsos++; //Incrementa una revolución
    
  //Lectura de Posición  
  else{
    int b = digitalRead(ENCODER_B);
    if(b > 0){
      //Incremento variable global
      theta++;
    }
    else{
      //Decremento variable global
      theta--;
    }
  }
}