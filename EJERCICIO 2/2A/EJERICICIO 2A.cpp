// GRUPO 3 SENSORES Y ACTUADORES .  PUNTO 2A
// defino variables 
// variables que uso para el encoder
int  ENCODER_A=3;
int  ENCODER_B=5;
int pot = A0;// pin del potenciometro
long i = 0;// valor inicial de la variable entera i
int M1=HIGH,M2=LOW; // valores que definen la direccion del motor
int value; // variable para almacenar el estado del pulsador
int IN1 = 6;// defino el pin del driver LD293 que se conecta al arduino para controlar el sentido de giro del motor
int IN2 = 7; //defino el pin del driver LD293 que se conecta al arduino para controlar el sentido de giro del motor
//Variable Global Velocidad
int vel = 0;
//Variable Global del pin de control de la velocidad 
int EN1=10 ;
//
int valor =0; // incializo la variable que almacena la lectura del potenciometro


// funcion de configuracion
void setup() {
  // configuro la interrupcion provocada por el pulsador en pin 2
  attachInterrupt(0,turn,RISING); // interrupt 0
  attachInterrupt(1,sub,RISING); // interrupt 1
// configuro los pines que manejan el sentido de giro del motor como salida
  pinMode(IN1,OUTPUT); // pines del driver LD293 y un pin del motor
  pinMode(IN2,OUTPUT); // pines del driver LD293 y el otro pin del motor 
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
   // inicializo la comunicacion serial a 9600
  Serial.begin(9600);
  
}
// funcion principal
void loop() {
  // codigo que se ejecuta 
   // envio un mensaje al monito serie
  Serial.print("ENCODER  ");
  
  // defino el sentido de giro del motor 
  digitalWrite(IN1,M1);
  digitalWrite(IN2,M2);
   //Lee el Valore del Potenciometro
  valor= analogRead(pot);
  //Transforma el valor del Pot a velocidad con la funcion mapeo
  vel = map(valor,0,1023,0,255);
  // envio un mensaje al monito serie, el valor convertido a salida analogica
  Serial.print("PWM: ");
  Serial.println(vel);
  // envio ese valor al pin de control de velocidad del LD293 
  analogWrite(EN1,vel);
}
// funciones que responden a las interrupciones
void turn(){ // configuracion de la funcion interrupcion ocurrida por el  pin 2

  M1=!M1;
  M2=!M2;
}

void sub(){ //interrupcion ocurrida por el pin 3
// leo el estado del sensor y verifico la direccion del motor
  value = digitalRead(ENCODER_A);
  // condicional que verifica el sentido de giro del motor
  if(value==1){ // pulsador no presionado
    i++;
  }
  else // pulsador presionado
  {
    i--;
  }
}