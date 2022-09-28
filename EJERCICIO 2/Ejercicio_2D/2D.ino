// grupo 3 -SENSORES Y ACTUADORES 2022
// EJERCICIO 2 D
// DECLARACION DE VARIABLES GLOBALES
int Sensor_derecha = 8;
int Sensor_izquierda = 9; 
int Motor_izquierdo = 6; 
int Motor_derecho = 7; 
void setup() {
  // configurar los pines que se conectan con la placa arduino 
  pinMode(Sensor_derecha, INPUT);// 
  pinMode(Sensor_izquierda, INPUT);// 
  pinMode(Motor_izquierdo, OUTPUT);// 
  pinMode(Motor_derecho, OUTPUT);// 
}

void loop() {
 int sensorD;// variables para almacenar las lecturas de los sensores
 int sensorI;
 sensorD=digitalRead(Sensor_derecha);//se realiza lectura del sensor de la derecha
 sensorI=digitalRead(Sensor_izquierda);//se realiza lectura del sensor de la izquierda
 if ( sensorD==LOW)// preguntamos la condicion del sensor, si esta en nivel bajo
 { digitalWrite(Motor_izquierdo,LOW);// manda nivel bajo al motor izquierdo
}
 else 
{
  digitalWrite(Motor_izquierdo,HIGH);// manda nivel alto al motor izquierdo
 }
 if ( sensorI==LOW)               // consulta condicion del otro sensor y actua de la misma manera
 { digitalWrite(Motor_derecho,LOW);
}
 else 
{
  digitalWrite(Motor_derecho,HIGH);
 }
}
