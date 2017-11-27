#include <QTRSensors.h>
#define NUM_SENSORS   8     // numero de sensores usados
#define TIMEOUT       2500  // esperar 2.5 ms para tener una respuesta del sensado
#define EMITTER_PIN   10     // este pin controla el led on del los sensores (enciende y apaga)
  
//aqui se pone el numero de los pines conectados a los sensores
QTRSensorsRC qtrrc((unsigned char[]) {19, 18, 17, 16, 15, 14, 11, 12},
NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

//variables para almacenar valores de sensores y posicion


int linea=0;
#define led1  13 
#define led2 2
#define mi1 5 //4
#define mi2 4 //3
#define pwmi 3 //6
#define md1 6 //8
#define md2 7 //7
#define pwmd 9 //5
#define boton_1 8 // 9
int position=0;
int salida_pwm=0;
int proporcional_pasado=0;
int velocidad=0;
int derivativo=0;
int proporcional=0;
int error=0;
int error_pasado=0;
int integral=0;
float kp=1;
float kd=0;
float ki=0;
void setup(){
  pinMode(mi1,OUTPUT);
  pinMode(mi2,OUTPUT);
  pinMode(md1,OUTPUT);
  pinMode(md2,OUTPUT);
  pinMode(13,OUTPUT);
   pinMode(led1, OUTPUT); //led1
 pinMode(led2, OUTPUT); //led2
 pinMode(boton_1, INPUT); //boton 1 como pull up
         
 for (int i = 0; i <40; i++)  //calibracion durante 2.5 segundos,
 {                                 //para calibrar es necesario colocar los sensores sobre la superficie negra y luego 
  digitalWrite(led1, HIGH);  //la blanca
  delay(20);
  qtrrc.calibrate();    //funcion para calibrar sensores   
  digitalWrite(led1, LOW);  
  delay(20);
 }
digitalWrite(led1, LOW); //apagar sensores para indicar fin de calibracion 
delay(400); 
digitalWrite(led2,HIGH); //encender led 2 para indicar la
while(true)
{
    int x=digitalRead(boton_1); //leemos y guardamos el valor
                                      // del boton en variable x
    delay(100);
    if(x==0) //si se presiona boton 
    {
        digitalWrite(led2,LOW); //indicamos que se presiono boton
        digitalWrite(led1,HIGH); //encendiendo led 1
        delay(100);
        break; //saltamos hacia el bucle principal
    }
}
}
void loop(){
  pid(linea,velocidad,kp,ki,kd); //funcion para algoritmo pid 
                                 //primer parametro: 0 para lineas negras, tipo 1 para lineas blancas
                                 //segundo parametro: velocidad pwm de 0 a 255
                                 //tercer parametro: constante para accion proporcional
                                 //cuarto parametro: constante para accion integral
                                 //quinto parametro: constante para accion derivativa
  //frenos_contorno(0,700);
  frenos_contorno(linea,700);
}

 void pid(int linea, int velocidad, float Kp, float Ki, float Kd)
{
  position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, linea); //0 para linea negra, 1 para linea blanca
  proporcional = (position) - 3500; // set point es 3500, asi obtenemos el error
  integral=integral + proporcional_pasado; //obteniendo integral
  derivativo = (proporcional - proporcional_pasado); //obteniedo el derivativo
  if (integral>1000) integral=1000; //limitamos la integral para no causar problemas
  if (integral<-1000) integral=-1000;
  salida_pwm =( proporcional * Kp ) + ( derivativo * Kd )+(integral*Ki);
   
  if (  salida_pwm > velocidad )  salida_pwm = velocidad; //limitamos la salida de pwm
  if ( salida_pwm < -velocidad )  salida_pwm = -velocidad;
   
  if (salida_pwm < 0)
 {
  motores(velocidad+salida_pwm, velocidad);
 }
 if (salida_pwm >0)
 {
  motores(velocidad, velocidad-salida_pwm);
 }
 
 proporcional_pasado = proporcional;  
}
 
void motores(int motor_izq, int motor_der){
  if(motor_izq >= 0 ){
    digitalWrite(mi1,LOW);
    digitalWrite(mi2,HIGH); // con high avanza
    analogWrite(pwmi,motor_izq); //se controla de manera
                                        //inversa para mayor control
 }else{
    digitalWrite(mi1,HIGH);
    digitalWrite(mi2,LOW); // con high avanza
    motor_izq=motor_izq*(-1);
    analogWrite(pwmi,motor_izq); //se controla de manera
 }
 if(motor_der >= 0 ){ //motor derecho
    digitalWrite(md1,LOW);
    digitalWrite(md2,HIGH); // con high avanza
    analogWrite(pwmd,motor_der); //se controla de manera 
 }else{
    digitalWrite(md1,HIGH);
    digitalWrite(md2,LOW);
    motor_der=motor_der*(-1);
    analogWrite(pwmd,motor_der); //se controla de manera
 } 
}

void frenos_contorno(int tipo,int flanco_comparacion)
{
   
if(tipo==0)
{
  if(position<=50) //si se salio por la parte derecha de la linea
 {
  motores(-80,90); //debido a la inercia, el motor 
                                  //tendera a seguri girando
                                  //por eso le damos para atras , para que frene
                                 // lo mas rapido posible 
  while(true)  
  {
   qtrrc.read(sensorValues); //lectura en bruto de sensor   
   if( sensorValues[0]>flanco_comparacion || sensorValues[1]>flanco_comparacion ) 
   //asegurar que esta en linea
   {
    break;
   } 
  }
 }
 
 if (position>=6550) //si se salio por la parte izquierda de la linea
 { 
  motores(90,-80);
  while(true)
  {
   qtrrc.read(sensorValues);
   if(sensorValues[7]>flanco_comparacion || sensorValues[6]>flanco_comparacion )
   {
    break;
   }  
  }
 }
}
 
if(tipo==1) //para linea blanca con fondo negro
{
 if(position<=50) //si se salio por la parte derecha de la linea
 {
  motores(-80,90); //debido a la inercia el motor 
                   //tendera a seguri girando
                   //por eso le damos para atras  
                   //para que frene lo mas rapido posible 
  while(true)  
  {
   qtrrc.read(sensorValues); //lectura en bruto de sensor
   if(sensorValues[0]<flanco_comparacion || sensorValues[1]<flanco_comparacion )   //asegurar que esta en linea
   {
    break;
   }
  }
 }
 
 if(position>=6550) //si se salio por la parte izquierda de la linea
 { 
  motores(90,-80);
  while(true)
  {
   qtrrc.read(sensorValues);
   if(sensorValues[7]<flanco_comparacion || sensorValues[6]<flanco_comparacion)
   {
    break;
   }  
  }
 }
}
}


