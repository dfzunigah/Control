#include <SoftwareSerial.h>

SoftwareSerial bluetoothSerial(0,1);  // RX, TX pins para el módulo HC-05


const int sensorPin = 2;  // Pin digital conectado al módulo FC-03
volatile int pulsaciones = 0;

// Pins del controlador L298N
const int motorEnA = 10;
const int motorPin1 = 9;
const int motorPin2 = 8;

//Definir variables
float y;
float y_0 = 0;
const float d = 0.88; // cm

// PID control
float delta_y = 0;
int Set_point;
bool PID = false;
double kp = 2;   // Constante proporcional
double ki = 4;   // Constante integral
double kd = 0.1;   // Constante derivativa
double error = 0.0;
double lastError = 0.0;
double integral = 0.0;
double derivative = 0.0;
double output = 0.0;
float PID_value;

// Datos
int dato = 0;
int direccion = 1;
//Mediciones
unsigned long currentTime;
unsigned long previousTime = 0;
double deltaTime;

void contarPulsaciones()
{
  pulsaciones++;
}

void setup()
{
  // Controlar motor
  pinMode(motorEnA, OUTPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);

  //FC-03 calcular pulsaciones
  pinMode(sensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensorPin), contarPulsaciones, FALLING);

  // Inicializar conexión bluetooth y serial
  bluetoothSerial.begin(9600);
  Serial.begin(9600);

}

void loop()
{
  // Calcular posición en Y

  
  // Leer datos del puerto Bluetooth

 
  if (bluetoothSerial.available() > 0) {
    
    dato = bluetoothSerial.read();
    //Serial.print("Dato recibido: ");
    //Serial.println(dato);

    if (dato >= 0){
      Set_point = dato;      
    }

    if (dato == 250){
      PID = true;      
    } 
    else if (dato == 251){
      PID = false;
    }    
  }


  // Enviar datos al puerto Bluetooth
  // bluetoothSerial.print(y);

  if (PID == true){ 

  currentTime = millis();
  deltaTime = (currentTime - previousTime) / 1000.0;   // Tiempo transcurrido en segundos
  if (deltaTime >= 0.01) {   // Frecuencia de muestreo de 100 H
    error = Set_point - y;   // Calcular el error
    integral += error * deltaTime;   // Calcular la parte integral
    derivative = (error - lastError) / deltaTime;   // Calcular la parte derivativa
    output = kp * error + ki * integral + kd * derivative;   // Calcular la salida del controlador

    // El motor arranca con por lo menos el 30% de la señal, y el rango de señal es de 0 a 255
    PID_value = map(abs(output)*40 + 60 , 0, 100, 0, 255)*(output/abs(output)); 

    if (PID_value >= 255){
      PID_value = 255;

    }
    if (error <= -0.1){
      // Girar en sentido horario
      motorControl(HIGH, LOW, PID_value);  // Velocidad máxima (0-255)
      direccion = -1; 
      y = y_0 + (direccion*pulsaciones*d*3.1415)/20;
      y_0 = y;
      byte val = y;
      pulsaciones = 0;
      Serial.println(y);
      bluetoothSerial.write(val)
    }

    else if (error >= 0.1){
      // Girar en sentido antihorario
      Serial.println(true); 
      motorControl(LOW, HIGH, PID_value);  // Velocidad máxima (0-255)
      direccion = 1; 
      y = y_0 + (direccion*pulsaciones*d*3.1415)/20;
      y_0 = y;
      byte val = y;
      pulsaciones = 0;
      Serial.println(y);
      bluetoothSerial.write(val)
    }

    else{
      motorControl(LOW, LOW, 0); //Apagar motor
    }

    lastError = error;
    previousTime = currentTime;
    }
  
  }

  else{
    y_0 = 0;
    pulsaciones=0;
    // Girar en sentido antihorario, es decir hacia arriba
    if (Set_point == 1){
      Serial.println(y);
      motorControl(HIGH, LOW, 255);    
    } 
    // Girar en sentido horario, es decir hacia abajo
    else if (Set_point == 2){
      Serial.println(y);
      motorControl(LOW, HIGH, 255); 
      
    }
    else{
      motorControl(LOW, LOW, 0); //Apagar motor
    }   
  }
  
}

void motorControl(int motorDir1, int motorDir2, int motorSpeed) {
  digitalWrite(motorPin1, motorDir1);
  digitalWrite(motorPin2, motorDir2);
  analogWrite(motorEnA, motorSpeed);
}
