#define LED_BUILTIN 2

void setup() 
{ 
  Serial.begin(9600); // sensor buart rate 
  pinMode(LED_BUILTIN, OUTPUT);  // Led Pin Connected To D5 Pin  
}  
void loop()  
{ 
 int s1=analogRead(A0); // IR Sensor output pin connected to A0 
 Serial.println(s1);  // See the Value In Serial Monitor    
 delay(100); 
 if(s1< 900 ) 
 { 
  digitalWrite(LED_BUILTIN,HIGH); // LED ON 
 } 
  else 
 { 
  digitalWrite(LED_BUILTIN,LOW); // LED OFF 
 } 
}  
