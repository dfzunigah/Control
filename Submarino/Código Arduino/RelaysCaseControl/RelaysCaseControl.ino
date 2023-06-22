// set pin numbers
const int releApagado = 4;     // the number of the pushbutton pin
const int releDireccion =  5;       // the number of the LED pin

void setup() {
  // initialize the pushbutton pin as an input
  pinMode(releApagado, OUTPUT);
  // initialize the LED pin as an output
  pinMode(releDireccion, OUTPUT);
  Serial.begin(9600); 
}

void loop() {
  if (Serial.available()) {
    String incomingString = Serial.readString();
    incomingString.trim();
    Serial.print("Se recibió: ");
    Serial.println(incomingString);

    if(incomingString.equals("1")){
      Serial.println("Opción 1: LOW LOW");
      digitalWrite(releApagado, LOW);
      digitalWrite(releDireccion, LOW);

    }else if(incomingString.equals("2")){
      Serial.println("Opción 2: LOW HIGH");
      digitalWrite(releApagado, LOW);
      digitalWrite(releDireccion, HIGH);

    }else if(incomingString.equals("3")){
      Serial.println("Opción 3: HIGH LOW");
      digitalWrite(releApagado, HIGH);
      digitalWrite(releDireccion, LOW);

    }else if(incomingString.equals("4")){
      Serial.println("Opción 4: HIGH HIGH");
      digitalWrite(releApagado, HIGH);
      digitalWrite(releDireccion, HIGH);

    }else{
      Serial.println("Otra opción");
    }
  }
}
