int sensorPin = 0;
int greenLED =2;
int redLED = 3;

void setup() {
  pinMode(greenLED , OUTPUT);
  pinMode(redLED , OUTPUT);
  Serial.begin(9600);

}

void loop() {
  int sensorValue = analogRead(sensorPin);
  float temp = sensorValue * 0.48828125;

  Serial.print("Temperature :");
  Serial.println(temp);
  delay(1000);

  if(temp>=49){
    digitalWrite(redLED,HIGH);
    digitalWrite(greenLED,LOW);
  }else{
    digitalWrite(redLED,LOW);
    digitalWrite(greenLED,HIGH);
  }








  


  

}
