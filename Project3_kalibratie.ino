#define GAS A4

void setup() {
  // put your setup code here, to run once:

  float sensor_volt = 0; //Define variable for sensor voltage 
  float RS_air; //Define variable for sensor resistance
  float R0; //Define variable for R0
  float sensorValue = 0; //Define variable for analog readings 
  
  for(int x = 0 ; x < 500 ; x++) //Start for loop 
  {
    sensorValue = sensorValue + analogRead(GAS); //Add analog values of sensor 500 times 
  }
  sensorValue = sensorValue/500.0; //Take average of readings
  sensor_volt = sensorValue*(5.0/1023.0); //Convert average to voltage 
  RS_air = ((5.0*10.0)/sensor_volt)-10.0; //Calculate RS in fresh air 
  R0 = RS_air/4.4; //Calculate R0 

  Serial.begin(9600); 
  Serial.println("Kalibratie MQ2 sensor"); 
  Serial.print("\tR0 = "); //Display "R0"
  Serial.println(R0); //Display value of R0 

}

void loop() {
  // put your main code here, to run repeatedly:

}
