int sensorPin0 = A0;
int sensorPin1 = A1;
int sensorPin2 = A2;
int sensorPin3 = A3;

int sensorValue0 = 0;
int sensorValue1 = 0;
int sensorValue2 = 0;
int sensorValue3 = 0;


void setup() {
  Serial.begin(9600); //sets serial port for communication
}



void loop()
{
  float sensorAverage = 0.0;
  float sensorVariance = 0.0;

  // read the value from the 4 sensors:
  sensorValue0 = analogRead(sensorPin0);
  sensorValue1 = analogRead(sensorPin1);
  sensorValue2 = analogRead(sensorPin2);
  sensorValue3 = analogRead(sensorPin3);

  //Print the results
  Serial.print("SensorValue 0 = ");
  Serial.print(sensorValue0); //prints the values coming from the sensor on the screen
  Serial.print("\t");
  
  Serial.print("SensorValue 1 = ");
  Serial.print(sensorValue1);
  Serial.print("\t");
  
  Serial.print("SensorValue 2 = ");
  Serial.print(sensorValue2);
  Serial.print("\t");
  
  Serial.print("SensorValue 3 = ");
  Serial.println(sensorValue3);


 
  delay(5000);   
}