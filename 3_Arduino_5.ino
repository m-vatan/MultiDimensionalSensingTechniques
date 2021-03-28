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

  // read the value from the sensor:
  sensorValue0 = analogRead(sensorPin0);
  sensorValue1 = analogRead(sensorPin1);
  sensorValue2 = analogRead(sensorPin2);
  sensorValue3 = analogRead(sensorPin3);
  //put them in an array
  int sensor_array [4] = {sensorValue0,sensorValue1,sensorValue2,sensorValue3};
  
  //calculate the Average of sensor values    
  sensorAverage = (sensor_array[0]+sensor_array[1]+sensor_array[2]+sensor_array[3])/4;

  //calculate the variance of sensor values
  float diff_sensor0 = sensor_array[0] - sensorAverage;
  float diff_sensor1 = sensor_array[1] - sensorAverage;
  float diff_sensor2 = sensor_array[2] - sensorAverage;
  float diff_sensor3 = sensor_array[3] - sensorAverage;
    
  sensorVariance = (sq(diff_sensor0) 
    + sq(diff_sensor1) 
    + sq(diff_sensor2) 
    + sq(diff_sensor3))/4;
  
  //Print the results
  Serial.print("Sensor 0 = ");
  Serial.print(sensorValue0); //prints the values coming from the sensor on the screen
  Serial.print("\t");
  
  Serial.print("Sensor 1 = ");
  Serial.print(sensorValue1);
  Serial.print("\t");
  
  Serial.print("Sensor 2 = ");
  Serial.print(sensorValue2);
  Serial.print("\t");
  
  Serial.print("Sensor 3 = ");
  Serial.print(sensorValue3);
  Serial.print("\t");

  Serial.print("Average = ");
  Serial.print(sensorAverage);
  Serial.print("\t");
  
  Serial.print("Variance = ");
  Serial.println(sensorVariance);
  
  delay(5000);   
}