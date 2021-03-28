int sensorPin0 = A0;
int sensorPin1 = A1;
int sensorPin2 = A2;
int sensorPin3 = A3;

int sensorValue0 = 0;
int sensorValue1 = 0;
int sensorValue2 = 0;
int sensorValue3 = 0;

int Average0 = 0;
int Average1 = 0;
int Average2 = 0;
int Average3 = 0;

unsigned long Variance0 = 0;
unsigned long Variance1 = 0;
unsigned long Variance2 = 0;
unsigned long Variance3 = 0;

int sensorArray0 [99] = {};
int sensorArray1 [99] = {};
int sensorArray2 [99] = {};
int sensorArray3 [99] = {};


long sum0 = 0;
long sum1 = 0;
long sum2 = 0;
long sum3 = 0;


long sum_diff0 = 0;
long sum_diff1 = 0;
long sum_diff2 = 0;
long sum_diff3 = 0;

void setup() {
  Serial.begin(9600); //sets serial port for communication
}

void loop()
{
  // read the value from the sensor A0:
  for(int i=0; i<99; i++){
    sensorValue0 = analogRead(sensorPin0);
    sensorArray0[i] = sensorValue0;
    Serial.print(" 0 = ");
    Serial.println(sensorArray0[i]);
    sum0 += sensorArray0[i];
    delay(100);
  }
  Average0 = sum0 / 99;
  
  sum0 = 0;
  long x0 = 0;
  for(int ii=0; ii<99; ii++){
    delay(100);
    x0 += sensorArray0[ii] - Average0;
    sum_diff0 += sq(x0);
    delay(100);
  }
  Variance0 = sum_diff0 / 99;
  sum_diff0 = 0;
  
  Serial.print("  Average 0 = ");
  Serial.println(Average0);
  Serial.print("  Variance 0 = ");
  Serial.println(Variance0);
   
  // read the value from the sensor A1:
  for(int j=0; j<99; j++){
    sensorValue1 = analogRead(sensorPin1);
    sensorArray1[j] = sensorValue1;
    Serial.print(" 1 = ");
    Serial.println(sensorArray1[j]);
    sum1 += sensorArray1[j];
  }
  Average1 = sum1 / 99;
  
  sum1 = 0;
  long x1 = 0;
 // sum_diff0 = 0;
  for(int jj=0; jj<99; jj++){
    x1 += sensorArray1[jj] - Average1;
    sum_diff1 += sq(x1);
  }
  Variance1 = sum_diff1 / 99;
  sum_diff1 = 0;
  
  
  Serial.print("  Average 1 = ");
  Serial.println(Average1);
  Serial.print("  Variance 1 = ");
  Serial.println(Variance1);
  
  
  
    // read the value from the sensor A2:
  for(int k=0; k<99; k++){
    sensorValue2 = analogRead(sensorPin2);
    sensorArray2[k] = sensorValue2;
    Serial.print(" 2 = ");
    Serial.println(sensorArray2[k]);
    sum2 += sensorArray2[k];
  }
  Average2 = sum2 / 99;
  
  sum2 = 0;
  long x2 = 0;
  for(int kk=0; kk<99; kk++){
    x2 += sensorArray2[kk] - Average2;
    sum_diff2 += sq(x2);
  }
  Variance2 = sum_diff2 / 99;
  sum_diff2 = 0;
  
  Serial.print("  Average 2 = ");
  Serial.println(Average2);
  Serial.print("  Variance 2 = ");
  Serial.println(Variance2);
  
  
  
    // read the value from the sensor A3:
  for(int m=0; m<99; m++){
    sensorValue3 = analogRead(sensorPin3);
    sensorArray3[m] = sensorValue3;
    Serial.print(" 3 = ");
    Serial.println(sensorArray3[m]);
    sum3 += sensorArray3[m];
  }
  Average3 = sum3 / 99;
  
  sum3 = 0;
  long x3 = 0;
  for(int mm=0; mm<99; mm++){
    x3 += sensorArray3[mm] - Average3;
    sum_diff3 += sq(x3);
  }
  Variance3 = sum_diff3 / 99;
  sum_diff3 = 0;
  
  Serial.print("  Average 3 = ");
  Serial.println(Average3);
  Serial.print("  Variance 3 = ");
  Serial.println(Variance3);
  
  delay(1000);   
}