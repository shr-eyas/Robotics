// Current sensor pins
#define RIGHT_CURRENT 35
#define LEFT_CURRENT 32

// Global varibles
const float Kt = 0.11;
float raw1 = 0;
float raw2 = 0;
float initialVolt1 = 0;
float initialVolt2 = 0;
volatile float rightCurrent = 0;
volatile float leftCurrent = 0;

// Kalman Variables
volatile float measurement1 = 0;
volatile float estimate1 = 0;
volatile float measurementVariance1 = 0.5;
volatile float estimateVariance1 = 0.3;
volatile float estimateMeasurementWeight1 = 0.001;
volatile float kalmanGain1;
volatile float oldEstimate1;

volatile float measurement2 = 0;
volatile float estimate2 = 0;
volatile float measurementVariance2 = 0.5;
volatile float estimateVariance2 = 0.3;
volatile float estimateMeasurementWeight2 = 0.001;
volatile float kalmanGain2;
volatile float oldEstimate2;

void setup() {
  Serial.begin(115200); 

  pinMode(RIGHT_CURRENT, INPUT);
  pinMode(LEFT_CURRENT, INPUT);
}

void loop() {
  raw1 = analogRead(RIGHT_CURRENT);
  raw2 = analogRead(LEFT_CURRENT);

  for(int i=0; i<500; i++){
  initialVolt1 = initialVolt1 + 3.3*(raw1/4096);
  }
  initialVolt1 = initialVolt1/500;

  for(int i=0; i<500; i++){
    initialVolt2 = initialVolt2 + 3.3*(raw2/4096);
  }
  initialVolt2 = initialVolt2/500;

  float voltage1 = 3.3*(raw1/4096);
  float voltage2 = 3.3*(raw2/4096);

  float current1 = (initialVolt1 - voltage1)*10 - 0.035;
  float current2 = (initialVolt2 - voltage2)*10 - 0.035;

  // Kalman Filtering
  oldEstimate1 = estimate1;
  measurement1 = current1; 
  kalmanGain1 = estimateVariance1/(estimateVariance1 + measurementVariance1); 
  estimate1 = estimate1 + kalmanGain1*(measurement1-estimate1); 
  estimateVariance1 = (1 - kalmanGain1)*(estimateVariance1) + fabs(oldEstimate1 - estimate1)*estimateMeasurementWeight1; 

  oldEstimate2 = estimate2;
  measurement2 = current2; 
  kalmanGain2 = estimateVariance2/(estimateVariance2 + measurementVariance2); 
  estimate2 = estimate2 + kalmanGain2*(measurement2 - estimate2);
  estimateVariance2 = (1 - kalmanGain2)*(estimateVariance2) + fabs(oldEstimate2 - estimate2)*estimateMeasurementWeight2; 

  rightCurrent = estimate1;
  leftCurrent = estimate2;
  
  Serial.print(rightCurrent);
  Serial.print(" ");
  Serial.println(leftCurrent);
}
