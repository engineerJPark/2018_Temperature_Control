// tmp36온도 센서를 A0핀으로 설정합니다.
int tmpSensor1 = A0;
int tmpSensor2 = A1;
int tmpSensor3 = A2;

// 열선 패드를 5번 핀으로 설정합니다.
int heatingPad1 = 7;
int heatingPad2 = 6;
int heatingPad3 = 5;
int heatingPad4 = 4;

// 현재 설정 온도 값(유지시간)을 나타내는 변수입니다.
int tmpSetting = 0;


// tmp36센서가 측정한 현재 온도를 저장할 변수입니다.
float tmpCurrent1 = 0;
float tmpCurrent2 = 0;
float tmpCurrent3 = 0;
float tmpCurrentmean = 0;

// tmp36센서값으로 읽어온 수치를 계산하기 위해 임시적으로 필요한 변수입니다.
float tmpTemp1 = 0;
float tmpTemp2 = 0;
float tmpTemp3 = 0;

//PID제어 관련 변수들
//PID제어의 결정값
float P_control, I_control, D_control = 0;
float delaytime = 2000;
float PID_control = 0;

//P_control, I_control, D_control의 결정 값.
float Kp = -0.011460511;
float Ki = 0.0000035208;
float Kd = 52.98960896;

//PID parameter 측정에 필요한 변수. K는 80도 정도로 두면 될 듯 함. 필요하다면 더 높게
float K = 80;  //목표 온도

//error = 동작 신호, 편차
float error = 0;
float previous_error = 0;

void setup() {

  // 열선 패드1을 사용하기로 설정합니다. 
  pinMode(heatingPad1, OUTPUT);
  pinMode(heatingPad2, OUTPUT);
  pinMode(heatingPad4, OUTPUT);
  
  Serial.begin(9600);
}

void loop() {
  
  // 온도 계산부
  tmpTemp1 = (float)analogRead(tmpSensor1) * 5 / 1024;
  tmpCurrent1 = (100 * (tmpTemp1 - 0.5));
  tmpTemp2 = (float)analogRead(tmpSensor2) * 5 / 1024;
  tmpCurrent2 = (100 * (tmpTemp2 - 0.5));
  tmpTemp3 = (float)analogRead(tmpSensor3) * 5 / 1024;
  tmpCurrent3 = (100 * (tmpTemp3 - 0.5));
  tmpCurrentmean = (tmpCurrent1 + tmpCurrent1 + tmpCurrent1)/3;
  
  //PID 제어
  error = K - tmpCurrentmean;
  P_control = Kp * error;
  D_control = Kd * error*delaytime;
  I_control = Ki * (error - previous_error) / delaytime;
  PID_control = P_control + D_control + I_control;
  previous_error = error;

  if(PID_control > 0){
    tmpSetting = (int)(PID_control/1000);
    digitalWrite(heatingPad1, HIGH);
    digitalWrite(heatingPad2, HIGH);
    digitalWrite(heatingPad4, HIGH);
  }
  else{
    tmpSetting = (int)(PID_control/1000);
    tmpSetting = (-1)*tmpSetting;
    digitalWrite(heatingPad1, LOW);
    digitalWrite(heatingPad2, LOW);
    digitalWrite(heatingPad4, LOW);
  }
  
  //PID 상수 값들을 표현. 시리얼 통신을 이용하여.
  Serial.print("temperature1 : ");
  Serial.println(tmpCurrent1, 4);
  Serial.print("temperature2 : ");
  Serial.println(tmpCurrent2, 4);
  Serial.print("temperature3 : ");
  Serial.println(tmpCurrent3, 4);
  Serial.print("temperaturemean : ");
  Serial.println(tmpCurrentmean, 4);
  Serial.print("PID_control : ");
  Serial.println(PID_control, 10);
  Serial.println();
 
  delay(tmpSetting);
}
