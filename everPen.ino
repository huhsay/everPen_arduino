

/*
 * 2018.05.12 
 * 화중 합치는 중
 * 블루투스 통신 정상
 * 가속도 센서 정상
 * 압력 센서 정상
 * 
 * 압력센서 동작별로 상태 나눠놓음
 * readFsr() 함수에 리턴값으로 정리
 * 
 */

#include<Wire.h>
#include<SoftwareSerial.h> // bluetooth

// pin설정

#define buttonPin 2
#define ledPin 13 
#define btTxPin 4 //송신핀
#define btRxPin 3 //수신핀
#define fsrPin A0 // 압력센서

SoftwareSerial BTSerial(btTxPin, btRxPin);


const int MPU=0x68; //MPU 6050 의 I2C 기본주소
const int MPU2=0x89; //MPU 6050

int pressS; // 눌린 상태
int prePressS=0;
boolean sw = false; // 펜 산태
int fsrValue; // 압력 센서 값
int time=0;
long DELAY = 100;


// 가속도 코드
const int max =1;
const float min_data = 1000.0;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float baseAcX=0, baseAcY=0, baseAcZ=0;
float before_AcX, before_AcY, before_AcZ;
float before_SpX, before_SpY, before_SpZ;
float dt;
float timeGap, acXGap,acYGap;
float spXGap=0, spYGap=0;
float spX=0, spY=0;
int count=0;
float X=0,Y=0;

unsigned long t_now;
unsigned long t_prev;


void setup() {

  //기타 초기화
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  pinMode(fsrPin, INPUT);
  attachInterrupt(0, changeState, FALLING); // 버튼 인터럽트
  BTSerial.begin(9600);
  Serial.begin(115200);


  // 가속도 센서 초기화
  Wire.begin(); //Wire 라이브러리 초기화
  Wire.beginTransmission(MPU); //MPU로 데이터 전송 시작
  Wire.write(0x6B); //
  Wire.write(0);
  Wire.endTransmission(true); // mpu-6050 시작

  initDT();

}

void loop() {
    
  if(sw){

    // 스마트펜이 ON 상태일 때

    int p = readFsr();

    initDT();
    calibAccelGyro();
    calcDT();
    calculate();
    delay(5);
    String btStr =String(X) + " " + String(Y) +" "+String(p)+"\n";
    BTSerial.println(btStr);


    Serial.println(btStr);

    // 배열에 각 값 저장
    
  }else{
    
    // off 인상태
    
  }
}

void changeState() {
  
  _delay_ms(100);

  if(digitalRead(buttonPin)==LOW){
    sw = !sw;

    if(sw==true){
      digitalWrite(ledPin, HIGH);
      Serial.println("on");
    }else{
      digitalWrite(ledPin, LOW);
      Serial.println("off");     
    }
  }
}


int readFsr(){

  fsrValue = analogRead(fsrPin);

  if(fsrValue > 500){
    pressS = 1;
  }
  else{
    pressS = 0;
  }

  int s =0;
  
  if(prePressS == 0 && pressS ==0){
    s = 0; // 공중
  }
  else if (prePressS == 0 && pressS ==1){
    // 획 시작
    s =1;
    
  }else if(prePressS == 1 && pressS ==0){
    // 획 끝
    s = 3;
  }else if(prePressS == 1 && pressS == 1){
    // 획 중간
    s = 2;
  }

  prePressS = pressS;

  return s;
  
}


void readAccelGyro(){  //센서값 리드
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=-Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();

  if(AcX < min_data && AcX > - min_data ){
    AcX =0;
  }

  if( AcY < min_data && AcY > - min_data){
    AcY =0;
  }
}


void calibAccelGyro(){  // 튀는값 보정
  float sumAcX = 0, sumAcY = 0, sumAcZ = 0;
  before_AcX = baseAcX;
  before_AcY = baseAcY;
  before_AcZ = baseAcZ;

  //읽어드렸으면 이제 읽어드린 값을 토대로 평균값을 구하면 됨
  for(int i=0; i<max; i++){

    readAccelGyro();

    sumAcX += AcX; 
    sumAcY += AcY;
    sumAcZ += AcZ;

    // 더할때 튀는값을 방지하기위해 가장 큰값 2개 작은값 2개를 빼고 6개로 나누기

    delay(1);//0.03초
  }


  //맨 처음 기본 센서 값들을 보여지고 그다음에 평균값을 구하는 함수
  baseAcX = sumAcX / max;
  baseAcY = sumAcY / max;
  baseAcZ = sumAcZ / max;

}


void initDT(){
  
  t_prev = millis();
  
}

void calcDT(){
  
  t_now = millis();
  dt = (t_now - t_prev) / 1000.0;
  t_prev = t_now;
  
}

void calculate(){

  if(baseAcX - before_AcX < 500 && baseAcX - before_AcX > -500 && before_AcY - baseAcY > -500 && before_AcY - baseAcY < 500){
    count++;

    if(count > 10){
      spX = 0;
      spY = 0;
    }
    
  }
  else{
    count=0;
  }

  //x축은 양수값이 크다!,y축은 음수값이 크다!
  if(baseAcX>0) baseAcX *=1.0;
  if(baseAcY<0) baseAcY *=0.80;
  if(baseAcY>0) baseAcY *=1.10;

  //가속도 -> 속도 적분

  acXGap = baseAcX - before_AcX;
  acYGap = baseAcY - before_AcY;
  before_SpX = spX;
  before_SpY = spY;

  spX += dt * (baseAcX);
  spY += dt * (baseAcY);

  spXGap = spX - before_SpX;
  spYGap = spY - before_SpY;

  //속도  -> 위치 적분
  X += (dt * spX) / 100.0 ;
  Y += (dt * spY) / 100.0 ;

}

void arr_print(){
  
  /*
  Serial.print(dt,DEC);
  Serial.print(" ");
  Serial.print(baseAcX);
  Serial.print(" ");
  Serial.print(baseAcY);
  Serial.println(" ");
  Serial.print(baseAcZ);
  Serial.println(" ");
  */

  /*
  Serial.print(" ");
  Serial.print(spX);
  Serial.print(" ");
  Serial.println(spY); 
   */
   
  Serial.print(X);
  Serial.print(" ");
  Serial.println(Y);

}

