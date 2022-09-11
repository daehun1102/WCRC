#include "AIRO_XARM.h"
#include <TimerFive.h>
#include <Pixy2.h>
#include <string.h>

Pixy2   pixy;

//Robot parameter
#define L       45      //45mm
#define D       85      //85mm
#define LPLUSD  0.13    //L+D=130mm/1000 = 0.13m
#define R       30      //30mm

#define NUM_OBJECT  4
#define ITER_NUM 10

//for Encoder pin define
#define ENC1_CHA    18  //INT.5
#define ENC1_CHB    31
#define ENC2_CHA    19  //INT.4
#define ENC2_CHB    38
#define ENC3_CHA    3   //INT.1
#define ENC3_CHB    49
#define ENC4_CHA    2   //INT.0
#define ENC4_CHB    A1

//for Motor I/O pin define
#define M1_I1       34
#define M1_I2       35
#define M1_PWM      12
#define M2_I1       37
#define M2_I2       36
#define M2_PWM      8
#define M3_I1       42
#define M3_I2       43
#define M3_PWM      9
#define M4_I1       A5
#define M4_I2       A4
#define M4_PWM      5
#define LED_R       A13
#define LED_G       A14
#define LED_B       A15
//
#define PSD_R       A7
#define PSD_L       A6

//for PID control
//motor1 gain
#define Kp1         0.65
#define Ki1         0.2
#define Kd1         0.15
//motor2 gain
#define Kp2         0.6
#define Ki2         0.2
#define Kd2         0.15
//motor3 gain
#define Kp3         0.6
#define Ki3         0.2
#define Kd3         0.16
//motor4 gain
#define Kp4         0.6
#define Ki4         0.2
#define Kd4         0.15

bool ul_rcv_flag= false;
byte ul_rdata ='0';

//Timer control
bool t5_flag=0;

//pixy Variables
int object_position[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int x_value[NUM_OBJECT];
int block_check[8];
int pixy_err_count = 0;
int pixy_err_count_1 = 0;
int block_p = 0;
int dir_flag = 0;
float lr_time = 0, lr_value = 0;
int iter_count = 0;

int signiture_xvalue = 0; // 미세 조정을 위한 블록의 x축 값
int detect_blocks = 0;
int xvalue_avg_value = 0;
int xvalue_avg[NUM_OBJECT] = {};
int yvalue_avg[NUM_OBJECT] = {};

int mission[NUM_OBJECT] = {1,3,4,6}; //미션 인식 존에서 인식한 물체 저장
int frameposition[NUM_OBJECT] = {}; //

int object_signiture[ITER_NUM][NUM_OBJECT] = {}; //적재함에서 인식한 물체 저장
int object_raw_xvalue[ITER_NUM][NUM_OBJECT] = {};
int object_raw_yvalue[ITER_NUM][NUM_OBJECT] = {};
int object_sorted_xvalue[ITER_NUM][NUM_OBJECT] = {};
int object_sorted_yvalue[ITER_NUM][NUM_OBJECT] = {};

float pixy_x = 0;


//Encoder value
long    e1cnt = 0;
long    e1cnt_k = 0, e1cnt_k_1 = 0, d_e1cnt = 0;
long    e2cnt = 0;
long    e2cnt_k = 0, e2cnt_k_1 = 0, d_e2cnt = 0;
long    e3cnt = 0;
long    e3cnt_k = 0, e3cnt_k_1 = 0, d_e3cnt = 0;
long    e4cnt = 0;
long    e4cnt_k = 0, e4cnt_k_1 = 0, d_e4cnt = 0;

//motor value
float   m1speed = 0;
float   m2speed = 0;
float   m3speed = 0;
float   m4speed = 0;
float x_val = 0, y_val = 0, w_val = 0;

//for motor control variable
//motor1
float   m1_ref_spd = 0;
float   m1_err_spd = 0;
float   m1_err_spd_k_1 = 0;
float   m1_derr_spd = 0;
float   m1_err_sum = 0;
float   m1_ctrl_up = 0;
float   m1_ctrl_ui = 0;
float   m1_ctrl_ud = 0;
int     m1_ctrl_u = 0;
int     m1_ipwm_u = 0;

//motor2
float   m2_ref_spd = 0;
float   m2_err_spd = 0;
float   m2_err_spd_k_1 = 0;
float   m2_derr_spd = 0;
float   m2_err_sum = 0;
float   m2_ctrl_up = 0;
float   m2_ctrl_ui = 0;
float   m2_ctrl_ud = 0;
int     m2_ctrl_u = 0;
int     m2_ipwm_u = 0;
//motor3
float   m3_ref_spd = 0;
float   m3_err_spd = 0;
float   m3_err_spd_k_1 = 0;
float   m3_derr_spd = 0;
float   m3_err_sum = 0;
float   m3_ctrl_up = 0;
float   m3_ctrl_ui = 0;
float   m3_ctrl_ud = 0;
int     m3_ctrl_u = 0;
int     m3_ipwm_u = 0;
//motor4
float   m4_ref_spd = 0;
float   m4_err_spd = 0;
float   m4_err_spd_k_1 = 0;
float   m4_derr_spd = 0;
float   m4_err_sum = 0;
float   m4_ctrl_up = 0;
float   m4_ctrl_ui = 0;
float   m4_ctrl_ud = 0;
int     m4_ctrl_u = 0;
int     m4_ipwm_u = 0;

unsigned int dis_l, dis_r = 0;
unsigned int dis_avg_l = 0, dis_avg_r = 0, dis_avg = 0 ,dis_avg_ll=0 ,dis_avg_rr=0;
unsigned int m_seq = 0, wait_flag = 0, set_cycle = 0;
unsigned long time = millis() ;

void setup() {
  pinMode(ENC1_CHA, INPUT_PULLUP);
  pinMode(ENC1_CHB, INPUT_PULLUP);
  pinMode(ENC2_CHA, INPUT_PULLUP);
  pinMode(ENC2_CHB, INPUT_PULLUP);
  pinMode(ENC3_CHA, INPUT_PULLUP);
  pinMode(ENC3_CHB, INPUT_PULLUP);
  pinMode(ENC4_CHA, INPUT_PULLUP);
  pinMode(ENC4_CHB, INPUT_PULLUP);
  pinMode(M1_I1, OUTPUT);
  pinMode(M1_I2, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_I1, OUTPUT);
  pinMode(M2_I2, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M3_I1, OUTPUT);
  pinMode(M3_I2, OUTPUT);
  pinMode(M3_PWM, OUTPUT);
  pinMode(M4_I1, OUTPUT);
  pinMode(M4_I2, OUTPUT);
  pinMode(M4_PWM, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1_CHA), Enc1chA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_CHA), Enc2chA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_CHA), Enc3chA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4_CHA), Enc4chA_ISR, CHANGE);
  //Motor initialize
  digitalWrite(M1_I1, HIGH);  //M1: I1(H), I2(L) -> Forward
  digitalWrite(M1_I2, LOW);
  analogWrite(M1_PWM, 0);
  digitalWrite(M2_I1, HIGH);  //M2: I1(H), I2(L) -> Forward
  digitalWrite(M2_I2, LOW);
  analogWrite(M2_PWM, 0);
  digitalWrite(M3_I1, HIGH);  //M3: I1(H), I2(L) -> Forward
  digitalWrite(M3_I2, LOW);
  analogWrite(M3_PWM, 0);
  digitalWrite(M4_I1, HIGH);  //M4: I1(H), I2(L) -> Forward
  digitalWrite(M4_I2, LOW);
  analogWrite(M4_PWM, 0);
  delay(500);

  //pixy 2 -> init
  pixy.init();
  delay(100);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
  Serial.begin(115200);
  Serial2.begin(115200);
  delay(500);

        dae_Servomove(100,840,100,700,465,1000);
        delay(1000);
  //led setup before mission start
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_R, HIGH);
    delay(500);
    digitalWrite(LED_R, LOW);
    delay(500);
  }
  digitalWrite(LED_B, HIGH);
  delay(500);
  digitalWrite(LED_B, LOW);
  delay(500);

  //Timer5 setup
  Timer5.initialize(10000);       //10msec,
  Timer5.attachInterrupt(T5ISR);  //T5ISR
}

void loop() {
        //스위치문 사용해서 각 단계별로 코딩
        //각 단계를 진행하면서 m_seq가 바뀐다.
        
    switch (m_seq) {
    case 0: // step 0->장애물까지 전진
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 1; //case 1로 이동a7
          set_cycle = 0;
          wait_flag = 0;
          dis_avg = 0;
        }
      }
      else {
        if (set_cycle > 375) {  //about 10cm (가까워질수록 값이 커진다)
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
            dis_avg_l=0;
            dis_avg_r=0;
          }
        }
        else {
          pid();
          x_val = 10;
          y_val = 0;
          w_val = 0;
        }
      }
      break;
      
          

          case 1: // 미션인식존까지 이동
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 11; //case 1로 이동a7
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else {
        if (set_cycle > 280) {  //about 10cm
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = 10;
          w_val = 0.29;
        }
      }
      break;

      case 11: // 미션인식존까지 이동
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 50) { //wait for 1sec(100*10msec)
          m_seq = 2; //case 1로 이동a7
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else {
        if (set_cycle > 40) {  //about 10cm
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = -10;
          y_val = 0;
          w_val = 0.35;
        }
      }
      break;
      
    case 2: //미션 인식 존...pixy2이용해서 미션 인식하기
      if (wait_flag) {
        if (set_cycle > 50) { //wait for
          m_seq = 3;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else {
        if (set_cycle > 101) {
          //To use Pixy2, turn off timer 5
          Timer5.stop();
          delay(200);
          dae_Servomove(792,975,382,502,830,1000);
          delay(1000);
          //parameter_clr();


          //------------------------------------------------------- 픽시 캠 사용 ---------------------------------------------------------
          pixy.ccc.getBlocks(); //인식하는 것을 평균...내거나 해서 필터링해야하는데...

          detect_blocks = pixy.ccc.numBlocks;
          delay(50);

          while (NUM_OBJECT != detect_blocks) //원하는 개수만큼 인식이 되지 않는다면 인식될 때까지 대기
          {
            if (pixy_err_count == 0)
            
            {
              ServoMove(Serial2, 5 , 850 , 500);
              delay(500);
              pixy_err_count = 1;
              delay(500);

            }
            else if (pixy_err_count == 1)
            {
               ServoMove(Serial2, 5 , 830 , 500);
              delay(500);
              pixy_err_count = 2;
              delay(500);
            }
            else
            {
              ServoMove(Serial2, 5 , 840 , 500);
              delay(500);
              pixy_err_count = 0;
              delay(500);
            }
            //repeat printing red led

            digitalWrite(LED_R, HIGH);
            delay(200);
            digitalWrite(LED_R, LOW);
            delay(500);
            pixy.ccc.getBlocks();
            detect_blocks = pixy.ccc.numBlocks;
          }
          
          pixy_err_count = 0;

          for (int i = 0; i < detect_blocks; i++) //지정된 개수만큼 저장
          {
            mission[i] = pixy.ccc.blocks[i].m_signature;
            x_value[i] = pixy.ccc.blocks[i].m_x;
          }
          SortBlocks();//x좌표 정보에 따른 버블정렬을 수행한다
          detect_blocks = 0;

          //정렬이 완료되면 여기 시퀀스는 끝
          //시퀀스 ++, 타이머인터럽트 restart();
          /**/
          
          //Change Arm pos for detect 2
          //angle_mission_detect1[3] = 1086;
            dae_Servomove(790,840,100,700,465,1000);
            delay(1000);


          //printing led for checking if values are saved correctly
          //printled(pixy.ccc.numBlocks); // 인식한 개수만큼 led 깜박이는 명령어. 내부에서 mission 배열 불러옴.

          //timer5 restart
          Timer5.restart();

          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }

        }
        else {
          if (set_cycle == 0) {
            parameter_clr();
            set_cycle = 1;
          }
        }
      }
      break;


      case 3: //직선 이동...(블록 집는 구역으로)
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 110) { //wait for 1sec(100*10msec)
          m_seq = 31;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else {
        if (set_cycle > 635) {  //원래 run for 2.5sec
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 10;
          y_val = 0;
          w_val = 0.2;
        }
      }
      break;

        case 31: // 왼쪽으로가서 벽찍기
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 32;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else {
        if (set_cycle > 200) {  //원래 run for 2.5sec
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = 10;
          w_val = 0.3;
        }
      }
      break;

            case 32: //적재함 앞으로..
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 4;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else {
        if (set_cycle > 340) {  //원래 run for 2.5sec
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = -10;
          w_val = 0.3;
        }
      }
      break;

      case 4: // 로봇 팔 들기...(픽시 캠 인식을 위한 각도로)
      if (wait_flag) {
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 5;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        dae_Servomove(792,1030,382,602,470,1000);
        delay(1000);
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      case 5: // 미션 그립 구역에서 픽시캠 인식 단계
      if (wait_flag) {
        if (set_cycle > 100) { //wait for
          m_seq = 499;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else {
        if (set_cycle > 101) {  //wait for 1.0s before turn off timer5
          ////To use Pixy2, turn off timer 5
          Timer5.stop();
          delay(200);
          //parameter_clr();

          //------------------------------------------------------------ 픽시 캠 사용 ----------------------------------------
          detect_blocks = 0;

          pixy.ccc.getBlocks();
          detect_blocks = pixy.ccc.numBlocks;
          delay(50);

          while (NUM_OBJECT != detect_blocks) //원하는 개수만큼 인식이 되지 않는다면 인식될 때까지 대기
          {
           
//              if(pixy_err_count == 0)
//              {
//              ServoMove(Serial2 , 5 , 440 , 500);
//              pixy_err_count = 1;
//              delay(500);
//
//              }
//              else if(pixy_err_count == 1)
//              {
//              ServoMove(Serial2 , 5 , 455 , 500);
//              pixy_err_count = 2;
//              delay(500);
//              }
//              
//              else if(pixy_err_count == 2)
//              {
//              ServoMove(Serial2 , 5 , 470 , 500);
//              pixy_err_count = 3;
//              delay(500);
//              }
//
//              else if(pixy_err_count == 3)
//              {
//              ServoMove(Serial2 , 5 , 485 , 500);
//              pixy_err_count = 4;
//              delay(500);
//              }
//              
//              else
//              {
//              ServoMove(Serial2 , 5 , 500 , 500);
//              pixy_err_count = 0;
//              delay(500);
//              }

            pixy.ccc.getBlocks();
            detect_blocks = pixy.ccc.numBlocks;
            
            //repeat printing red led
            digitalWrite(LED_R, HIGH);
            delay(200);
            digitalWrite(LED_R, LOW);
            delay(500);

          Serial.println(detect_blocks);
            
          }
          //pixy_err_count = 0;

          //NUM_OBJECT 개수만큼 각각 배열에 저장하기
          for (int iter = 0; iter < ITER_NUM; iter++)
          {
            pixy.ccc.getBlocks();
            detect_blocks = pixy.ccc.numBlocks;
            for (int onum = 0; onum < NUM_OBJECT; onum++)
            {
              //미션 수행 블럭들의 빨주노초 숫자와 x, y 값을 각각 저장
              //행은 반복 횟수를 의미한다. 각 행의 열 순서는 랜덤으로 저장되는 값들이다.
              object_signiture[iter][onum] = pixy.ccc.blocks[onum].m_signature;
              object_raw_xvalue[iter][onum] = pixy.ccc.blocks[onum].m_x;
              object_raw_yvalue[iter][onum] = pixy.ccc.blocks[onum].m_y;
            }
            delay(20);
          }

          //이제 이렇게 각 block들 *10번 저장한 값들을,,, 미션 인식 존에서 인식한 순서로 정렬
          //mission[]에 저장되어있다.
          for (int iter = 0; iter < ITER_NUM; iter++)
          {
            for (int i = 0; i < NUM_OBJECT; i++)
            {
              for (int j = 0; j < NUM_OBJECT; j++)
              {
                if (mission[i] == object_signiture[iter][j])
                {
                  object_sorted_xvalue[iter][i] = object_raw_xvalue[iter][j];
                  object_sorted_yvalue[iter][i] = object_raw_yvalue[iter][j];
                }
              }
            }
          }

          //이제 mission 인식한 순서대로 x, y value가 저장되어있으므로, 평균내기
          //평균을 내기 전에 극단적인 값들을 빼주려면 정렬이 필요함.
          //행에 대해서 정렬하는 알고리즘이 필요,,,
          SortBlocks_row(object_sorted_xvalue, ITER_NUM, NUM_OBJECT);
          SortBlocks_row(object_sorted_yvalue, ITER_NUM, NUM_OBJECT);

          //각각의 object의 x값과 y값에 대해 평균내기(극값 각각 2개씩 제외)
          for (int col = 0; col < NUM_OBJECT; col ++)
          {
            for (int row = 2; row < ITER_NUM - 2; row++)
            {
              xvalue_avg[col] = xvalue_avg[col] + object_sorted_xvalue[row][col];
              yvalue_avg[col] = yvalue_avg[col] + object_sorted_yvalue[row][col];
            }
            xvalue_avg[col] = xvalue_avg[col] / 6;
            yvalue_avg[col] = yvalue_avg[col] / 6;
          }

          //반복하고 평균내서 구한 x y값에 따라 구역을 나눠보기
          positioning(xvalue_avg, yvalue_avg); // object_position라는 배열에 저장됨
          //object_position에 저장되는 순서는 이미 mission에 의해 정렬해 놓은 x y 값들


          //----------------------------------------------------------여기까지는 추가한 것
          Timer5.restart();

          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }

        }
        else {
          if (set_cycle == 0) {
            parameter_clr();
            set_cycle = 1;
          }
        }
      }
      break;
      
      
      case 499:
      arm_clear(500);
      break;

      case 500://  1번째 물체 잡으러갈때 y축으로 얼마나 이동할지 결정
      container_ymove(object_position[0], 52);
      break;

      
      
      case 51: // 로봇 팔 회전...(픽시캠으로 잡으려는 물체인식)
      pixy_1x2x(object_position[0], 501);
      break;
      
      case 501:
      pixy_get_X(mission[0] , 502);
      break;
      
      case 502: 
      pixy_ymove(52);
      break;

      case 52: // 로봇 팔 회전...(픽시 캠 인식 후, 1번째  블럭을 잡기 좋은 위치로)
      xram_decide_floor(object_position[0], 6);
      break;
      


    case 6: // x축 직진...(블럭 잡는 지점 바로앞으로)
    dis_avgg(7);
    break;


    case 7: // 첫 번째물건 그립(음,,,)
      if (wait_flag) {
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 8;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        ServoMove(Serial2 , 1 , 790 , 1000);
        delay(1000);
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

    case 8: //후진하기 (블록 goal 지점방향으로 회전하기 위해)
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 82;
          set_cycle = 0;
          wait_flag = 0;

        }
      }
      else {
        if (set_cycle > 140) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = -5;
          y_val = 0;
          w_val = 0;
        }
      }
      break;

      case 82:
      arm_clear(83);
      break;

      case 83:       //1번째 물체 옮기러 goal지점 앞으로
      container_ybackmove(object_position[0], 101);
      break;

      case 101: // 로봇 팔 회전... (goal지점에 물체넣기)
      goal_in_arm(mission[0] , 102);
      break;

    case 102: // 그리퍼 놓기
      if (wait_flag) {
        if (set_cycle > 90) { //wait for 1sec(100*10msec)
          m_seq = 109;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        ServoMove(Serial2, 1 , 300,1000);
        delay(1000);
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      //------------------------------------------------- 1 번째 물체 이동 완료 --------------------------------------------------------------------------------


    //------------------------------------------------- 2 번째 물체 목표 -----------------------------------------------------------------------------
    case 109: // 로봇 팔 회전... 1층잡으러갈때 걸리는 거 방지  
      arm_clear(103);  
      break;


      case 103:// 후진, 2번째 물체를 향해
       container_getback(object_position[1], 1202);
       break; 

      case 110: // 로봇 팔 회전...(픽시캠으로 잡으려는 물체인식)
      pixy_1x2x(object_position[1], 12);
      break;
      
      case 12:
      dis_avgg2(1200);
      break;

      case 1200:
      pixy_get_X(mission[1] , 1201);
      break;
      
      case 1201: 
      pixy_ymove(1202);
      break;

      case 1202: // 로봇 팔 회전...(1층 or 2층)
      xram_decide_floor(object_position[1], 1203);
      break;

    case 1203: // x축 직진...(블럭 잡는 지점 바로앞으로)
      dis_avgg(121);
      break;
      

      case 121:    // 그리퍼로 잡기
      if (wait_flag) {
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 122;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        ServoMove(Serial2, 1 , 790 , 1000);
        delay(1000);

        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      case 122:  //후진하기
      if (wait_flag) { //시퀀스가 완료되었다면.... 
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 139;
          set_cycle = 0;
          wait_flag = 0;

        }
      }
      else {
        if (set_cycle > 140) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = -5;
          y_val = 0;
          w_val = 0;
        }
      }
      break;

      case 139:
      arm_clear(140);
      break;

      case 140:       //2번째 물체 옮기러 goal지점 앞으로
      container_ybackmove(object_position[1], 14);
      break;

      case 14: // 로봇 팔 회전... (goal지점에 물체넣기)
      goal_in_arm(mission[1] , 141);
      break;
      
      case 141:    // 그리퍼로 놓기
      if (wait_flag) {
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 159;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        ServoMove(Serial2, 1 , 300 , 1000);
        delay(1000);

        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;
    //------------------------------------------------- 2 번째 물체 이동 완료 --------------------------------------------------------------------------

    //------------------------------------------------- 3 번째 물체 목표 -----------------------------------------------------------------------------
      case 159: // 로봇 팔 회전... 1층잡으러갈때 걸리는 거 방지
      arm_clear(150);
      break;


      case 150:// 적재함으로 이동... 3번째 물체를 향해
      container_getback(object_position[2], 1602);
      break; 

      case 151: // 로봇 팔 회전...(픽시캠으로 잡으려는 물체인식)
      pixy_1x2x(object_position[2], 160);
      break;
        

      case 160:
      dis_avgg2(1600);
      break;

      case 1600:
      pixy_get_X(mission[2] , 1601);
      break;
      
      case 1601: 
      pixy_ymove(1602);
      break;

      case 1602:
      xram_decide_floor(object_position[2],1603);
      break;
      
      case 1603: //x축 직진...(3번째 물체 그립을 위해)
      dis_avgg(161);
      break;


      case 161:    // 그리퍼로 잡기
      if (wait_flag) {
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 162;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        ServoMove(Serial2, 1 , 790 , 1000);
        delay(1000);

        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      case 162:  //후진하기
      if (wait_flag) { //시퀀스가 완료되었다면.... 
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 179;
          set_cycle = 0;
          wait_flag = 0;

        }
      }
      else {
        if (set_cycle > 140) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = -5;
          y_val = 0;
          w_val = 0;
        }
      }
      break;

      
      case 179:
      arm_clear(180);
      break;

      case 180:       //3번째 물체 옮기러 goal지점 앞으로
      container_ybackmove(object_position[2], 18);
      break;

      case 18: // 로봇 팔 회전... (goal지점에 물체넣기)
      goal_in_arm(mission[2] , 181);
      break;
      
      case 181:    // 그리퍼로 놓기
      if (wait_flag) {
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 199;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        ServoMove(Serial2, 1 , 390 , 1000);
        delay(1000);

        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;
    //------------------------------------------------- 3 번째 물체 이동 완료 --------------------------------------------------------------------------

    //------------------------------------------------- 4 번째 물체 목표 -----------------------------------------------------------------------------
      case 199: // 로봇 팔 회전... 1층잡으러갈때 걸리는 거 방지
      arm_clear(19);
      break;

      case 19:// 적재함 앞으로... 4번째 물체를 향해
      container_getback(object_position[3], 2002);
      break;

      case 1999: // 로봇 팔 회전...(픽시캠으로 잡으려는 물체인식)
      pixy_1x2x(object_position[3], 200);
      break;

      case 200:
      dis_avgg2(2000);
      break;

      case 2000:
      pixy_get_X(mission[3] , 2001);
      break;
      
      case 2001: 
      pixy_ymove(2002);
      break;

      case 2002: // 로봇팔움직이기 (1층 or 2층)
      xram_decide_floor(object_position[3],2003);
      break;

      case 2003: //x축 직진...(4번째 물체 그립을 위해)
        dis_avgg(201);
      break;

      case 201:    // 그리퍼로 잡기
      if (wait_flag) {
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 202;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        ServoMove(Serial2, 1 , 790 , 1000);
        delay(1000);

        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      case 202:  //후진하기
      if (wait_flag) { //시퀀스가 완료되었다면.... 
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 219;
          set_cycle = 0;
          wait_flag = 0;

        }
      }
      else {
        if (set_cycle > 140) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = -5;
          y_val = 0;
          w_val = 0;
        }
      }
      break;

      
      case 219:
      arm_clear(220);
      break;

      case 220:       //4번째 물체 옮기러 goal지점 앞으로
      container_ybackmove(object_position[3], 22);
      break;

      case 22: // 로봇 팔 회전... (goal지점에 물체넣기)
      goal_in_arm(mission[3] , 221);
      break;
      
      case 221:    // 그리퍼로 놓기
      if (wait_flag) {
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 776;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        ServoMove(Serial2, 1 , 300 , 1000);
        delay(1000);

        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      case 776:
      arm_clear(777);
      break;

      

     
       //------------------------------------------------- 4 번째 물체 이동 완료 --------------------------------------------------------------------------
  

            case 777:    // 집 갈 준비
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 778;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else {
        if (set_cycle > 100) {  //run for 1.2 sec
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = -10;
          w_val = 0;
        }
      }
      break;


            case 778:    // 집으로ㄱㄱ
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 50) { //wait for 1sec(100*10msec)
          m_seq = 779;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else {
        if (set_cycle > 460) {  //run for 1.2 sec
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = -20;
          y_val = 0;
          w_val = 0;
        }
      }
      break;
      
}

}


void psd(){
  
   dis_l = analogRead(PSD_L);
   dis_r = analogRead(PSD_R);
   dis_l = dis_l + analogRead(PSD_L);
   dis_r = dis_r + analogRead(PSD_R);
    dis_l = dis_l + analogRead(PSD_L);
    dis_r = dis_r + analogRead(PSD_R);
    
    dis_avg_l = dis_l / 3;
    dis_avg_r = dis_r / 3;
    dis_avg = (dis_avg_l + dis_avg_r) / 2;

    dis_l = 0;
    dis_r = 0;
}


void stop_(){
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
  analogWrite(M3_PWM, 0);
  analogWrite(M4_PWM, 0);
}

void T5ISR(){              //타이머가 10ms 마다 실행되면서 이함수도 같이실행됨
                           //set_cycle로 시간을 조절할수있음
    set_cycle++;
    t5_flag = true;
    e1cnt_k = e1cnt;
    e2cnt_k = e2cnt;
    e3cnt_k = e3cnt;
    e4cnt_k = e4cnt;
        
    d_e1cnt = e1cnt_k - e1cnt_k_1;
    d_e2cnt = e2cnt_k - e2cnt_k_1;
    d_e3cnt = e3cnt_k - e3cnt_k_1;
    d_e4cnt = e4cnt_k - e4cnt_k_1;

                    // Store current value in past value
    e1cnt_k_1 = e1cnt_k;
    e2cnt_k_1 = e2cnt_k;
    e3cnt_k_1 = e3cnt_k;
    e4cnt_k_1 = e4cnt_k;
}

void pid(){
  if(t5_flag){
        t5_flag = 0;

    m1speed = d_e1cnt * 17/10;      // * 1200/706.806
    m2speed = d_e2cnt * 17/10;
    m3speed = d_e3cnt * 17/10;
    m4speed = d_e4cnt * 17/10;
  

   //================================================//
     //Robot Control
     MecanumVelocity(x_val, y_val, w_val); //vx[cm/s], vy[cm/s], a_vel[deg/s]
     Motor1Control(m1_ref_spd);
     Motor2Control(m2_ref_spd);
     Motor3Control(m3_ref_spd);
     Motor4Control(m4_ref_spd);
     parameter_clr(); 
  
  }
}

//for Encoder 1 interrupt
void Enc1chA_ISR() {
  if (digitalRead(ENC1_CHA) == HIGH) {
    if (digitalRead(ENC1_CHB) == LOW) e1cnt--;
    else                             e1cnt++;
  }
  else {
    if (digitalRead(ENC1_CHB) == HIGH) e1cnt--;
    else                              e1cnt++;
  }
}

//for Encdoer 2 interrupt
void Enc2chA_ISR() {
  if (digitalRead(ENC2_CHA) == HIGH) {
    if (digitalRead(ENC2_CHB) == LOW) e2cnt--;
    else                             e2cnt++;
  }
  else {
    if (digitalRead(ENC2_CHB) == HIGH) e2cnt--;
    else                              e2cnt++;
  }
}

//for Encdoer 3 interrupt
void Enc3chA_ISR() {
  if (digitalRead(ENC3_CHA) == HIGH) {
    if (digitalRead(ENC3_CHB) == LOW) e3cnt++;
    else                             e3cnt--;
  }
  else {
    if (digitalRead(ENC3_CHB) == HIGH) e3cnt++;
    else                              e3cnt--;
  }
}

//for Encdoer 4 interrupt
void Enc4chA_ISR() {
  if (digitalRead(ENC4_CHA) == HIGH) {
    if (digitalRead(ENC4_CHB) == LOW) e4cnt++;
    else                             e4cnt--;
  }
  else {
    if (digitalRead(ENC4_CHB) == HIGH) e4cnt++;
    else                              e4cnt--;
  }
}

//Motor Control
//motor1
void Motor1Control(float m1_ref_spd) {
  //Error
  m1_err_spd = m1_ref_spd - m1speed;
  m1_derr_spd = m1_err_spd - m1_err_spd_k_1;
  m1_err_sum = m1_err_sum + m1_err_spd;

  m1_err_spd_k_1 = m1_err_spd;

  //PID-Controller
  m1_ctrl_up = Kp1 * m1_err_spd;
  m1_ctrl_ui = Ki1 * m1_err_sum;
  m1_ctrl_ud = Kd1 * m1_derr_spd;

  m1_ctrl_u = (int)(m1_ctrl_up + m1_ctrl_ud + m1_ctrl_ui);
  if (m1_ref_spd == 0) m1_ctrl_u = 0;
  if (m1_ctrl_u >= 0) {
    digitalWrite(M1_I1, HIGH);  //M1: I1(H), I2(L) -> Forward
    digitalWrite(M1_I2, LOW);
    if (m1_ctrl_u > 255) m1_ipwm_u = 255;
    else m1_ipwm_u = (int)m1_ctrl_u;
  }
  else {
    digitalWrite(M1_I1, LOW);  //M1: I1(L), I2(H) -> Backward
    digitalWrite(M1_I2, HIGH);
    if (m1_ctrl_u < -255) m1_ipwm_u = 255;
    else m1_ipwm_u = (int)m1_ctrl_u * (-1);
  }
  analogWrite(M1_PWM, m1_ipwm_u);    //motor input
}
//motor2
void Motor2Control(float m2_ref_spd) {
  //Error
  m2_err_spd = m2_ref_spd - m2speed;
  m2_derr_spd = m2_err_spd - m2_err_spd_k_1;
  m2_err_sum = m2_err_sum + m2_err_spd;

  m2_err_spd_k_1 = m2_err_spd;

  //PID-Controller
  m2_ctrl_up = Kp2 * m2_err_spd;
  m2_ctrl_ui = Ki2 * m2_err_sum;
  m2_ctrl_ud = Kd2 * m2_derr_spd;

  m2_ctrl_u = (int)(m2_ctrl_up + m2_ctrl_ud + m2_ctrl_ui);
  if (m2_ref_spd == 0) m2_ctrl_u = 0;
  if (m2_ctrl_u >= 0) {
    digitalWrite(M2_I1, HIGH);  //M2: I1(H), I2(L) -> Forward
    digitalWrite(M2_I2, LOW);
    if (m2_ctrl_u > 255) m2_ipwm_u = 255;
    else m2_ipwm_u = (int)m2_ctrl_u;
  }
  else {
    digitalWrite(M2_I1, LOW);  //M2: I1(L), I2(H) -> Backward
    digitalWrite(M2_I2, HIGH);
    if (m2_ctrl_u < -255) m2_ipwm_u = 255;
    else m2_ipwm_u = (int)m2_ctrl_u * (-1);
  }
  analogWrite(M2_PWM, m2_ipwm_u);    //motor input
}
//motor3
void Motor3Control(float m3_ref_spd) {
  //Error
  m3_err_spd = m3_ref_spd - m3speed;
  m3_derr_spd = m3_err_spd - m3_err_spd_k_1;
  m3_err_sum = m3_err_sum + m3_err_spd;

  m3_err_spd_k_1 = m3_err_spd;

  //PID-Controller
  m3_ctrl_up = Kp3 * m3_err_spd;
  m3_ctrl_ui = Ki3 * m3_err_sum;
  m3_ctrl_ud = Kd3 * m3_derr_spd;

  m3_ctrl_u = (int)(m3_ctrl_up + m3_ctrl_ud + m3_ctrl_ui);
  if (m3_ref_spd == 0) m3_ctrl_u = 0;
  if (m3_ctrl_u >= 0) {
    digitalWrite(M3_I1, HIGH);  //M3: I1(H), I2(L) -> Forward
    digitalWrite(M3_I2, LOW);
    if (m3_ctrl_u > 255) m3_ipwm_u = 255;
    else m3_ipwm_u = (int)m3_ctrl_u;
  }
  else {
    digitalWrite(M3_I1, LOW);  //M3: I1(L), I2(H) -> Backward
    digitalWrite(M3_I2, HIGH);
    if (m3_ctrl_u < -255) m3_ipwm_u = 255;
    else m3_ipwm_u = (int)m3_ctrl_u * (-1);
  }
  analogWrite(M3_PWM, m3_ipwm_u);    //motor input
}
//motor4
void Motor4Control(float m4_ref_spd) {
  //Error
  m4_err_spd = m4_ref_spd - m4speed;
  m4_derr_spd = m4_err_spd - m4_err_spd_k_1;
  m4_err_sum = m4_err_sum + m4_err_spd;

  m4_err_spd_k_1 = m4_err_spd;

  //PID-Controller
  m4_ctrl_up = Kp4 * m4_err_spd;
  m4_ctrl_ui = Ki4 * m4_err_sum;
  m4_ctrl_ud = Kd4 * m4_derr_spd;

  m4_ctrl_u = (int)(m4_ctrl_up + m4_ctrl_ud + m4_ctrl_ui);
  if (m4_ref_spd == 0) m4_ctrl_u = 0;
  if (m4_ctrl_u >= 0) {
    digitalWrite(M4_I1, HIGH);  //M4: I1(H), I2(L) -> Forward
    digitalWrite(M4_I2, LOW);
    if (m4_ctrl_u > 255) m4_ipwm_u = 255;
    else m4_ipwm_u = (int)m4_ctrl_u;
  }
  else {
    digitalWrite(M4_I1, LOW);  //M4: I1(L), I2(H) -> Backward
    digitalWrite(M4_I2, HIGH);
    if (m4_ctrl_u < -255) m4_ipwm_u = 255;
    else m4_ipwm_u = (int)m4_ctrl_u * (-1);
  }
  analogWrite(M4_PWM, m4_ipwm_u);    //motor input
}

// motor parameter clear
void parameter_clr() {
  //Variables
  //Encoder value
  e1cnt = 0;
  e1cnt_k = 0;
  e1cnt_k_1 = 0;
  d_e1cnt = 0;

  e2cnt = 0;
  e2cnt_k = 0;
  e2cnt_k_1 = 0;
  d_e2cnt = 0;

  e3cnt = 0;
  e3cnt_k = 0;
  e3cnt_k_1 = 0;
  d_e3cnt = 0;

  e4cnt = 0;
  e4cnt_k = 0;
  e4cnt_k_1 = 0;
  d_e4cnt = 0;

  //motor value
  m1speed = 0;
  m2speed = 0;
  m3speed = 0;
  m4speed = 0;

  //for motor control variable
  //motor1
  m1_ref_spd = 0;
  m1_err_spd = 0;
  m1_err_spd_k_1 = 0;
  m1_derr_spd = 0;
  m1_err_sum = 0;
  m1_ctrl_up = 0;
  m1_ctrl_ui = 0;
  m1_ctrl_ud = 0;
  m1_ctrl_u = 0;
  m1_ipwm_u = 0;
  //motor2
  m2_ref_spd = 0;
  m2_err_spd = 0;
  m2_err_spd_k_1 = 0;
  m2_derr_spd = 0;
  m2_err_sum = 0;
  m2_ctrl_up = 0;
  m2_ctrl_ui = 0;
  m2_ctrl_ud = 0;
  m2_ctrl_u = 0;
  m2_ipwm_u = 0;
  //motor3
  m3_ref_spd = 0;
  m3_err_spd = 0;
  m3_err_spd_k_1 = 0;
  m3_derr_spd = 0;
  m3_err_sum = 0;
  m3_ctrl_up = 0;
  m3_ctrl_ui = 0;
  m3_ctrl_ud = 0;
  m3_ctrl_u = 0;
  m3_ipwm_u = 0;
  //motor4
  m4_ref_spd = 0;
  m4_err_spd = 0;
  m4_err_spd_k_1 = 0;
  m4_derr_spd = 0;
  m4_err_sum = 0;
  m4_ctrl_up = 0;
  m4_ctrl_ui = 0;
  m4_ctrl_ud = 0;
  m4_ctrl_u = 0;
  m4_ipwm_u = 0;
}

void MecanumVelocity(float vx, float vy, float dpi) {
  //vx, vy(robot velocity) unit -> cm/sec
  //dpi(robot angular velocity) unit -> deg/sec
  float v1, v2, v3, v4;
  float drot;

  //[deg/sec] to [rad/sec] :( *pi/180)--> cm(rad)/sec
  dpi *= 31.41593;
  dpi /= 18;

  drot = LPLUSD * dpi;
  v1 = vx - vy - drot;
  v2 = vx + vy - drot;
  v3 = vx - vy + drot;
  v4 = vx + vy + drot;

  //wheel linear velocity to wheel speed
  m1_ref_spd = v1 * 10 / 3.141593;
  m2_ref_spd = v2 * 10 / 3.141593;
  m3_ref_spd = v3 * 10 / 3.141593;
  m4_ref_spd = v4 * 10 / 3.141593;
}

///// for Pixy
void SortBlocks() { //인식한 물체들을 x좌표 순서대로 정렬하는 알고리즘.
  for (int i = NUM_OBJECT - 1 ; i > 0 ; i--)
  {
    for (int j = 0; j < i; j++)
    {
      if (x_value[j] > x_value[j + 1])
      {
        int temp = 0;
        temp = mission[j];
        mission[j] = mission[j + 1];
        mission[j + 1] = temp;
        int temp_x = 0;
        temp_x = x_value[j];
        x_value[j] = x_value[j + 1];
        x_value[j + 1] = temp_x;
      }
    }
  }
}

void SortBlocks_row(int sarray[][NUM_OBJECT], int rnum, int cnum) {
  //2차원 배열 중 '행' 값에 대해 오름차순으로 정렬
  //모든 열에 대해서 반복(변수 col)

  int rindex;
  int cindex;

  rindex = rnum;
  cindex = cnum;
  for (int col = 0; col < cindex; col++)
  {
    for (int i = rindex - 1; i > 0 ; i--) // 버블정렬 수행
    {
      for (int j = 0; j < i; j++)
      {
        if (sarray[j][col] > sarray[j + 1][col])
        {
          int temp = sarray[j][col];
          sarray[j][col] = sarray[j + 1][col];
          sarray[j + 1][col] = temp;
        }
      }
    }
  }
}

void positioning(int x[], int y[]) {    //픽시에서 불러온 각각의 적재함 위치에 해당하는 x,y좌표로 물체들이 어디에 있는지 object_position에 저장  
  for (int h = 0; h < NUM_OBJECT; h++) {
    if (y[h] < 140) { //8개의 칸 중 윗층
      if (x[h] < 76) {
        object_position[h] = 1; //1번 칸
      }
      if (76 < x[h] && x[h] < 162) {
        object_position[h] = 2; //2번 칸
      }
      if (162 < x[h] && x[h] < 220) {
        object_position[h] = 3; //3번 칸
      }
      if (220 < x[h]) {
        object_position[h] = 4; //4번 칸
      }
    }

    else { //8개의 칸 중 아래층
      if (220 < x[h]) {
        object_position[h] = 8; //8번 칸
      }
      if (162 < x[h] && x[h] < 220) {
        object_position[h] = 7; //7번 칸
      }
      if (76 < x[h] && x[h] < 162) {
        object_position[h] = 6; //6번 칸
      }
      if (x[h] < 76) {
        object_position[h] = 5; //5번 칸
      }
    }
  }
  //1번함  32 56
  //2번함  96 130
  //3번함  184 210     110
  //4번함  260 292
  //5번함  43          170
  //6번함  43
  //7번함  43
  //8번함  43
}

void dae_Servomove(int id1, int id2 , int id3 , int id4 , int id5,int postime){    //로봇팔 조절함수
  ServoMove(Serial2, 1,id1 , postime);
  ServoMove(Serial2, 2,id2 , postime);
  ServoMove(Serial2, 3,id3 , postime);
  ServoMove(Serial2, 4,id4 , postime);
  ServoMove(Serial2, 5,id5 , postime);
}

void container_ymove(int con_position, int next_step){     // 처음 ymove
  switch (con_position) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 어느 위치인지
      {
        case 1:
        case 5:                 //1 또는 5번 위치이면 왼쪽으로 가는 sequence로 가기
                                // 1번째 열에 있는 블럭을 잡으러 가는 경우
      if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = next_step;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 115) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = 10;
          w_val = 0.7;
        }
      }
          break;

        case 2:
        case 6:                 // 2번째 열에 있는 블럭을 잡으러 가는 경우, 움직이지 않음
      if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = next_step;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 40) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = 10;
          w_val = 0.7;
        }
      }
      break;

        case 3:
        case 7:
                                // 3번째 열에 있는 블럭을 잡으러 가는 경우
      if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = next_step;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 40) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = -10;
          w_val = 0.7;
        }
      }
      break;


        case 4:
        case 8:
                                   // 4번째 열에 있는 블럭을 잡으러 가는 경우
      if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = next_step;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 125) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = -10;
          w_val = 0.7;
        }
      }
      break;

      default: break;
      }
     
}

void container_ybackmove(int con_backposition, int next_step2){   //적재함에서 --> goal지점으로 이동
  float www = 0.47;
  switch (con_backposition) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 어느 위치인지
      {
        case 1:
        case 5:                 //1 또는 5번 위치이면 왼쪽으로 가는 sequence로 가기
                                // 1번째 열에서--> goal지점으로
      if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = next_step2;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 230) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = 10;
          w_val = www;
        }
      }
          break;

        case 2:
        case 6:                 // 2번째 열에서--> goal지점으로
      if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = next_step2;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 315) {  
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = 10;
          w_val = www;
        }
      }
          break;

        case 3:
        case 7:
                                // 3번째 열에서--> goal지점으로
      if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = next_step2;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 380) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = 10;
          w_val = www;
        }
      }
      break;


        case 4:
        case 8:
                                   // 4번째 열에서--> goal지점으로
      if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = next_step2;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 470) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = 10;
          w_val = www;
        }
      }
      break;

      default: break;
      }
}

void container_getback(int con_backposition, int next_step2){   //goal지점에서 --> 적재함으로 이동
  float www = 0.47;
  switch (con_backposition) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 어느 위치인지
      {
        case 1:
        case 5:                 //1 또는 5번 위치이면 왼쪽으로 가는 sequence로 가기
                                // 1번째 열에 있는 블럭을 잡으러 가는 경우
      if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = next_step2;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 185) {  //about 0.7s    원래 100
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = -10;
          w_val = www;
        }
      }
          break;

        case 2:
        case 6:                 // 2번째 열에 있는 블럭을 잡으러 가는 경우, 움직이지 않음
      if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = next_step2;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 265) {  //원래 170
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = -10;
          w_val = www;
        }
      }
          break;

        case 3:
        case 7:
                                // 3번째 열에 있는 블럭을 잡으러 가는 경우
      if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = next_step2;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 375) {  //about 0.7s 원래 240
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = -10;
          w_val = www;
        }
      }
      break;


        case 4:
        case 8:
                                   // 4번째 열에 있는 블럭을 잡으러 가는 경우
      if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = next_step2;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 430) {  //about 0.7s 원래 310
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = -10;
          w_val = www;
        }
      }
      break;

      default: break;
      }
}

void goal_in_arm(int goal_num , int nextstep3){      //목표지점에서 번호별 로봇팔 각도조정
  if (wait_flag) {
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = nextstep3;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (goal_num)
        {
          case 1: // 원소가 빨간색이면...
            dae_Servomove(790,780,346,370,980,1000);  //5번함     //대회당일에 수정
            delay(1000);
            break;

          case 2: // 원소가 주황색이면...
            break;
          case 3: // 원소가 노란색이면...
            dae_Servomove(790,730,290,380,680,1000);  //8번함
            delay(1000);
            break;

          case 4: // 원소가 초록색이면...
            dae_Servomove(790,620,366,290,810,1000);  //3번함
            delay(1000);
            break;

          case 5: // 원소가 파란색이면...
            break;

          case 6: // 원소가 보라색이면...
            dae_Servomove(790,555,346,260,950,1000);  //1번함
            delay(1000);
            break;
          default: break;
        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      
  // 1번함  dae_Servomove(790,555,346,260,950,2000);
  // 2번함  dae_Servomove(790,620,366,290,880,2000);
  // 3번함  dae_Servomove(790,620,366,290,810,2000);
  // 4번함  dae_Servomove(790,560,386,250,750,2000);
  // 5번함  dae_Servomove(790,780,346,370,980,2000);
  // 6번함  dae_Servomove(790,790,290,400,880,2000);
  // 7번함  dae_Servomove(790,800,290,400,790,2000); 
  // 8번함  dae_Servomove(790,730,290,380,710,2000);
}

void xram_decide_floor(int con_position, int next_step3){     //물체를 잡으러갈때 로봇팔 각도
  if (wait_flag) {
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = next_step3;
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (con_position) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 1층인지 2층인지
        {
          case 1:
          case 2:
          case 3:
          case 4: // 2층위치이면 2층 잡게 로봇 팔 움직이기.
            dae_Servomove(50,529,177,475,460,1000);     //적재함2층
            delay(1000);
            break;

          case 5:
          case 6:
          case 7:
          case 8:
            dae_Servomove(50,280,13,400,460, 1000);     //적재함1층
            delay(1000);
            break;
        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
}

void pixy_1x2x(int con_position, int next_step3){      //픽시로 미세조정할때 물체를 더 잘비추기위한 각도로 로봇팔 조정
  if (wait_flag) {
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = next_step3;
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (con_position) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 1층인지 2층인지
        {
          case 1:
          case 2:
          case 3:
          case 4: // 2층위치이면 2층 잡게 로봇 팔 움직이기.
            dae_Servomove(50,280,13,400,460, 1000);     //적재함2층 비출때
            delay(1000);
            break;

          case 5:
          case 6:
          case 7:
          case 8:
            dae_Servomove(50,400,13,400,460, 1000);     //적재함1층 비출때
            delay(1000);
            break;
        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
}

void arm_clear(int next_step4){    //로봇팔 초기화 (로봇팔위치에 따라 무게중심이 변하면서 오차가 생기므로)
  if (wait_flag) {
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = next_step4;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        dae_Servomove(790,840,100,700,465,1000);
        delay(1000);

        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
}

void dis_avgg(int next_level){     //장애물 앞까지 직진
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = next_level;  //next step
          set_cycle = 0;
          wait_flag = 0;
          dis_avg = 0;
        }
      }
      else {
        if (dis_avg_l > 390 || dis_avg_r > 390 ) {  //about 18 cm (가까워질수록 값이 커진다)
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
            dis_avg_l = 0;
            dis_avg_r = 0;
          }
        }
        else {
          psd();
          pid();
          x_val = 5;
          y_val = 0;
          w_val = 0;
        }
      }
}

void dis_avgg2(int next_level){     //앞으로 직진
  if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = next_level;  //next step
          set_cycle = 0;
          wait_flag = 0;
          dis_avg = 0;
        }
      }
      else {
        if (set_cycle > 40) {  //about 18 cm (가까워질수록 값이 커진다)
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
            dis_avg_l = 0;
            dis_avg_r = 0;
          }
        }
        else {
          pid();
          x_val = 5;
          y_val = 0;
          w_val = 0;
        }
      }
}

void pixy_get_X(int mission_num , int next_step){  //픽시에서 목표물체의 x좌표구하기
      int i;
      int while_seq=0;
      if (wait_flag) {
        if (set_cycle > 100) { //wait for
          m_seq = next_step;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else {
        if (set_cycle > 101) {  //wait for 1.0s before turn off timer5
          ////To use Pixy2, turn off timer 5
          Timer5.stop();
          delay(200);
          //parameter_clr();

          //------------------------------------------------------------ 픽시 캠 사용 ----------------------------------------
          detect_blocks = 0;

          pixy.ccc.getBlocks();
          detect_blocks = pixy.ccc.numBlocks;
          delay(50);
          Serial.println(detect_blocks);
          while (pixy_x == 0){  
          if (detect_blocks != 0)
          { 
            pixy.ccc.getBlocks();
            for (i=0; i<detect_blocks; i++){
            if(pixy.ccc.blocks[i].m_signature == mission_num)
            {
              pixy_x=pixy.ccc.blocks[i].m_x; 
            }

            else{         //물체를 인식했는데 원하는게 없다면 파란불
            digitalWrite(LED_G, HIGH);
            delay(200);
            digitalWrite(LED_G, LOW);
            delay(200);
            while_seq++;
            Serial.println(mission_num);
            if (while_seq> 10){   // 10번시도했는데 계속 안되면 그대로
              pixy_x= 158;
            }
            
            }
          }
          }
          else {          //물체를 아무것도 인식 못했다면 빨간불
            pixy.ccc.getBlocks();
            digitalWrite(LED_R, HIGH);
            delay(200);
            digitalWrite(LED_R, LOW);
            delay(200);
            detect_blocks = pixy.ccc.numBlocks;
          }
          }
          Timer5.restart();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
          
        }
}
}

void pixy_ymove(int next_step) { //픽시로 인식한 x좌표를 이용해 적재함 앞에서 미세조정
  Serial.print("pixy_x: ");
  Serial.print(pixy_x);
  Serial.println(' ');
  if (pixy_x> 252){ //오른쪽으로 많이이동
    if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = next_step;
          set_cycle = 0;
          wait_flag = 0;

        }
      }
      else {
        if (set_cycle > 55) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = -10;
          w_val = 0;
        }
      }
  }
  else if (pixy_x <= 252 && pixy_x > 189){ //오른쪽으로 조금이동
    if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = next_step;
          set_cycle = 0;
          wait_flag = 0;

        }
      }
      else {
        if (set_cycle > 30) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = -10;
          w_val = 0;
        }
      }
  }
  else if (pixy_x<= 189 && pixy_x > 126){ //제자리
    if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = next_step;
          set_cycle = 0;
          wait_flag = 0;

        }
      }
      else {
        if (set_cycle > 20) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = 0;
          w_val = 0;
        }
      }
  }
  else if (pixy_x<= 126 && pixy_x > 63){ //왼쪽으로 조금이동
    if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = next_step;
          set_cycle = 0;
          wait_flag = 0;

        }
      }
      else {
        if (set_cycle > 20) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = 10;
          w_val = 0;
        }
      }
  }
  else if (pixy_x<= 63 && pixy_x > 0) { // 왼쪽으로 많이 이동
    if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = next_step;
          set_cycle = 0;
          wait_flag = 0;

        }
      }
      else {
        if (set_cycle > 40) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = 10;
          w_val = 0;
        }
      }
  }

  else if (pixy_x == 0){
            digitalWrite(LED_R, HIGH);
            delay(50);
            digitalWrite(LED_R, LOW);
            delay(50);
  }
  pixy_x = 0;
  
}
