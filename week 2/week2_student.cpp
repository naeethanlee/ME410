#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <unistd.h>

/* got tired of typing raspberry lol
type $env:USERPROFILE\.ssh\id_rsa.pub | ssh pi@10.42.0.1 "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys"
*/

// gcc -o flight/wk2_student flight/wk2_student.cpp -lwiringPi -lm
// scp week2_student.cpp pi@10.42.0.1:/home/pi/flight/wk2_student.cpp
// for when u get the rx error sudo killall udp_rx


#define GYRO_LIMIT 300.0f
#define ROLL_LIMIT 45.0f
#define PITCH_LIMIT 45.0f
#define JOYSTICK_TIMEOUT 100.0f
#define THRUST_MAX 2000
#define THRUST_MIN 0


int setup_imu();
void calibrate_imu();
void read_imu();
void update_filter();
void setup_joystick();
void trap(int signal);
void kill_motors(const char* reason);
void safety_check();
void set_motor_values();
void motor_enable();
void set_motors(int motor0, int motor1, int motor2, int motor3);
void camera_control();

//global variables
int accel_address,gyro_address;
int print_counter=0;
float x_accel_calibration=0;
float y_accel_calibration=0;
float z_accel_calibration=0;
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0;
float accel_z_calibration=0;
float imu_data[6]; //accel xyz,  gyro xyz,
float time_curr=0;
float time_prev=0;
struct timespec te;
float yaw=0;
float pitch_angle=0;
float roll_angle=0;
float roll_accel=0;//accel-only roll (for graphing)
float pitch_accel=0;   // accel-only pitch (for graphing)
float roll_gyro_int=0; // gyro-integrated roll (for graphing)
float pitch_gyro_int=0;// gyro-integrated pitch (for graphing)
float program_time=0; // elapsed time in seconds
float dt=0; // timestep in seconds

// Milestone 3
int motor_commands[]={0,0,0,0}; // 0 and 2 front, 1 and 3 back(left then right)
int motor_paused=1; // start paused; A to pause, Y to run
float thrust=0;
float thrust_neutral=900; // neutral thrust value
float thrust_amplitude=0; // joystick thrust range
float pitch_amplitude=15; // max commanded pitch (deg)
float pitch_gain = 15.5; // pitch P gain
float derivative_gain = 3.5; // pitch D gain
float integral_pitch = 0; // integral pitch
float integral_gain = 0.05; // pitch I gain
float integral_saturate = 90; // max and min integral value

float roll_amplitude = 15; // joystick roll read (degrees max)
float roll_gain = 15.5; // roll P gain
float roll_derivative_gain = 3.5; // roll D gain
float integral_roll = 0; // roll integral accumulator
float integral_gain_roll = 0.05; // roll I gain
float integral_saturate_roll = 90; // roll integral clamp

////////////////////////////////////////////////////////////////

// Milestone 8
bool auto_mode = false;
int x_prev = 0;
float yaw_cam_p = -3.3; // try 1

float cam_desired_y = 0;
float cam_desired_x = 0;
float cam_estimate_y = 0;
float cam_estimate_x = 0;

// Camera Pitch PID Values
float cam_pitch_p = 0;
float cam_pitch_d = 0;
float cam_y_prev = 0;

float yaw_gain = 4.2; // yaw P gain
float yaw_amplitude = 65.0; // max commanded yaw rate (deg/s)



//
// Week 4
//
int motor_address;

  struct Joystick // data struct from udp_rx_cam_joy
  {
  int key0;
  int key1;
  int key2;
  int key3;
  int pitch;
  int roll;
  int yaw;
  int thrust;
  float x;
  float y;
  float z;
  float camera_yaw;
  int success;
  int sequence_num;
};

Joystick* shared_memory;
Joystick joystick_data;
int run_program=1;
int last_sequence_num=0;
float last_joystick_time=0;

int main(int argc, char *argv[])
{

    setup_imu();
    calibrate_imu();
    motor_address=wiringPiI2CSetup(0x56);
    motor_enable();
    setup_joystick();
    signal(SIGINT, &trap);
    timespec_get(&te, TIME_UTC);
    time_prev=te.tv_nsec;
    read_imu();
    pitch_angle=pitch_accel;
    roll_angle=roll_accel;

    while(run_program==1)
    {
      joystick_data=*shared_memory;
      read_imu();
      update_filter();
      safety_check();
      if(auto_mode == false){
        set_motor_values();
      }      
      else{
        camera_control();
      }
      printf("%d %d %d %d\n", motor_commands[0], motor_commands[1], motor_commands[2], motor_commands[3]);
      set_motors(motor_commands[0], motor_commands[1], motor_commands[2], motor_commands[3]);
//       if(print_counter % 20 == 0)
//         printf(
// "joystick: %d %d %d %d | yaw: %d pitch: %d roll: %d thrust: %d | "
// "x: %.3f y: %.3f z: %.3f yaw: %.3f | success: %d seq: %d\n",
// joystick_data.key0,
// joystick_data.key1,
// joystick_data.key2,
// joystick_data.key3,
// joystick_data.yaw,
// joystick_data.pitch,
// joystick_data.roll,
// joystick_data.thrust,
// joystick_data.x,
// joystick_data.y,
// joystick_data.z,
// joystick_data.camera_yaw,
// joystick_data.success,
// joystick_data.sequence_num
// );
    }

    return 0;
}

void calibrate_imu()
{
  float x_gyro_calibration_sum=0;
  float y_gyro_calibration_sum=0;
  float z_gyro_calibration_sum=0;
  float pitch_calibration_sum=0;
  float roll_calibration_sum=0;

  for(int i=0; i<1000; i++){
    read_imu();

    //accum gyro dps
    x_gyro_calibration_sum+=imu_data[3];
    y_gyro_calibration_sum+=imu_data[4];
    z_gyro_calibration_sum+=imu_data[5];

    //accum accel angles
    pitch_calibration_sum+=atan2(imu_data[1], imu_data[0])*180.0/M_PI;
    roll_calibration_sum+=atan2(imu_data[2], imu_data[0])*180.0/M_PI;
  }

  //avg offsets, subtracted later in read_imu
  x_gyro_calibration=x_gyro_calibration_sum/1000;
  y_gyro_calibration=y_gyro_calibration_sum/1000;
  z_gyro_calibration=z_gyro_calibration_sum/1000;
  pitch_calibration=pitch_calibration_sum/1000;
  roll_calibration=roll_calibration_sum/1000;

  printf("calibration complete, %f %f %f %f %f %f\n\r",x_gyro_calibration,y_gyro_calibration,
    z_gyro_calibration,roll_calibration,pitch_calibration,accel_z_calibration);
}

void read_imu()
{
  uint8_t address=0;
  float ax=0;
  float az=0;
  float ay=0;
  int vh=0;
  int vl=0;
  int vw=0;
  float pitch_measure=0;
  float roll_measure=0;


  //accel reads

  address=0x12;//accelX reg
  vw=wiringPiI2CReadReg16(accel_address,address);
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[0]=((float)vw)*3.0/32768.0;//convert to g's

  address=0x14;//accelY reg
  vw=wiringPiI2CReadReg16(accel_address,address);
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[1]=((float)vw)*3.0/32768.0;//convert to g's

  address=0x16;//accelZ reg
  vw=wiringPiI2CReadReg16(accel_address,address);
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[2]=((float)vw)*3.0/32768.0;//convert to g's

  //gyro reads

  address=0x02;//gyroX reg
  vw=wiringPiI2CReadReg16(gyro_address,address);
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[3]=((float)vw)*1000.0/32768.0 - x_gyro_calibration;//convert to degrees/sec

  address=0x04;//gyroY reg
  vw=wiringPiI2CReadReg16(gyro_address,address);
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[4]=((float)vw)*1000.0/32768.0 - y_gyro_calibration;//convert to degrees/sec

  address=0x06;//gyroZ reg
  vw=wiringPiI2CReadReg16(gyro_address,address);
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[5]=((float)vw)*1000.0/32768.0 - z_gyro_calibration;//convert to degrees/sec

  pitch_measure=(atan2(imu_data[1], imu_data[0])*180.0/M_PI) - pitch_calibration;
  roll_measure=(atan2(imu_data[2], imu_data[0])*180.0/M_PI) - roll_calibration;

  pitch_accel=pitch_measure;
  roll_accel=roll_measure;
}


int setup_imu()
{
  wiringPiSetup ();

  accel_address=wiringPiI2CSetup(0x19) ;
  gyro_address=wiringPiI2CSetup(0x69) ;

  if(accel_address==-1)
  {
    printf("-----cant connect to accel I2C device %d --------\n",accel_address);
    return -1;
  }
  else if(gyro_address==-1)
  {
    printf("-----cant connect to gyro I2C device %d --------\n",gyro_address);
    return -1;
  }
  else
  {
    printf("all i2c devices detected\n");
    sleep(1);
    wiringPiI2CWriteReg8(accel_address, 0x7d, 0x04); //power on accel
    wiringPiI2CWriteReg8(accel_address, 0x41, 0x00); //accel range to +_3g
    wiringPiI2CWriteReg8(accel_address, 0x40, 0x89); //high speed filtered accel

    wiringPiI2CWriteReg8(gyro_address, 0x11, 0x00);//power on gyro
    wiringPiI2CWriteReg8(gyro_address, 0x0f, 0x01);//set gyro to +-1000dps
    wiringPiI2CWriteReg8(gyro_address, 0x01, 0x03);//set data rate and bandwith

    sleep(1);
  }
  return 0;
}

void update_filter()
{
  //get current time in nanoseconds
  timespec_get(&te,TIME_UTC);
  time_curr=te.tv_nsec;
  //compute time since last execution
  dt=time_curr - time_prev;

  //check for rollover
  if(dt<=0)
  {
    dt+=1000000000.0;
  }
  //convert to seconds
  dt=dt/1000000000.0;
  time_prev=time_curr;

  program_time+= dt;

  //gyro-only integration
  roll_gyro_int += (imu_data[4] * dt); //gyroX drives roll
  pitch_gyro_int += (imu_data[5] * dt); //gyroY drives pitch

  //complementary filter: high-pass gyro, low-pass accel
  float A = 0.01f;
  pitch_angle = pitch_accel*A + (1.0f-A) * (pitch_angle - imu_data[5]*dt);
  roll_angle  = roll_accel *A + (1.0f-A) * (roll_angle  + imu_data[4]*dt);
}


//when cntrl+c pressed, kill motors

void trap(int signal)
{
  set_motors(0, 0, 0, 0);
  printf("Control+C: killing motors and ending program\n\r");
  run_program=0;
}

void setup_joystick()
{

  int segment_id;
  struct shmid_ds shmbuffer;
  int segment_size;
  const int shared_segment_size=0x6400;
  int smhkey=33222;

  segment_id=shmget(smhkey, shared_segment_size, IPC_CREAT | 0666);
  shared_memory=(Joystick*)shmat(segment_id, 0, 0);
  printf("shared memory attached at address %p\n", shared_memory);
  shmctl(segment_id, IPC_STAT, &shmbuffer);
  segment_size=shmbuffer.shm_segsz;
  printf("segment size: %d\n", segment_size);

}

void kill_motors(const char* reason)
{
  set_motors(0, 0, 0, 0);
  printf("safety: %s — killing motors and ending program\n", reason);
  run_program=0;
}

void safety_check()
{
  if(imu_data[3]>GYRO_LIMIT || imu_data[3]<-GYRO_LIMIT ||
     imu_data[4]>GYRO_LIMIT || imu_data[4]<-GYRO_LIMIT ||
     imu_data[5]>GYRO_LIMIT || imu_data[5]<-GYRO_LIMIT)
    kill_motors("gyro rate exceeded limit");

  if(roll_angle>ROLL_LIMIT || roll_angle<-ROLL_LIMIT)
    kill_motors("roll angle exceeded limit");
  if(pitch_angle>PITCH_LIMIT || pitch_angle<-PITCH_LIMIT)
    kill_motors("pitch angle exceeded limit");

  if(joystick_data.key1==1)
    kill_motors("joystick kill button pressed");

  if(joystick_data.key0==1)
  {
    motor_paused=1;
    integral_pitch=0;
    integral_roll=0;
    printf("motors PAUSED\n");
  }

  if(joystick_data.key3==1)
  {
    motor_paused=0;
    printf("motors RUNNING\n");
  }

  if(joystick_data.sequence_num != last_sequence_num)
  {
    last_sequence_num=joystick_data.sequence_num;
    last_joystick_time=program_time;
  }
  else if(program_time-last_joystick_time > JOYSTICK_TIMEOUT)
    kill_motors("joystick timeout");

  if(x_prev == 0){
    if(joystick_data.key2 == 1){
      auto_mode = !auto_mode;
      printf("%d\n", auto_mode);
    }
  }
  x_prev = joystick_data.key2;
}

void set_motor_values()
{
  /* thrust */
  float joystick_thrust_value=joystick_data.thrust-128;
  thrust=thrust_neutral-(joystick_thrust_value/128.0f*thrust_amplitude);

  /* pitch */
  float joystick_pitch_value=(float)(joystick_data.pitch)-128.0f;
  float pitch_desired=joystick_pitch_value/128.0f*pitch_amplitude;
  float pitch_error=pitch_desired-pitch_angle;

  if(!motor_paused)
  {
    integral_pitch+=integral_gain*pitch_error;
    if(integral_pitch>integral_saturate) integral_pitch=integral_saturate;
    else if(integral_pitch<-integral_saturate) integral_pitch=-integral_saturate;
  }

  float pitch_pid=pitch_gain*pitch_error+derivative_gain*imu_data[5]+integral_pitch;

  /* roll */
  float joystick_roll_value=(float)(joystick_data.roll)-128.0f;
  float roll_desired=joystick_roll_value/128.0f*roll_amplitude;
  float roll_error=roll_desired-roll_angle;

  if(!motor_paused)
  {
    integral_roll+=integral_gain_roll*roll_error;
    if(integral_roll>integral_saturate_roll) integral_roll=integral_saturate_roll;
    else if(integral_roll<-integral_saturate_roll) integral_roll=-integral_saturate_roll;
  }

  float roll_pid=roll_gain*roll_error-roll_derivative_gain*imu_data[4]+integral_roll;

  /* yaw */
  float yaw_pid = 0;
  float yaw_rate=-imu_data[3];
  float joystick_yaw_value=(float)(joystick_data.yaw)-128.0f;
  float yaw_desired=joystick_yaw_value/128.0f*yaw_amplitude;
  yaw_pid=yaw_gain*(yaw_desired-yaw_rate);

  motor_commands[0]=(int)(thrust-pitch_pid-roll_pid+yaw_pid); // front-left
  motor_commands[1]=(int)(thrust+pitch_pid-roll_pid-yaw_pid); // back-left
  motor_commands[2]=(int)(thrust-pitch_pid+roll_pid-yaw_pid); // front-right
  motor_commands[3]=(int)(thrust+pitch_pid+roll_pid+yaw_pid); // back-right

  if(motor_paused)
  {
    motor_commands[0]=1;
    motor_commands[1]=1;
    motor_commands[2]=1;
    motor_commands[3]=1;
  }

  print_counter++;
  // if(print_counter % 20 == 0)
    // printf("%.4f %d %d %d %d | pitch:%.2f des:%.2f | roll:%.2f des:%.2f | yaw_des:%.2f rate:%.2f | thr:%.0f\n",
    //        program_time,
    //        motor_commands[0], motor_commands[1], motor_commands[2], motor_commands[3],
    //        pitch_angle, pitch_desired, roll_angle, roll_desired, yaw_desired, yaw_rate, thrust);
}

void motor_enable()
{

    uint8_t motor_id=0;
    uint8_t special_command=0;
    uint16_t commanded_speed=0;
    uint8_t data[2];

    int cal_delay=50;

    for(int i=0;i<1000;i++)
    {

      motor_id=0;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;
      wiringPiI2CWrite(motor_address,data[0]);
      usleep(cal_delay);
      wiringPiI2CWrite(motor_address,data[1]);


      usleep(cal_delay);
      motor_id=1;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;
      wiringPiI2CWrite(motor_address,data[0]);
      usleep(cal_delay);
      wiringPiI2CWrite(motor_address,data[1]);

      usleep(cal_delay);
      motor_id=2;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;
      wiringPiI2CWrite(motor_address,data[0]);
      usleep(cal_delay);
      wiringPiI2CWrite(motor_address,data[1]);


      usleep(cal_delay);
      motor_id=3;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;
      wiringPiI2CWrite(motor_address,data[0]);
      usleep(cal_delay);
      wiringPiI2CWrite(motor_address,data[1]);
      usleep(cal_delay);

    }

    for(int i=0;i<2000;i++)
    {

      motor_id=0;
      commanded_speed=50;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;
      wiringPiI2CWrite(motor_address,data[0]);
      usleep(cal_delay);
      wiringPiI2CWrite(motor_address,data[1]);


      usleep(cal_delay);
      motor_id=1;
      commanded_speed=50;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;
      wiringPiI2CWrite(motor_address,data[0]);
      usleep(cal_delay);
      wiringPiI2CWrite(motor_address,data[1]);

      usleep(cal_delay);
      motor_id=2;
      commanded_speed=50;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;
      wiringPiI2CWrite(motor_address,data[0]);
      usleep(cal_delay);
      wiringPiI2CWrite(motor_address,data[1]);


      usleep(cal_delay);
      motor_id=3;
      commanded_speed=50;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;
      wiringPiI2CWrite(motor_address,data[0]);
      usleep(cal_delay);
      wiringPiI2CWrite(motor_address,data[1]);
      usleep(cal_delay);

    }


    for(int i=0;i<500;i++)
    {

      motor_id=0;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;
      wiringPiI2CWrite(motor_address,data[0]);
      usleep(cal_delay);
      wiringPiI2CWrite(motor_address,data[1]);


      usleep(cal_delay);
      motor_id=1;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;
      wiringPiI2CWrite(motor_address,data[0]);
      usleep(cal_delay);
      wiringPiI2CWrite(motor_address,data[1]);

      usleep(cal_delay);
      motor_id=2;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;
      wiringPiI2CWrite(motor_address,data[0]);
      usleep(cal_delay);
      wiringPiI2CWrite(motor_address,data[1]);


      usleep(cal_delay);
      motor_id=3;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;
      wiringPiI2CWrite(motor_address,data[0]);
      usleep(cal_delay);
      wiringPiI2CWrite(motor_address,data[1]);
      usleep(cal_delay);

    }

}

void set_motors(int motor0, int motor1, int motor2, int motor3)
{
    if(motor0<0)
      motor0=0;
    if(motor0>THRUST_MAX)
      motor0=THRUST_MAX;
    if(motor1<0)
      motor1=0;
    if(motor1>THRUST_MAX)
      motor1=THRUST_MAX;
    if(motor2<0)
      motor2=0;
    if(motor2>THRUST_MAX)
      motor2=THRUST_MAX;
    if(motor3<0)
      motor3=0;
    if(motor3>THRUST_MAX)
      motor3=THRUST_MAX;



    uint8_t motor_id=0;
    uint8_t special_command=0;
    uint16_t commanded_speed=0;
    uint8_t data[2];

    int com_delay=500;

    motor_id=0;
    commanded_speed=motor0;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;
    wiringPiI2CWrite(motor_address,data[0]);
    usleep(com_delay);
    wiringPiI2CWrite(motor_address,data[1]);


    usleep(com_delay);
    motor_id=1;
    commanded_speed=motor1;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;
    wiringPiI2CWrite(motor_address,data[0]);
    usleep(com_delay);
    wiringPiI2CWrite(motor_address,data[1]);

    usleep(com_delay);
    motor_id=2;
    commanded_speed=motor2;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;
    wiringPiI2CWrite(motor_address,data[0]);
    usleep(com_delay);
    wiringPiI2CWrite(motor_address,data[1]);


    usleep(com_delay);
    motor_id=3;
    commanded_speed=motor3;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;
    wiringPiI2CWrite(motor_address,data[0]);
    usleep(com_delay);
    wiringPiI2CWrite(motor_address,data[1]);
    usleep(com_delay);


}

void camera_control()
{
  /* thrust */
  // float joystick_thrust_value=joystick_data.thrust-128;
  // thrust=thrust_neutral-(joystick_thrust_value/128.0f*thrust_amplitude);
  thrust = thrust_neutral;

  /* pitch */
  // joystick control
  float joystick_pitch_value=(float)(joystick_data.pitch)-128.0f;
  float joystick_pitch_desired=joystick_pitch_value/128.0f*pitch_amplitude;
  float joystick_pitch_error=joystick_pitch_desired-pitch_angle;
  float joystick_pitch_pid=pitch_gain*joystick_pitch_error+derivative_gain*imu_data[5]+integral_pitch;


  float camera_current_loc_y = joystick_data.y;
  cam_estimate_y = (cam_estimate_y * 0.6) + (camera_current_loc_y * 0.4);
  float camera_pitch_desired = cam_pitch_p * (cam_estimate_y - cam_desired_y) - cam_pitch_d * 
                              (cam_estimate_y - cam_y_prev) / (program_time - time_prev);
  

  float pitch_pid = joystick_pitch_pid * 0.5 + camera_pitch_desired * 0.5;
  pitch_pid = 0;

  // cam_y_prev = camera_current_loc_y;
  // time_prev = program_time;







  


  // /* roll */
  float roll_pid = 0;
  // float joystick_roll_value=(float)(joystick_data.roll)-128.0f;
  // float roll_desired=joystick_roll_value/128.0f*roll_amplitude;
  // float roll_error=roll_desired-roll_angle;

  // if(!motor_paused)
  // {
  //   integral_roll+=integral_gain_roll*roll_error;
  //   if(integral_roll>integral_saturate_roll) integral_roll=integral_saturate_roll;
  //   else if(integral_roll<-integral_saturate_roll) integral_roll=-integral_saturate_roll;
  // }

  // float roll_pid=roll_gain*roll_error-roll_derivative_gain*imu_data[4]+integral_roll;
  

  // yaw control
  float yaw_pid;
  float cam_yaw = joystick_data.camera_yaw;
  yaw_pid = cam_yaw * yaw_cam_p;

  motor_commands[0]=(int)(thrust-pitch_pid-roll_pid+yaw_pid); // front-left
  motor_commands[1]=(int)(thrust+pitch_pid-roll_pid-yaw_pid); // back-left
  motor_commands[2]=(int)(thrust-pitch_pid+roll_pid-yaw_pid); // front-right
  motor_commands[3]=(int)(thrust+pitch_pid+roll_pid+yaw_pid); // back-right

  if(motor_paused)
  {
    motor_commands[0]=1;
    motor_commands[1]=1;
    motor_commands[2]=1;
    motor_commands[3]=1;
  }
}