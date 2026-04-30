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
#define GYRO_LIMIT 300.0f
#define ROLL_LIMIT 45.0f
#define PITCH_LIMIT 45.0f
#define JOYSTICK_TIMEOUT 0.35f
#define THRUST_MAX 2000f
#define THRUST_MIN 0f

// gcc -o motor_test motor_test.cpp -lwiringPi  -lm
// scp motor_test.cpp pi@10.42.0.1:/home/pi/flight/motor_test.cpp


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

//global variables
int accel_address,gyro_address;
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
int motor_commands[] = {0, 0, 0, 0}; // 0 and 2 forward, 1 and 3 back(left then right)
float thrust=0;
float thrust_neutral=800; // neutral thrust value
float thrust_amplitude=100; // joystick thrust read
float pitch_amplitude=10; // joystick pitch read
float pitch_gain = 10; // pitch gain
float derivative_gain = 0; // derivative gain
float integral_pitch = 0; // integral pitch
float integral_gain = 0; // integral gain * Perror
float integral_saturate = 100; // max and min integral value

// 
// Week 4
//
int motor_address;

struct Joystick
{
  int key0;
  int key1;
  int key2;
  int key3;
  int pitch;
  int roll;
  int yaw;
  int thrust;
  int sequence_num;
};

Joystick* shared_memory;
Joystick joystick_data;
int run_program=1;
int last_sequence_num=0;
float last_joystick_time=0;

int main (int argc, char *argv[])
{

    setup_imu();
    calibrate_imu();
    motor_address=wiringPiI2CSetup(0x56); 
    motor_enable();
    setup_joystick();
    signal(SIGINT, &trap);

    while(run_program==1)
    {
      joystick_data=*shared_memory;
      read_imu();
      update_filter();
      safety_check();
      set_motor_values();
      set_motors(motor_commands[0], motor_commands[1], motor_commands[2], motor_commands[3]);
    }

    return 0;
}

int setup_imu()
{
  wiringPiSetup ();
  
  //setup imu on I2C
  accel_address=wiringPiI2CSetup (0x19) ; 
  gyro_address=wiringPiI2CSetup (0x69) ; 
  
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

void read_imu()
{
  uint8_t address=0;//todo: set address value for accel x value 
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
  //convert from 2's complement
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }       
  imu_data[0]=((float)vw)*3.0/32768.0;//convert to g's     
  //imu_data[0]=(vw - x_accel_calibration)*3/32768;//convert to g's  
  
  address=0x14;//accelY reg
  vw=wiringPiI2CReadReg16(accel_address,address);   
  //convert from 2's complement
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  } 
  imu_data[1]=((float)vw)*3.0/32768.0;//convert to g's           
  //imu_data[1]=(vw - y_accel_calibration)*3/32768;//convert to g's  
  
  address=0x16;//accelZ reg
  vw=wiringPiI2CReadReg16(accel_address,address);   
  //convert from 2's complement     
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }        
  imu_data[2]=((float)vw)*3.0/32768.0;//convert to g's    
  //imu_data[2]=(vw - z_accel_calibration)*3/32768;//convert to g's  
  
  
     

  //gyro reads

  address=0x02;//gyroX reg
  vw=wiringPiI2CReadReg16(gyro_address,address);   
  //convert from 2's complement          
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[3]=((float)vw )*1000.0/32768.0 - x_gyro_calibration;//convert to degrees/sec
  
  address=0x04;//gyroY reg
  vw=wiringPiI2CReadReg16(gyro_address,address);    
  //convert from 2's complement              
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[4]=((float)vw)*1000.0/32768.0 - y_gyro_calibration;//convert to degrees/sec
  
  address=0x06;//gyroZ reg
  vw=wiringPiI2CReadReg16(gyro_address,address);   
  //convert from 2's complement               
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[5]=((float)vw)*1000.0/32768.0 - z_gyro_calibration;//convert to degrees/sec  

  pitch_measure=-((atan2(imu_data[1], imu_data[0])*180.0/M_PI) - pitch_calibration);
  roll_measure=(atan2(imu_data[2], imu_data[0])*180.0/M_PI) - roll_calibration;

  pitch_accel=pitch_measure;
  roll_accel=roll_measure;
  
  //printf("%10.5f %10.5f %10.5f %10.5f %10.5f\n", imu_data[3], imu_data[4], imu_data[5], pitch_measure, roll_measure);
}

void calibrate_imu()
{
  //sum vars for avg
  float x_gyro_calibration_sum=0;
  float y_gyro_calibration_sum=0;
  float z_gyro_calibration_sum=0;
  float pitch_calibration_sum=0;
  float roll_calibration_sum=0;
  
  // avg 1000 samples stationary to hardware offset
  for(int i = 0; i < 1000; i++){
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

void motor_enable()
{
  
    uint8_t motor_id=0;
    uint8_t special_command=0;
    uint16_t commanded_speed_0=1000;    
    uint16_t commanded_speed_1=0;
    uint16_t commanded_speed=0;
    uint8_t data[2]; 
    
    int cal_delay=100;
    
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
    // printf("%d %d %d %d\n", motor0, motor1, motor2, motor3);

    if(motor0<0)
      motor0=0;
    if(motor0>2000)
      motor0=2000;
    if(motor1<0)
      motor1=0;
    if(motor1>2000)
      motor1=2000;
    if(motor2<0)
      motor2=0;
    if(motor2>2000)
      motor2=2000;
    if(motor3<0)
      motor3=0;
    if(motor3>2000)
      motor3=2000;
      
    
    
    uint8_t motor_id=0;
    uint8_t special_command=0;
    uint16_t commanded_speed_0=1000;    
    uint16_t commanded_speed_1=0;
    uint16_t commanded_speed=0;
    uint8_t data[2]; 
    
   // wiringPiI2CWriteReg8(motor_address, 0x00,data[0] );
    //wiringPiI2CWrite (motor_address,data[0]) ;
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

void setup_joystick()
{

  int segment_id;
  struct shmid_ds shmbuffer;
  int segment_size;
  const int shared_segment_size = 0x6400;
  int smhkey=33222;

  /* Allocate a shared memory segment.  */
  segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666);
  /* Attach the shared memory segment.  */
  shared_memory = (Joystick*) shmat (segment_id, 0, 0);
  printf ("shared memory attached at address %p\n", shared_memory);
  /* Determine the segment's size. */
  shmctl (segment_id, IPC_STAT, &shmbuffer);
  segment_size  =               shmbuffer.shm_segsz;
  printf ("segment size: %d\n", segment_size);
  /* Write a string to the shared memory segment.  */
  //sprintf (shared_memory, "test!!!!.");

}

void trap(int signal)
{
  motor_commands[0]=0;
  motor_commands[1]=0;
  motor_commands[2]=0;
  motor_commands[3]=0;
  set_motor_values();
  printf("Control+C: killing motors and ending program\n\r");
  run_program=0;
}

void set_motor_values()
{
  /* thrust */
  float joystick_thrust_value = 0; // temp variable of joystick
  joystick_thrust_value = joystick_data.thrust - 128;

  // lerp
  thrust = thrust_neutral - (joystick_thrust_value / 128 * thrust_amplitude);

  /* pitch */
  //
  // proportional control
  // 
  float pitch_error = 0;
  float pitch_measured = pitch_angle;
  float pitch_desired = 0;
  float joystick_pitch_value = (float)(joystick_data.pitch) - 128.0;

  // lerp
  pitch_desired = -(joystick_pitch_value / 128.0 * pitch_amplitude);
  
  pitch_error = pitch_desired - pitch_measured; // pitch error calculation

  // front motors decrease, rear motors increase
  // motor_commands[0] = (int)(thrust + (pitch_gain * pitch_error)); // motor 1
  // motor_commands[2] = (int)(thrust + (pitch_gain * pitch_error));
  // motor_commands[1] = (int)(thrust - (pitch_gain * pitch_error));
  // motor_commands[3] = (int)(thrust - (pitch_gain * pitch_error));
  // printf("%.4f %d %d %.4f %.4f %.4f\n",program_time,
  //        motor_commands[0], motor_commands[1], thrust,
  //        pitch_desired * 10, pitch_measured * 10);

  //
  // derivative control
  //

  // motor_commands[0] = (int)(thrust - (derivative_gain * imu_data[5])); // motor 1
  // motor_commands[2] = (int)(thrust - (derivative_gain * imu_data[5]));
  // motor_commands[1] = (int)(thrust + (derivative_gain * imu_data[5]));
  // motor_commands[3] = (int)(thrust + (derivative_gain * imu_data[5]));
  // printf("%.4f %d %d %.4f %.4f %.4f\n",program_time,
  //        motor_commands[0], motor_commands[1], pitch_measured * 10,
  //        imu_data[5], thrust);
  
  // integral
  integral_pitch += integral_gain * pitch_error;
  if(integral_pitch > integral_saturate)
    integral_pitch = integral_saturate;
  else if(integral_pitch < -integral_saturate)
    integral_pitch = -integral_saturate;

  // motor_commands[0] = (int)(thrust - (integral_pitch)); // motor 1
  // motor_commands[2] = (int)(thrust - (integral_pitch));
  // motor_commands[1] = (int)(thrust + (integral_pitch));
  // motor_commands[3] = (int)(thrust + (integral_pitch));

  // printf("%.4f %d %d %.4f %.4f %.4f\n",program_time,
  //        motor_commands[0], motor_commands[1], pitch_measured * 10,
  //        pitch_desired * 10, thrust);


  // PID combined
  float pid = (pitch_gain * pitch_error) - (derivative_gain * imu_data[5]) - (integral_pitch);

  // motor_commands[0] = (int)(thrust + pid);
  // motor_commands[2] = (int)(thrust + pid);
  // motor_commands[1] = (int)(thrust - pid);
  // motor_commands[3] = (int)(thrust - pid);

  motor_commands[0] = 500;
  motor_commands[2] = 500;
  motor_commands[1] = 500;
  motor_commands[3] = 500;


  printf("%d %d %d %d\n", motor_commands[0],
         motor_commands[1], motor_commands[2],
         motor_commands[3]);
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
    dt+=1000000000;
  }
  //convert to seconds
  dt=dt/1000000000;
  time_prev=time_curr;

  program_time+= dt;

  //gyro-only integration
  roll_gyro_int += (imu_data[4] * dt); //gyroX drives roll
  pitch_gyro_int += (imu_data[5] * dt); //gyroY drives pitch (negated to match pitch_accel sign)

  //equation for the igh-pass gyro and low-pass accel
  float A = 0.02f;
  roll_angle= roll_accel*A +(1.0f- A) *(imu_data[4]*dt + roll_angle);
  pitch_angle = pitch_accel* A+ (1.0f -A) * (imu_data[5]*dt+ pitch_angle);
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

  if(joystick_data.sequence_num != last_sequence_num)
  {
    last_sequence_num=joystick_data.sequence_num;
    last_joystick_time=program_time;
  }
  else if(program_time - last_joystick_time > JOYSTICK_TIMEOUT)
    kill_motors("joystick timeout");
}

void kill_motors(const char* reason)
{
  motor_commands[0]=0;
  motor_commands[1]=0;
  motor_commands[2]=0;
  motor_commands[3]=0;
  set_motor_values();
  printf("safety: %s — killing motors and ending program\n", reason);
  run_program=0;
}