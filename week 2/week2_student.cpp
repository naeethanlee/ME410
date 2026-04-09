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

//gcc -o week1 week_1_student.cpp -lwiringPi  -lm

#define GYRO_LIMIT 300.0f
#define ROLL_LIMIT 45.0f
#define PITCH_LIMIT 45.0f
#define JOYSTICK_TIMEOUT 0.35f

int setup_imu();
void calibrate_imu();
void read_imu();
void update_filter();
void setup_joystick();
void trap(int signal);
void safety_check();

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
long time_curr;
long time_prev;
struct timespec te;
float yaw=0;
float pitch_angle=0;
float roll_angle=0;
float roll_accel=0;//accel-only roll (for graphing)
float pitch_accel=0;   // accel-only pitch (for graphing)
float roll_gyro_int=0; // gyro-integrated roll (for graphing)
float pitch_gyro_int=0;// gyro-integrated pitch (for graphing)
float program_time=0; // elapsed time in seconds

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
    //calibrate_imu();
    setup_joystick();
    signal(SIGINT, &trap);
    sleep(5);

    while(run_program==1)
    {
      joystick_data=*shared_memory;
      read_imu();
      update_filter();
      safety_check();
      printf("%.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",program_time,
         roll_angle, roll_accel, roll_gyro_int,
         pitch_angle, pitch_accel, pitch_gyro_int);
    }

    return 0;
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

void update_filter()
{

  //get current time in nanoseconds
  timespec_get(&te,TIME_UTC);
  time_curr=te.tv_nsec;
  //compute time since last execution
  float imu_diff=time_curr-time_prev;           
  
  //check for rollover
  if(imu_diff<=0)
  {
    imu_diff+=1000000000;
  }
  //convert to seconds
  imu_diff=imu_diff/1000000000;
  time_prev=time_curr;
  
  program_time+= imu_diff;

  //gyro-only integration
  roll_gyro_int+=  imu_data[3]* imu_diff;//gyroX drives roll
  pitch_gyro_int +=-imu_data[4]*imu_diff; //gyroY drives pitch (negated to match pitch_accel sign)

  //equation for the igh-pass gyro and low-pass accel
  float A = 0.02f;
  roll_angle= roll_accel*A +(1.0f- A) *(imu_data[3]*imu_diff+ roll_angle);
  pitch_angle =pitch_accel* A+ (1.0f -A) *(-imu_data[4] *imu_diff+pitch_angle);
}


//when cntrl+c pressed, kill motors

void trap(int signal)

{



   printf("ending program\n\r");

   run_program=0;
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

void safety_check()
{
  //gyro rate check
  if(imu_data[3]>GYRO_LIMIT || imu_data[3]<-GYRO_LIMIT ||
     imu_data[4]>GYRO_LIMIT || imu_data[4]<-GYRO_LIMIT ||
     imu_data[5]>GYRO_LIMIT || imu_data[5]<-GYRO_LIMIT)
  {
    printf("safety: gyro rate exceeded limit\n");
    run_program=0;
  }
  //roll angle check
  if(roll_angle>ROLL_LIMIT || roll_angle<-ROLL_LIMIT)
  {
    printf("safety: roll angle exceeded limit\n");
    run_program=0;
  }
  //pitch angle check
  if(pitch_angle>PITCH_LIMIT || pitch_angle<-PITCH_LIMIT)
  {
    printf("safety: pitch angle exceeded limit\n");
    run_program=0;
  }
  //joystick B button check (key1)
  if(joystick_data.key1==1)
  {
    printf("safety: B button pressed\n");
    run_program=0;
  }
  //joystick timeout check
  if(joystick_data.sequence_num != last_sequence_num)
  {
    last_sequence_num=joystick_data.sequence_num;
    last_joystick_time=program_time;
  }
  else if(program_time - last_joystick_time > JOYSTICK_TIMEOUT)
  {
    printf("safety: joystick timeout\n");
    run_program=0;
  }
}
