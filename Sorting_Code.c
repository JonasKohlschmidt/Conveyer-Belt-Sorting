#include <stdio.h>
#include <stdlib.h>
#include <rtai_shm.h>
#include <rtai_lxrt.h>
#include <sys/mman.h>
#include "rtai_sched.h"
#include <pthread.h>
#include <comedilib.h>
#include "rtai_sem.h"
#include "regul.h"
#include "fixpoint.h"
#include "Fixpoint_PD-controller.h"


/*************** variables ***********************************************************/
unsigned int name_main,name_skid,name_belt, name_sensor, name_timer;
unsigned int Belt_data_write, Skid_data_write;
double Ts = 2e6;
double Tss = 1e5; //Sensor 100000 ns => 0,1ms period
int stopflag = 0;
char menu = 0;
char menu2 = 0;
char menu3 = 0;


// RT
pthread_t skidThread, beltThread, sensorThread, timerThread;
RT_TASK *taskMain, *taskBelt, *taskSkid, *tasksensor, *tasktimer;
RTIME currenttime, starttime, sens_start_time, sens_stop_time, skidtime, conveyertime;
struct sched_param mystruct;
statetype p_belt, p_skid;

// Comedi
lsampl_t Belt_data_read, Skid_data_read, ADdata_sens;
comedi_t *ADC;
lsampl_t ADdata_PD,DAdata_PD;

//Semphore declaration
SEM *my_sem;
unsigned int sem_name;

//Skid Controller (PD)
double PD_ref=5.3; // left:6.3, middle:5.3, right:4.2
double PD_data_vltg, PD_data_write;
double PD_e=0;

// no oscillation
double PD_a1=-0.7408;
double PD_b0=121.8;
double PD_b1=-114.8;

/*// minimal oscillation
double PD_a1=-0.8704;
double PD_b0=58.87;
double PD_b1=-55.47;*/

//Fixed point PD-controller
short b0m_PD = 31181;
short b1m_PD = -29388;
short a1m_PD = -24274;
short e_PD = 0;
short pos = 3270; //3050 - 3230 - 3400
int x = 0;

//Belt Controller (PI)
double PI_ref=0; // larger than zero!!!!
double PI_data_vltg, PI_data_write;
double PI_e=0;
double PI_a1=-1;
double PI_b0=0.9955;
double PI_b1=-0.984;
double speed = 0;

//Statistics
int toosmall=0;
int small=0;
int medium=0;
int large=0;
int toolarge=0;
double small_ref=5;
double medium_ref=7.5;
double large_ref=10;


double PD_ref_matrix[2][8] = {{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
double buffer;
int length = 0;
int j;


// Functions
void *Skid(void *arg);
void *Belt(void *arg);
void *Sensor(void *arg);
void *Timer(void *arg);


/*********************************************** Main ********************************************************/
int main()
{
  // Allow root
  rt_allow_nonroot_hrt();
  mlockall(MCL_CURRENT | MCL_FUTURE);

  // Softrealtime init
  mystruct.sched_priority=sched_get_priority_max(SCHED_FIFO);
  sched_setscheduler(0, SCHED_FIFO,&mystruct);
  rt_make_soft_real_time();

  //Semaphore init
  sem_name=nam2num("semkk");
  my_sem=rt_typed_sem_init(sem_name,1,CNT_SEM);

  // Comedi init
  ADC = comedi_open("/dev/comedi2");
  if(ADC == NULL){
      printf("Couldn't open ADC\n");
  }

  // Create task for main thread
  name_main=nam2num("Main");
  taskMain=rt_task_init(name_main, 0x00FF, 0, 0);

  // Create threads
  pthread_create(&skidThread,NULL,Skid,NULL);
  pthread_create(&beltThread,NULL,Belt,NULL);
  pthread_create(&sensorThread,NULL,Sensor,NULL);
  pthread_create(&timerThread,NULL,Timer,NULL);

  while(stopflag==0){
    printf( "\n*******************MENU************************* \n"
    "                                                           \n"
    " Select one of the options below                           \n\n"
    " 1: Show paramater values                                  \n"
    " 2: Set skid position                                      \n"
    " 3: Set Belt speed                                         \n"
    " 4: Show Statistics                                        \n"
    " 5: Change length categories                               \n"
    " 6: Close program                                          \n"
    "                                                           \n"
    "***********************************************  \n\n\n");
    //read input
    fflush(stdout);
    scanf("%s",&menu);
    fflush(stdin);
    

    if(menu=='1'){
      printf("Program paramaters is set to:\n\n");
      printf("PD controller\tPd fixpoint\tPI controller\n");
      printf("b0 = %.3f\tb0 = %d\tb0 = %.3f\n", PD_b0, b0m_PD, PI_b0);
      printf("b1 = %.3f\tb1 = %d\tb1 = %.3f\n", PD_b1, b1m_PD, PI_b1);
      printf("a1 = %.3f\ta1 = %d\ta1 = %.3f\n\n", PD_a1, a1m_PD, PI_a1);
      printf("  Sampling time = %.1f[ms]\n", Ts/1e6);
    }


    else if(menu=='2'){
      printf("Set skid position\n");
      printf("Values from 0 to 10\n");
      fflush(stdout);
      scanf("%d",&x);
      fflush(stdin);
      pos = 3000 + (46*x);
      if(pos>3460){
        pos = 3460;
      }else if(pos<3000){
        pos = 3000;
      }
    }

    else if(menu=='3')
    {
      printf("Set belt speed in m/s\n");
      printf("Maximum speed 2.4 m/s\n");
      fflush(stdout);
      scanf("%lf",&speed);
      fflush(stdin);
      PI_ref = (speed*4.138029);
      if(PI_ref>9.95){
        PI_ref = 9.95;
      }else if(PI_ref<0){
        PI_ref = 0;
      }
    }

    else if(menu=='4')
    { 
      int total;
      total=small+medium+large+toosmall+toolarge;

      printf("************ STATISTICAL ASSESMENT *************\n\n"
      " Number of sticks per category                          \n"
      "                    ____                                \n"
      "            ____   |    |                               \n"
      "    ____   |    |  |    |                               \n"
      "   |    |  |    |  |    |                               \n"
      "   |    |  |    |  |    |                               \n"
      "     %d       %d       %d        %d          %d         \n"
      "   small   medium  large   too small  too large         \n\n"
      " TOTAL: *%d*                                            \n\n"
      " Select one of the options below                        \n"
      " 1: Reset parameters                                    \n"
      " 2: Return to main menu                                 \n",small,medium,large,toosmall,toolarge,total);

      fflush(stdout);
      scanf("%s",&menu2);
      fflush(stdin);
      if (menu2=='1'){
        toosmall=0;
        small=0;
        medium=0;
        large=0;
        toolarge=0;
        total=0;

        printf("************ STATISTICAL ASSESMENT *************\n\n"
        " Number of sticks per category                         \n"
        "                    ____                               \n"
        "            ____   |    |                              \n"
        "    ____   |    |  |    |                              \n"
        "   |    |  |    |  |    |                              \n"
        "   |    |  |    |  |    |                              \n"
        "     %d       %d       %d        %d          %d        \n"
        "   small   medium  large   too small  too large        \n\n"
        " TOTAL: *%d*                                           \n\n"
        " Select one of the options below                       \n"
        " 1: Reset parameters                                   \n"
        " 2: Return to main menu                                \n",small,medium,large,toosmall,toolarge,total);

        fflush(stdout);
        scanf("%s",&menu2);
        fflush(stdin);
      }
    }

    else if(menu=='5')
    {
      printf("*************** LENGTH CATEGORY ****************\n\n"
      " Current category:                                     \n"
      "   Small stick  = %.1f                                 \n"
      "   Medium stick = %.1f                                 \n"
      "   Large stick  = %.1f                                 \n\n"
      "                                                       \n"
      " Select one of the options below                       \n"
      " 1: Change values                                      \n"
      " 2: Return to main menu                                \n",small_ref,medium_ref,large_ref);

      fflush(stdout);
      scanf("%s",&menu3);
      fflush(stdin);

      if (menu3=='1'){
        printf("Set length of Small stick in cm\n");
          fflush(stdout);
          scanf("%lf",&small_ref);
          fflush(stdin);
          printf("\nSet length of Medium stick in cm\n");
          fflush(stdout);
          scanf("%lf",&medium_ref);
          fflush(stdin);
          printf("\nSet length of Large stick in cm\n");
          fflush(stdout);
          scanf("%lf",&large_ref);
          fflush(stdin);
      }
    }

    else if(menu=='6')
    {
      printf("Program closing.\n");
      stopflag=1;
    }

    else{
      printf("It is not a valid number, please enter one of available options\n");
    }
   
  }

  stopflag=1;

  // Join thread's
  pthread_join(skidThread,NULL);
  pthread_join(beltThread,NULL);
  pthread_join(sensorThread,NULL);
  pthread_join(timerThread,NULL);

  comedi_data_write(ADC, 1, 0, 0, AREF_GROUND, 2048);
  comedi_data_write(ADC, 1, 1, 0, AREF_GROUND, 2048);

  //delte Semaphore
  rt_sem_delete(my_sem);

  // Close comedi
  if(comedi_close(ADC) != 0){
    printf("Did not close ADC\n");
  }

  // Check for task delete.
  if(rt_task_delete(taskMain)==0){
    printf("Main task succesfully deleted\n");
    return 0;
  }
  else {
    printf("ERROR: taskMain not deleted!\n");
    return 0;
  }

}

/*********************************************** Belt *******************************************************/

void *Belt(void *arg){
  name_belt=nam2num("Belt");
  taskBelt=rt_task_init(name_belt, 0x00FF, 0, 0);
  rt_task_make_periodic_relative_ns(taskBelt,0,Ts);
  regul_init(&p_belt);


  // start of the control loop
  while(stopflag==0){
    comedi_data_read_delayed(ADC, 0, 1, 0, AREF_DIFF, &Belt_data_read, 50000); // read bits

    PI_data_vltg=Belt_data_read-2048;
    PI_data_vltg=PI_data_vltg/(4095/20);
    PI_data_vltg-=0.15; // remove offset

    PI_e=PI_ref-PI_data_vltg;

    regul_out(&p_belt,PI_e,PI_b0);

    PI_data_write=p_belt.u*(4095/20);
    PI_data_write=PI_data_write+2048;

    if(PI_data_write>4095){
      PI_data_write=4095;
    }
    if(PI_data_write<2048){
      PI_data_write=2048;
    }

    Belt_data_write=(unsigned int)PI_data_write;
    comedi_data_write(ADC, 1, 1, 0, AREF_GROUND, Belt_data_write);

    regul_update(&p_belt,PI_e,PI_b1,PI_a1);
    rt_task_wait_period();
  }

  comedi_data_write(ADC, 1, 1, 0, AREF_GROUND, 2048); // set speed to zero
  rt_task_delete(taskBelt);
  return 0;
} 


/********************************************** Skid ************************************************/
void *Skid(void *arg){
  name_skid=nam2num("Skid");
  taskSkid=rt_task_init(name_skid, 0x00FF, 0, 0);
  rt_task_make_periodic_relative_ns(taskSkid,0,Ts);

  fix_regul_init();

  short u_PD,ADinput_PD;
  
  while(stopflag==0){
    //read AD converter
    comedi_data_read_delayed(ADC,0,0,0,AREF_DIFF,&ADdata_PD,50000);
    
    //Bit to Volt
    ADinput_PD = (short) ADdata_PD;
    
    //controller:
    e_PD = sub16(pos,ADinput_PD);
    e_PD = e_PD>>2; 
    
    u_PD = fix_regul_out(e_PD, &b0m_PD);
    u_PD = u_PD<<2;
    u_PD += 2048.0;

    if(u_PD>4095){
      u_PD = 4095;
    }else if(u_PD<0){
      u_PD = 0;
    }

    //read DA converter
    DAdata_PD = (unsigned int) u_PD;
    comedi_data_write(ADC,1,0,0,AREF_DIFF,DAdata_PD);
    
    //update controller values
    fix_regul_update(&a1m_PD, &b1m_PD);
 
    rt_task_wait_period();
  }

  rt_task_delete(taskSkid);
  return 0;
}


/******************************************** Sensor ***************************************************/
void *Sensor(void *arg)
{
  //create task
  name_sensor=nam2num("Tsens");
  tasksensor=rt_task_init(name_sensor, 0x00FF, 0, 0);
  rt_task_make_periodic_relative_ns(tasksensor,0,Tss);

  //AD value when no block is present=2048, AD value when block is present 4098
  double sens_limit=3500;
  
  //sensorflag=0 when no block is present, =1 when block is detected
  int sensor_value=0;
  
  //parameters
  double start_time = 0;
  double stop_time = 0;
  double stime,qtime;
  double brick_length;

  while(stopflag==0)
  {
    comedi_data_read_delayed(ADC,0,2,0,AREF_GROUND,&ADdata_sens,50000);
    double AD_sens = (double) ADdata_sens;

    if(sensor_value==0)     //If there is no brick
    {  
      if(AD_sens>sens_limit){
        sens_start_time=rt_get_time_ns();   //brick start time
        sensor_value=1;                     //brick present flag
        start_time=(double) sens_start_time;//we transform it into double 
      }
    }
    else            //If there is a brick
    {
      if(AD_sens<sens_limit){ //If the brick is gone
        sens_stop_time=rt_get_time_ns();            //brick stop time
        stop_time=(double) sens_stop_time;          //we transform it into double
        stime=(stop_time-start_time)/1000000000.0;  //time brick was present
        brick_length=stime*speed*100.0;             //brick length in cm
        if (speed>1.5){
          brick_length-=(-(1.5-speed)*3.3)+0.25;
          brick_length+=(brick_length-(small_ref+large_ref)/2)*(-0.4);
        }     
        printf("Speed: %f\n", speed);
        printf("Time: %f\n", stime );
        printf("Length: %f\n", brick_length);

        sensor_value=0;   //mark that brick is gone 
  
        //semaphone mutual exclusion start
        rt_sem_wait(my_sem);
  
        //logic for choosing skid position
        if(brick_length<small_ref+1.25){
          if (brick_length<small_ref-1.5){
            toosmall++;
          }
          else {
            small++;
          }
          PD_ref_matrix[1][length]=3140;
          brick_length=small_ref;
        }
        else if(brick_length>=small_ref+1.25 && brick_length<=medium_ref+1.25)
        { 
          PD_ref_matrix[1][length]=3276;
          brick_length=medium_ref;
          medium++;
        }
        else
        {
          if (brick_length>large_ref+2){
            toolarge++;
          }
          else {
            large++;
          }
          PD_ref_matrix[1][length]=3400;
          brick_length=large_ref;
        }
       /* if(brick_length>7){
          qtime=(stop_time/1000000000)+((((43+6.75*speed)-brick_length)/100)/speed); //Time that tell us when the skid will have to move
        }
        if(brick_length<=7){
          qtime=(stop_time/1000000000)+((((43+speed)-brick_length)/100)/speed); //Time that tell us when the skid will have to move
        }*/
        qtime=(stop_time/1000000000)+((((43+7.25*speed)-brick_length)/100)/speed); //Time that tell us when the skid will have to move

        PD_ref_matrix[0][length]=qtime; 
        length++; 

        //semaphone mutual exclusion end
        rt_sem_signal(my_sem);
      }
    }         

    rt_task_wait_period();
  }

  rt_task_delete(tasksensor);
  return 0;
}



/********************************************** Timer **************************************************/
void *Timer(void *arg)
{
  name_timer=nam2num("Ttime");
  tasktimer=rt_task_init(name_timer, 0x00FF, 0, 0);
  rt_task_make_periodic_relative_ns(tasktimer,0,Ts);
  
  double timer_time;
  
  while(stopflag==0)
  {
    currenttime=rt_get_time_ns();   //get the time
    timer_time=(double)currenttime;   //transform it into double 
    timer_time=timer_time/1000000000; 
    
    
    //semaphone mutual exclusion start
    rt_sem_wait(my_sem);

    if(PD_ref_matrix[0][0]!=0 && PD_ref_matrix[0][0]<=timer_time)
    {
      pos=PD_ref_matrix[1][0]; //We assign the reference for the skid
       
      //move values of matrix down
      for(j=0;j<length;j++)
      {
        PD_ref_matrix[0][j]=PD_ref_matrix[0][j+1];
        PD_ref_matrix[1][j]=PD_ref_matrix[1][j+1];
      }

      PD_ref_matrix[0][length]=0;
      PD_ref_matrix[1][length]=0;
      
      length--;
    }
 
    //semaphore mutual exclusion end
    rt_sem_signal(my_sem);
    
    rt_task_wait_period();
  } 

  rt_task_delete(tasktimer);
  return 0;
}


