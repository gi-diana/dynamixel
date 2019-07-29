#include "mbed.h"
#include "communication_1.h"
#include "MX106.h"
#include "AX12.h"
#include <stdio.h>
#include <string.h>


#define VERBOSE 1

#define CAN_FREQUENCY 125000
#define BAUDRATE  57600
#define SLEEP 0.02
#define JOINT_SET_POSITION 20
#define JOINT_STATUS 30
#define JOINT_ERROR 40
#define JOINT1_ID 4
#define JOINT2_ID 5
#define JOINT3_ID 6
#define JOINT4_ID 7


int dxl_id[]={1,2,3,4};
int joint_id[]={4,5,6,7};
int dxl_speed[]={50,25,50,50};
float dxl_gear[]={3,2,3,1};
float dxl_present_position[] = {0,0,0,0};
float dxl_goal_position[] = {0,0,0,0};
float dxl_offset[] = {0,0,0,0};
float dxl_current[] = {0,0,0,0};
float dxl_torque[] = {0,0,0,0};
float dxl_max_torque[] = {0,0,0,0};
float dxl_reset[]={0,0,0,0};
uint16_t dxl_temperature[]={0,0,0,0};
uint16_t dxl_limit[]={0,0,0,0};
int present, current, torque, temperature;
int status;

DigitalOut led(LED1);

communication_1 wire(PA_9, PA_10, BAUDRATE);

MX106 w_1(wire, dxl_id[0], dxl_gear[0]);
MX106 w_2(wire, dxl_id[1], dxl_gear[1]);
MX106 w_3(wire, dxl_id[2], dxl_gear[2]);
AX12  a_1(wire, dxl_id[3], dxl_gear[3]);

CAN can1(PB_5, PB_6);     // RX, TX

CANMessage messageIn;
CANMessage messageOut;



int main ()
{
  #if VERBOSE
  printf("START \n\r");
  #endif
  can1.frequency(CAN_FREQUENCY);
  messageIn.format=CANExtended;
  messageOut.format=CANExtended;

  wire.trigger();
  wire.trigger();
  wire.trigger();
  wire.trigger();

  #if VERBOSE
  printf("DYNAMIXEL: Init START \n\r");
  #endif
  w_1.setMotorEnabled(1);
  wait(SLEEP);
  #if VERBOSE
  printf("DYNAMIXEL: Init 1 \n\r");
  #endif
  w_2.setMotorEnabled(1);
  wait(SLEEP);
  #if VERBOSE
  printf("DYNAMIXEL: Init 2 \n\r");
  #endif
  w_3.setMotorEnabled(1);
  wait(SLEEP);
  #if VERBOSE
  printf("DYNAMIXEL: Init 3\n\r");
   #endif
  a_1.setMotorEnabled(1);
  wait(SLEEP);
  #if VERBOSE
  printf("DYNAMIXEL: Init 4\n\r");
  #endif

  w_1.setMode(2);
  wait(SLEEP);
  w_1.setMaxSpeed(dxl_speed[0]);
  wait(SLEEP);
  #if VERBOSE
  printf("DYNAMIXEL 1: SET MODE\n\r");
  #endif
  w_2.setMode(2);
  wait(SLEEP);
  w_2.setMaxSpeed(dxl_speed[1]);
  wait(SLEEP);
  w_2.setCWLimitUnits(1900);
  wait(SLEEP);
  w_2.setCCWLimitUnits(-1900);
  wait(SLEEP);
  #if VERBOSE
  printf("DYNAMIXEL 2: SET MODE\n\r");
  #endif
  w_3.setMode(2);
  wait(SLEEP);
  w_3.setMaxSpeed(dxl_speed[2]);
  wait(SLEEP);
  #if VERBOSE
  printf("DYNAMIXEL 3: SET MODE\n\r");
  #endif
  a_1.setMode(1);
  wait(SLEEP);
  a_1.setMaxSpeed(dxl_speed[3]);
  wait(SLEEP);
  #if VERBOSE
  printf("DYNAMIXEL 4: SET MODE\n\r");
  #endif

  w_1.setGoal((dxl_goal_position[0]-dxl_offset[0]));
  wait(SLEEP);
  w_2.setGoal((dxl_goal_position[1]-dxl_offset[1]));
  wait(SLEEP);
  w_3.setGoal((dxl_goal_position[2]-dxl_offset[2]));
  wait(SLEEP);
  a_1.setGoal((dxl_goal_position[3]-dxl_offset[3]));
  wait(SLEEP);

  wait(1);

  while (1)
  {
    if(can1.read(messageIn))
     {
       #if VERBOSE
       printf("CAN: Message passed!\tId: %d\n\r", messageIn.id);
       #endif
       led!=led;
       if(messageIn.id == ((JOINT_SET_POSITION << 8) + JOINT1_ID))
        {
           dxl_goal_position[0] = (messageIn.data[0] << 24) | (messageIn.data[1] << 16) | (messageIn.data[2] << 8) | (messageIn.data[3]);
        }
        if(messageIn.id == ((JOINT_SET_POSITION << 8) + JOINT2_ID))
         {
            dxl_goal_position[1] = (messageIn.data[0] << 24) | (messageIn.data[1] << 16) | (messageIn.data[2] << 8) | (messageIn.data[3]);
         }
        if(messageIn.id == ((JOINT_SET_POSITION << 8) + JOINT3_ID))
         {
             dxl_goal_position[2] = (messageIn.data[0] << 24) | (messageIn.data[1] << 16) | (messageIn.data[2] << 8) | (messageIn.data[3]);
         }
        if(messageIn.id == ((JOINT_SET_POSITION << 8) + JOINT4_ID))
         {
             dxl_goal_position[3] = (messageIn.data[0] << 24) | (messageIn.data[1] << 16) | (messageIn.data[2] << 8) | (messageIn.data[3]);
         }

         if(messageIn.id == ((JOINT_ERROR << 8) + JOINT1_ID))
          {
              dxl_reset[0] = ((messageIn.data[2] << 8) + (messageIn.data[3]));
          }
          if(messageIn.id == ((JOINT_ERROR << 8) + JOINT2_ID))
           {
               dxl_reset[1] = ((messageIn.data[2] << 8) + (messageIn.data[3]));
           }
           if(messageIn.id == ((JOINT_ERROR << 8) + JOINT3_ID))
            {
                dxl_reset[2] = ((messageIn.data[2] << 8) + (messageIn.data[3]));
            }
            if(messageIn.id == ((JOINT_ERROR << 8) + JOINT4_ID))
             {
                 dxl_reset[3] = ((messageIn.data[2] << 8) + (messageIn.data[3]));
             }
        }

          w_1.setGoal((dxl_goal_position[0]-dxl_offset[0]));
          wait(SLEEP);
          w_2.setGoal((dxl_goal_position[1]-dxl_offset[1]));
          wait(SLEEP);
          w_3.setGoal((dxl_goal_position[2]-dxl_offset[2]));
          wait(SLEEP);
          a_1.setGoal((dxl_goal_position[3]-dxl_offset[3]));
          wait(SLEEP);

          dxl_present_position[0]=w_1.getPosition();
          wait(SLEEP);
          dxl_present_position[1]=w_2.getPosition();
          wait(SLEEP);
          dxl_present_position[2]=w_3.getPosition();
          wait(SLEEP);
          dxl_present_position[3]=a_1.getPosition();
          wait(SLEEP);

          #if VERBOSE
          printf("DYNAMIXEL PRESENT POSITION %f %f %f %f\n\r", dxl_present_position[0],dxl_present_position[1], dxl_present_position[2] ,dxl_present_position[3] );
          #endif

          dxl_current[0]=w_1.getCurrent();
          wait(SLEEP);
          dxl_current[1]=w_2.getCurrent();
          wait(SLEEP);
          dxl_current[2]=w_3.getCurrent();
          wait(SLEEP);
          //dxl_current[3]=a_1.getCurrent();
          //wait(SLEEP);
          #if VERBOSE
          printf("DYNAMIXEL CURRENT %f %f %f %f\n\r", dxl_current[0],dxl_current[1], dxl_current[2] ,dxl_current[3] );
           #endif

          dxl_torque[0]=dxl_current[0]*0.875;
          dxl_torque[1]=dxl_current[1]*0.875;
          dxl_torque[2]=dxl_current[2]*0.875;
          //dxl_torque[3]=dxl_current[3]*0.875;
          #if VERBOSE
          printf("DYNAMIXEL TORQUE %f %f %f %f\n\r", dxl_torque[0],dxl_torque[1], dxl_torque[2] ,dxl_torque[3] );
          #endif

          dxl_temperature[0]=w_1.getTemp();
          wait(SLEEP);
          dxl_temperature[1]=w_2.getTemp();
          wait(SLEEP);
          dxl_temperature[2]=w_3.getTemp();
          wait(SLEEP);
          dxl_temperature[3]=a_1.getTemp();
          wait(SLEEP);

          #if VERBOSE
          printf("DYNAMIXEL TEMPERATURE %d %d %d %d\n\r", dxl_temperature[0],dxl_temperature[1], dxl_temperature[2] ,dxl_temperature[3] );
          #endif

          for(int i=0; i<( sizeof(dxl_id) / sizeof(dxl_id[0])); i++)
          {
            messageOut.len = 8;
            present=(int)(dxl_present_position[i]*100);
            current=(int)(dxl_current[i]*100);
            torque=(int)(dxl_torque[i]*100);
            temperature=(int)(dxl_temperature[i]*100);

            messageOut.id = (JOINT_STATUS << 8) + joint_id[i];

            messageOut.data[0] = (present >> 8);
            messageOut.data[1] = (present);
            messageOut.data[2] = (current >> 8);
            messageOut.data[3] = (current);
            messageOut.data[4] = (torque >> 8);
            messageOut.data[5] = (torque);
            messageOut.data[6] = (temperature >> 8);
            messageOut.data[7] = (temperature);

            status = can1.write(messageOut);

            #if VERBOSE
            printf("CAN send status Joint %d : %d", joint_id[i] , status );
            #endif
          }



          dxl_max_torque[0]=w_1.getTorqueMax();
          wait(SLEEP);
          dxl_max_torque[1]=w_2.getTorqueMax();
          wait(SLEEP);
          dxl_max_torque[2]=w_3.getTorqueMax();
          wait(SLEEP);
          dxl_max_torque[3]=a_1.getTorqueMax();
          wait(SLEEP);

          for (int i=0; i<( sizeof(dxl_id) / sizeof(dxl_id[0])); i++ )
          {

            messageOut.len = 4;
            messageOut.id= ( JOINT_ERROR << 8 ) + joint_id[i];

            messageOut.data[0]=(((int)(dxl_max_torque[i]))>>8);
            messageOut.data[1]= ((int)(dxl_max_torque[i]));
            status = can1.write(messageOut);

            #if VERBOSE
            printf("CAN send status ERROR Joint %d : %d", joint_id[i] , status );
             #endif

          }
          if(dxl_reset[0]!=0)
          {
            w_1.setMotorEnabled(0);
            wait(SLEEP);
            w_1.setMaxTorque(1);
            wait(SLEEP);
            w_1.setMotorEnabled(1);
            wait(SLEEP);
            w_1.setGoal(0-dxl_offset[0]);
            wait(SLEEP);
            #if VERBOSE
            printf("Dynamixel %d : RESET", joint_id[0] );
             #endif
          }
          if(dxl_reset[1]!=0)
          {
            w_2.setMotorEnabled(0);
            wait(SLEEP);
            w_2.setMaxTorque(1);
            wait(SLEEP);
            w_2.setMotorEnabled(1);
            wait(SLEEP);
            w_2.setGoal(0-dxl_offset[1]);
            wait(SLEEP);
            #if VERBOSE
            printf("Dynamixel %d : RESET", joint_id[1] );
             #endif
          }
          if(dxl_reset[2]!=0)
          {
            w_3.setMotorEnabled(0);
            wait(SLEEP);
            w_3.setMaxTorque(1);
            wait(SLEEP);
            w_3.setMotorEnabled(1);
            wait(SLEEP);
            w_3.setGoal(0-dxl_offset[2]);
            wait(SLEEP);
            #if VERBOSE
            printf("Dynamixel %d : RESET", joint_id[2] );
             #endif
          }
          if(dxl_reset[3]!=0)
          {
            a_1.setMotorEnabled(0);
            wait(SLEEP);
            a_1.setMaxTorque(1);
            wait(SLEEP);
            a_1.setMotorEnabled(1);
            wait(SLEEP);
            a_1.setGoal(0-dxl_offset[3]);
            wait(SLEEP);
            #if VERBOSE
            printf("Dynamixel %d : RESET", joint_id[3] );
             #endif
          }


      }
      return 0;
}
