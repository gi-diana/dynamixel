#include "mbed.h"
#include "communication_1.h"
#include "MX106.h"
#include "AX12.h"
#include "Servo.h"
#define SPEED 100
// Utility
InterruptIn button(USER_BUTTON);
DigitalOut led(LED1);

// Motor Control
//Serial pc(USBTX, USBRX);
communication_1 wire(PA_9, PA_10, 57600);
MX106 motor_1(wire, 1, 1);
MX106 motor_2(wire, 2, 1);
MX106 motor_3(wire, 3, 1);
AX12  motor_4(wire, 4, 1);
Servo cam1 (D9);
Servo cam2 (D10);
void button_int_handler()
{

}

// CAN
//Thread canrxa;
//Mutex mutex;
CAN can1(PA_11, PA_12);     // RX, TX

CANMessage messageIn;
CANMessage messageOut;

int filter = can1.filter(0x000, 0x400, CANStandard);
int pose=1;
int current_pose[6];
void canrx()
{
    if(can1.read(messageIn, filter))
    {
      pose=messageIn.data[0] + (messageIn.data[1] << 8) + (messageIn.data[2] << 16) + (messageIn.data[3] << 24);
      printf("CAN: mess %d\n\r", pose);
      printf("CAN: id %x \n\r ",messageIn.id);
      
      if((messageIn.id & 0x0FF) == 0x40)
      {
            //mutex.lock();
            current_pose[0]=pose;
            //mutex.unlock();
      }
      else if((messageIn.id & 0x0FF) == 0x50)
      {
           //mutex.lock();
            current_pose[1]=pose;
            //mutex.unlock();
       }
       
      else if((messageIn.id & 0x0FF) == 0x60)
      {
             //mutex.lock();
             current_pose[2]=pose;
             //mutex.unlock();
      }
      else if((messageIn.id & 0x0FF) == 0x70)
      {
             //mutex.lock();
             current_pose[3]=pose;
            //mutex.unlock();
      }
        else if((messageIn.id & 0x0FF) == 0x80)
        {
            //mutex.lock();
            current_pose[4]=pose;
            //mutex.unlock();
        }
         else if((messageIn.id & 0x0FF) == 0x90)
        {
            //mutex.lock();
             current_pose[5]=pose;
             //mutex.unlock();     
        }
    }
  
}
   
int main()
{
printf("DYNAMIXEL: Init \n\r");
led=1; 
   wire.trigger();
   wire.trigger();
   wire.trigger();
   wire.trigger();
   wait(5);
   // Setup Motor1 MultiTurn
   motor_1.setMotorEnabled(1);
   motor_1.setMode(0);
   //motor_1.setSpeed(0);
   //printf("Dynamixel 1 Position init: %f \n\r ", motor_1.getPosition());
  
   wait(5);
   printf("DYNAMIXEL: Init DONE 1\n\r");
   // Setup Motor2 MultiTurn
   motor_2.setMotorEnabled(1);
   motor_2.setMode(0);
   //motor_2.setSpeed(0);
   //printf("Dynamixel 2 Position init: %f \n\r ", motor_2.getPosition());
   
   wait(5);
   printf("DYNAMIXEL: Init DONE 2\n\r");
   // Setup Motor3 MultiTurn
   motor_3.setMotorEnabled(1);
   motor_3.setMode(0);
   //motor_3.setSpeed(0);
   //printf("Dynamixel 3 Position init: %f \n\r ", motor_3.getPosition());
   wait(5);
   printf("DYNAMIXEL: Init DONE 3\n\r");
   //motor_3.setGoalPosition(0);
   // Setup Motor4 MultiTurn
   //motor_4.setMotorEnabled(1);
   //motor_4.setMode(0);
   //motor_4.setSpeed(0);
   //printf("Dynamixel 4 Position init: %f \n\r ", motor_4.getPosition());
   wait(5);
    
   printf("DYNAMIXEL: Init DONE\n\r");
   
   button.rise(&button_int_handler);
  cam1=0,5;
  cam2=0,5;
   // CAN Initialization  
   //canrxa.start(canrx);

   printf("DONE: CAN Init\n\r");
  
  
   printf("Running!\n\r");
  
   while(true)
   {
      
            canrx();
            if (current_pose[0]==0)
            {
                motor_1.setSpeed(-SPEED);
                printf("Dynamixel 1 Position : %f \n\r ", motor_1.getPosition());
            }
            
            else if (current_pose[0]==50)
            {
                motor_1.setSpeed(0);
                printf("Dynamixel 1 Position : %f \n\r ", motor_1.getPosition());
            }
            else if (current_pose[0]==100)
            {
               
                motor_1.setSpeed(SPEED);
                printf("Dynamixel 1 Position : %f \n\r ", motor_1.getPosition());
                
            }
            
            else if (current_pose[1]==0)
            {
                motor_2.setSpeed(-SPEED);
                printf("Dynamixel 2 Position : %f \n\r ", motor_2.getPosition());
                if (motor_2.getPosition()<-90) motor_2.setSpeed(0);
            }
            
            else if (current_pose[1]==50)
            {
                motor_2.setSpeed(0);
                printf("Dynamixel 2 Position : %f \n\r ", motor_2.getPosition());
            }
            else if (current_pose[1]==100)
            {
               
                motor_2.setSpeed(SPEED);
                printf("Dynamixel 2 Position : %f \n\r ", motor_2.getPosition());
                if (motor_2.getPosition()>90) motor_2.setSpeed(0);
            }
            
             else if (current_pose[2]==0)
            {
                motor_3.setSpeed(-SPEED);
                printf("Dynamixel 3 Position : %f \n\r ", motor_3.getPosition());
            }
            
            else if (current_pose[2]==50)
            {
                motor_3.setSpeed(0);
                printf("Dynamixel 3 Position : %f \n\r ", motor_3.getPosition());
            }
            else if (current_pose[2]==100)
            {
               
                motor_3.setSpeed(SPEED);
                printf("Dynamixel 3 Position : %f \n\r ", motor_3.getPosition());
                
            }
            
            else if (current_pose[3]==0)
            {
                motor_4.setSpeed(-SPEED);
                printf("Dynamixel 4 Position : %f \n\r ", motor_4.getPosition());
            }
            
            else if (current_pose[3]==50)
            {
                motor_4.setSpeed(0);
                printf("Dynamixel 4 Position : %f \n\r ", motor_1.getPosition());
            }
            else if (current_pose[3]==100)
            {
               
                motor_4.setSpeed(SPEED);
                printf("Dynamixel 4 Position : %f \n\r ", motor_4.getPosition());    
            }
            
            else if (current_pose[4]==0)
            {
                if (cam1==0)
                    cam1=cam1;
                else cam1 = cam1-0.2;
                
                wait(0.2);
                
            }
            else if (current_pose[4]==100)
            {
            
                if (cam1==1)
                    cam1=cam1;
                else cam1 = cam1+0.2;
                
                wait(0.2);
            }
             else if (current_pose[5]==0)
            {
                if (cam2==0)
                    cam1=cam1;
                else cam2 = cam2-0.2;
                
                wait(0.2);
                
            }
            else if (current_pose[5]==100)
            {
            
                if (cam2==1)
                    cam2=cam2;
                else cam2 = cam2+0.2;
                
                wait(0.2);
            }
             
                        
   }
}
