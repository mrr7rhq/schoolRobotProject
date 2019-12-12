/**
* @mainpage ZumoBot Project
* @brief    You can make your own ZumoBot with various sensors.
* @details  <br><br>
    <p>
    <B>General</B><br>
    You will use Pololu Zumo Shields for your robot project with CY8CKIT-059(PSoC 5LP) from Cypress semiconductor.This 
    library has basic methods of various sensors and communications so that you can make what you want with them. <br> 
    <br><br>
    </p>
    
    <p>
    <B>Sensors</B><br>
    &nbsp;Included: <br>
        &nbsp;&nbsp;&nbsp;&nbsp;LSM303D: Accelerometer & Magnetometer<br>
        &nbsp;&nbsp;&nbsp;&nbsp;L3GD20H: Gyroscope<br>
        &nbsp;&nbsp;&nbsp;&nbsp;Reflectance sensor<br>
        &nbsp;&nbsp;&nbsp;&nbsp;Motors
    &nbsp;Wii nunchuck<br>
    &nbsp;TSOP-2236: IR Receiver<br>
    &nbsp;HC-SR04: Ultrasonic sensor<br>
    &nbsp;APDS-9301: Ambient light sensor<br>
    &nbsp;IR LED <br><br><br>
    </p>
    
    <p>
    <B>Communication</B><br>
    I2C, UART, Serial<br>
    </p>
*/

#include <project.h>
#include <stdlib.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "LSM303D.h"
#include "IR.h"
#include "Beep.h"
#include "mqtt_sender.h"
#include <time.h>
#include <sys/time.h>
#include "serial1.h"
#include <unistd.h>


/**
 * @file    main.c
 * @brief   
 * @details  ** Enable global interrupt since Zumo library uses interrupts. **<br>&nbsp;&nbsp;&nbsp;CyGlobalIntEnable;<br>
*/
    int findStatus(struct sensors_ dig){                                               
                int st=-1;
                if(dig.l1==1 && dig.l2==1 && dig.l3==1 && dig.r1==1 && dig.r2==1 && dig.r3==1){ //dark line return 0;
                              st=0;
                }
                else if(dig.l3==0 && dig.l2==0   && dig.l1==1 && dig.r1==1 && dig.r2==1 &&dig.r3==1 ){
                             st=3;
                        }
                else if(dig.l3==1 && dig.l2==1 && dig.l1==1 && dig.r1==1 &&dig.r2==0 && dig.r3==0 ){
                            st= 2;
                }else{
                            st=1;
                        }
                return st;
                /*
                How to use this to find line
                   
                int count=0;//register how mant time dark and light changes. one line has two changes since there are two edges 
                
                in while loop
                    reflectance_digital(&dig);
                    temp=findStatus(dig);//get new status
                    motor_forward(50,2); 
                    if(temp!=status){
                        count++;
                        
                    }
                    status=temp;//remember the current status
                
                */
}
    
   
    void motor_hardRight(uint8 speed,uint32 delay)//on point(tank) turn right 
{
    MotorDirLeft_Write(0);      // set LeftMotor forward mode
    MotorDirRight_Write(1);     // set RightMotor forward mode
    PWM_WriteCompare1(speed); 
    PWM_WriteCompare2(speed); 
    vTaskDelay(delay);
}

     void motor_hardLeft(uint8 speed,uint32 delay)//on point (tank) turn left
{
    MotorDirLeft_Write(1);      // set LeftMotor forward mode
    MotorDirRight_Write(0);     // set RightMotor forward mode
    PWM_WriteCompare1(speed); 
    PWM_WriteCompare2(speed); 
    vTaskDelay(delay);
}
struct sensors_ findThres(){                                                // find threhold by itself
    struct sensors_ raw, threshold;
        uint32_t maxl1=0,maxl2=0,maxl3=0,maxr1=0,maxr2=0,maxr3=0, minl1=0,minl2=0,minl3=0,minr1=0,minr2=0,minr3=0;
        motor_start();
        motor_forward(0,0);
        
       
        vTaskDelay(500);
        for(int i =0; i<500; i++){//white first
            
            reflectance_read(&raw);
             motor_hardLeft(150,4);
            if(i==0){
                maxl1=raw.l1;
                maxl2=raw.l2;
                maxl3=raw.l3;
                maxr1=raw.r1;
                maxr2=raw.r2;
                maxr3=raw.r3;
                
                minl1=raw.l1;
                minl2=raw.l2;
                minl3=raw.l3;
                minr1=raw.r1;
                minr2=raw.r2;
                minr3=raw.r3;
            }// init max and min value with the first reading 
            else if(raw.l1>maxl1){
                maxl1=raw.l1;
            }else if(raw.l1<minl1){
                minl1=raw.l1;
            }//find out the max or min value for L1 
            if(raw.l2>maxl2){
                maxl2=raw.l2;
            }else if(raw.l2<minl2){
                minl2=raw.l2;
            }//for L2
            if(raw.l3>maxl3){
                maxl3=raw.l3;
            }else if(raw.l3<minl3){
                minl3=raw.l3;
            }//for L3
            if(raw.r1>maxr1){
                maxl1=raw.l1;
            }else if(raw.r1<minr1){
                minr1=raw.r1;
            }// find the min value for r1
            if(raw.r2>maxr2){
                maxr2=raw.r2;
            }else if(raw.r2<minr2){
                minr2=raw.r2;
            }
            if(raw.r3>maxr3){
                maxr3=raw.r3;
            }else if(raw.r3<minr3){
                minr3=raw.r3;
            }
            
             
        }
        motor_forward(0,0);
        /*threshold.l1=(maxl1+minl1)/2;
        threshold.l2=(maxl2+minl2)/2;
        threshold.l3=(maxl3+minl3)/2;
        threshold.r1=(maxr1+minr1)/2;
        threshold.r2=(maxr2+minr2)/2;
        threshold.r3=(maxr3+minr3)/2;*/
        threshold.l1=minl1+0.25*(maxl1-minl1);
        threshold.l2=minl2+0.25*(maxl2-minl2);
        threshold.l3=minl3+0.25*(maxl3-minl3);
        threshold.r1=minr1+0.25*(maxr1-minr1);
        threshold.r2=minr2+0.25*(maxr2-minr2);
        threshold.r3=minr3+0.25*(maxr3-minr3);
        Beep(100,255);
        return threshold;
        /*
        how to using this function to set threshold 
        reflectance_start();
        struct sensors_ threshold,dig;
        threshold=findThres();
        
        reflectance_set_threshold(threshold.l3,threshold.l2,threshold.l1,threshold.r1,threshold.r2,threshold.r3);
        */
}
   int lineStatus(struct sensors_ dig){                    //check the reflection sensor situation 
    if(dig.l1==1 && dig.r1==1){
        
        return 3;// go straght
    }if(dig.r1==1 && dig.r2==1 &&dig.r3!=1){
        return 24; //go little right
    }if(dig.r2==1 && dig.r3==1 && dig.l3!=1){
        return 25; //go hard right
    }if(dig.l1==1 && dig.l2==1 && dig.l3!=1){
        return 14; //go little left
    }if(dig.l2==1 && dig.l3==1 && dig.r3!=1){
        return 15; // go hard left
    }
    return -1;
}
 struct accData_ findAccStatus(){      //to normalize acceration seneor readings
    struct accData_ stable;
    int32 maxx, minx, maxy,miny;
    for(int i=0;i<1000; i++){
        LSM303D_Read_Acc(&stable);
        if(i==0){
            maxx=stable.accX;
            minx=stable.accX;
            maxy=stable.accY;
            miny=stable.accY;
        }
        if(stable.accX>maxx){
            maxx=stable.accX;
        }if(stable.accX<minx){
            minx=stable.accX;
        }
        if(stable.accY>maxy){
            maxy=stable.accY;
        }if(stable.accY<miny){
            miny=stable.accY;
        }
    }
    stable.accX=(maxx+minx)/2;
    stable.accY=(maxy+miny)/2;
    return stable;
}
int findSmallStatus(int senser,struct sensors_ dig){  //to check reading changes of selected reflection seneor
                int st=-1;
              if(senser==1){// 1 for l3
                if(dig.l3==1){ //dark line return 0;
                              st=0;
                }
                else{
                            st=1;
                        }
            }else if(senser==2){// 2 for l2
                if(dig.l2==1){ //dark line return 0;
                              st=0;
                }
                else{
                            st=1;
                        }
            }else if(senser==3){// 3 for l1
                if(dig.l1==1){ //dark line return 0;
                              st=0;
                }
                else{
                            st=1;
                        }
            }else if(senser==4){//4 for r1
                if(dig.r1==1){ //dark line return 0;
                              st=0;
                }
                else{
                            st=1;
                        }
            }else if(senser==5){//5 doe r2
                if(dig.r2==1){ //dark line return 0;
                              st=0;
                }
                else{
                            st=1;
                        }
            }if(senser==6){//6 for r3
                if(dig.r3==1){ //dark line return 0;
                              st=0;
                }
                else{
                            st=1;
                        }
            }
                return st;
                /*
                How to use this to find line
                   
                int count=0;//register how mant time dark and light changes. one line has two changes since there are two edges 
                int temp status;
                status=findSmallStatus(seneor, dig);
                in while loop
                    reflectance_digital(&dig);
                    temp=findStatus(dig);//get new status
                    motor_forward(50,2); 
                    if(temp!=status){
                        count++;
                        
                    }
                    status=temp;//remember the current status
                
                */
}
void turn90(int option, int xlineNumb, struct sensors_ dig){
   //two parameter 1  option value control turn left(1) or right(2) 
    //2 dig is the reflection sensor digtal data structure                    to make a nice 90 degree turn using reflection sensors and to your selected direction  
    
    if(option==1){//left
        if(xlineNumb!=6){
            while((dig.l1==1 && dig.r1==1) || (dig.l1==1 && dig.r1==0) ||(dig.l1==0 && dig.r1==1)){  //I know the conditions are the same. at first I thought it should not be the same but after 
                                                                                                        //I do not want to change the code since I break the code so many time because of this kind of change 
                motor_hardLeft(100,0);
                reflectance_digital(&dig);
            }
        }else{
            while((dig.l1==1 && dig.r1==1) || (dig.l1==1 && dig.r1==0) ||(dig.l1==0 && dig.r1==1)){
                motor_hardLeft(100,0);
                reflectance_digital(&dig);
            }
        }
        while(dig.l1!=1 && dig.r1!=1){
           motor_hardLeft(100,0);
            reflectance_digital(&dig);
            
        }
        motor_forward(0,0);
    
                
      
            
        
        
         
      
    }else if(option==2){//right
       
       
        if(xlineNumb!=0){
            while((dig.l1==1 && dig.r1==1) || (dig.l1==1 && dig.r1==0) ||(dig.l1==0 && dig.r1==1)){
                motor_hardRight(100,0);
                reflectance_digital(&dig);
            }
        }else{
            while((dig.l1==1 && dig.r1==1) || (dig.l1==1 && dig.r1==0) ||(dig.l1==0 && dig.r1==1)){
                motor_hardRight(100,0);
                reflectance_digital(&dig);
            }
        }
        while(dig.l1!=1 && dig.r1!=1){
           motor_hardRight(100,0);
            reflectance_digital(&dig);
            
        }
        motor_forward(0,0);
            
        
        
         
        
    }
}

void lturn(int direction, struct sensors_ dig){
    int temp,status, n=0;
    
    if(direction==1){//left
        
            
     status=findSmallStatus(3,dig);
        while(true){
            
           
            reflectance_digital(&dig);
            temp=findSmallStatus(3,dig);
            if(temp!=status){
                n++;
                
                if(n==3){
                    
                    break;
                }
                status=temp;
            }
           // motor_hardLeft(120,0);
            motor_turn(0,110,0);
        }
        
    }else if(direction==2){
        motor_forward(50,100);
        status=findSmallStatus(3,dig);
        while(true){
            reflectance_digital(&dig);
            temp=findSmallStatus(3,dig);
            if(temp!=status){
                n++;
                
                if(n==2){
                    
                    break;
                }
                status=temp;
            }
            motor_hardRight(100,0);
        }
    }else if(direction==3){       //only using this to make a 180 degree turn
        status=findSmallStatus(4,dig);
        while(true){
            reflectance_digital(&dig);
            temp=findSmallStatus(4,dig);
            if(temp!=status){
                n++;
                
                if(n==5){
                    motor_forward(0,0);
                    
                    break;
                }
                status=temp;
            }
            motor_hardRight(150,0);
        }
    }
}

void gostrat(int speed,struct sensors_ dig){ // following the line
                    reflectance_digital(&dig);
                     
                    if(dig.l1==1 && dig.r1==1 && findStatus(dig)!=0){
                        motor_forward(speed,0);
                    }else
                    if(dig.l1==1 && dig.r2!=1 && findStatus(dig)!=0){
                        motor_turn(0,100,0);
                    }else if(dig.l1!=1 && dig.r2==1 && findStatus(dig)!=0){
                        motor_turn(100,0,0);
                    }else if(dig.l1==0 && dig.r1==1 && dig.r2==1){
                        motor_turn(150,0,0);
                          
                        
                    }else if(dig.l1==1 && dig.l2==1 && dig.r1==0 ){
                        motor_turn(0,150,0);
                        
                        
                    }else if(dig.l3==1 && dig.l2==1 && findStatus(dig)!=0){
                        motor_turn(0,100,0);
                    }else if(dig.r3==1 && dig.r2==1 && findStatus(dig)!=0){
                        motor_turn(100,0,0);
                    }else if(dig.l3==1 && dig.l2==1 && dig.l1==1 && dig.r2==0&& dig.r1==0 &&dig.r3==0 ){
                         motor_turn(0,100,0);
                    }else if(dig.l3==0 && dig.l2==0 && dig.l1==0 && dig.r1==1 && dig.r2==1 &&dig.r3==1 ){
                         motor_turn(100,0,0);
                    }else if(dig.l3==1 && dig.l2==0 && dig.l1==0 && dig.r1==0 && dig.r2==0 &&dig.r3==0 ){
                         motor_turn(200,0,0);
                    }else if(dig.l3==0 && dig.l2==0 && dig.l1==0 && dig.r1==0 && dig.r2==0 &&dig.r3==1 ){
                         motor_turn(0,200,0);
                    }
                   
                    
    
}

int findEdge(struct sensors_ dig){
    int p=-1;
    reflectance_digital(&dig);
    if(dig.l3==0  && dig.l1==1 && dig.r1==1 && dig.r2==1 &&dig.r3==1 ){
        p=0;
    }
    else if(dig.l3==1 && dig.l2==1 && dig.l1==1 && dig.r1==1 &&dig.r3==0 ){
        p= 6;
    }
    return p;
}


void initMap(int map[14][7]){       //set map to init status
    for(int i=0;i<14;i++){
            for(int p=0;p<7;p++){
                map[i][p]=0;
            }
        }
        
        
        map[0][0]=1;
        map[0][1]=1;
        map[1][0]=1;
        map[0][5]=1;
        map[0][6]=1;
        map[1][6]=1;
}

void getLine(int speed){                            //stop at a line
  
    struct sensors_ dig;
    while(1){
   
        reflectance_digital(&dig);
        motor_forward(speed,2);
        if(findStatus(dig)==0){                     //meet a line 
            while(findStatus(dig)==0){              //get out the line 
                reflectance_digital(&dig);
                motor_forward(speed,2);
            }
            break;
        } 
    }
    motor_forward(0,0);
}






#if 1 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////777
    void zmain(){
        TickType_t start, end;
        start=end=xTaskGetTickCount();
        
        
        //get reflectance sensor ready
        //get reflectance sensor ready
        reflectance_start();
        struct sensors_ dig, thres;
        //set threshold
        //thres=findThres();
        //reflectance_set_threshold(thres.l3,thres.l2,thres.l1,thres.r1,thres.r2,thres.r3);
        reflectance_set_threshold(11038, 10406, 9000, 9462, 9657, 10473);
        IR_Start();
        IR_flush();
        //get Ultra ready
       
        motor_start();
        
        
        int speed=50;
        int mindis=20;
        // n for find if it is the first time the bot enter the field
        int  n=0, p=0;
        // creat a bot
         struct bot{
            int x;
            int y;
            int head; //1 north 2 east 3 south 4 west
            
        }mybot;
        // define initial map
        int map[14][7];
        
        for(int i=0;i<14;i++){
            for(int p=0;p<7;p++){
                map[i][p]=0;
            }
        }
         Ultra_Start();
        Ultra_GetDistance();
        
        map[0][0]=1;
        map[0][1]=1;
        map[1][0]=1;
        map[0][5]=1;
        map[0][6]=1;
        map[1][6]=1;
        
        
        IR_wait();
        getLine(speed);                                                 // get to the waiting line and wait for IR signal
        print_mqtt("Zumo037/ready"," maze");
        
        IR_wait();
        start=xTaskGetTickCount();
        print_mqtt("Zumo037/start"," %lu",start);
        
                   
                
        
        while(true){
            reflectance_digital(&dig);
            gostrat(speed,dig);
            //met a line
             reflectance_digital(&dig);
            //if it is on the west edge findstatus return 3 
             if(findStatus(dig)==3 && mybot.x==0 && ( mybot.head%4+1!=2)){ //sometimes it double caculate the x value to protect it need x value and direction 
                
                 while(findStatus(dig)==3){
                        //wait until got out of the line
                        reflectance_digital(&dig);
                        motor_forward(speed,2);
                    }
                Beep(50,150);
                
                if((mybot.head%4+1)==1){
                              mybot.y--;
                        }else if((mybot.head%4+1)==2){
                              mybot.x++;
                       }else if((mybot.head%4+1)==3){
                              mybot.y++;
                        }else if((mybot.head%4+1)==4){
                              mybot.x--;
                       }
                        print_mqtt("Zumo037/position"," %d %d", mybot.x,mybot.y);
                        
                 if(mybot.head%4+1==1){
                     if(mybot.y==2){
                        motor_forward(50,100);
                                    motor_forward(0,0);//--edit mark
                                     reflectance_digital(&dig);
                                    while(dig.r2!=1){
                                        
                                        reflectance_digital(&dig);
                                        motor_hardRight(100,0);
                                        
                                        
                                    }
                                  motor_forward(0,0);
                                
                                    mybot.head++;
                    }
                    else
                    if(map[mybot.y][mybot.x+1]==0){
                        motor_forward(50,500);
                        turn90(2,mybot.x,dig);
                        mybot.head++;
                        vTaskDelay(200);//-------edit mark
                         if(Ultra_GetDistance()<mindis){
                                map[mybot.y][mybot.x+1]=1;
                                
                                turn90(1,mybot.x,dig);
                               
                                mybot.head--;
                                
                            }
                       
                        
                    }
                }
            
            }else  //if its one east edge findStatus return 2 and the other two arguments to prevent double readings 
            if(findStatus(dig)==2 && mybot.x==6 && ( mybot.head%4+1!=4)){
                
                 while(findStatus(dig)==2){
                        //wait until got out of the line
                        reflectance_digital(&dig);
                        motor_forward(speed,2);
                    }
                Beep(50,50);
                print_mqtt("Zumo037/position"," %d %d", mybot.x,mybot.y);
                if((mybot.head%4+1)==1){
                              mybot.y--;
                        }else if((mybot.head%4+1)==2){
                              mybot.x++;
                       }else if((mybot.head%4+1)==3){
                              mybot.y++;
                        }else if((mybot.head%4+1)==4){
                              mybot.x--;
                       }
                        
                        
                 if(mybot.head%4+1==1){
                    if(map[mybot.y][mybot.x-1]==0){
                        turn90(1,mybot.x,dig);
                        mybot.head--;
                        vTaskDelay(200);//-------edit mark
                         if(Ultra_GetDistance()<mindis){
                                map[mybot.y][mybot.x-1]=1;
                                
                                turn90(2,mybot.x,dig);
                               
                                mybot.head++;
                                
                            }
                       
                        
                    }
                }
            
            }else //meet a line  in the map not on the edges
            if(findStatus(dig)==0){                                                          
               
                
                
                    while(findStatus(dig)==0){
                        //wait until got out of the line
                        reflectance_digital(&dig);
                        motor_forward(speed,2);
                    }
                
                Beep(100,100);
              
                  if(n==0){
                    mybot.x=3;
                    mybot.y=13;
                    mybot.head=4;
                    
                    n++;
                  }else {
                        if((mybot.head%4+1)==1){
                              mybot.y--;
                        }else if((mybot.head%4+1)==2){
                              mybot.x++;
                       }else if((mybot.head%4+1)==3){
                              mybot.y++;
                        }else if((mybot.head%4+1)==4){
                              mybot.x--;
                       }
                    
                    }
                print_mqtt("Zumo037/position"," %d %d", mybot.x,mybot.y);
                
                if(mybot.head%4+1==1){
                    vTaskDelay(200);//-------edit mark
                   if(Ultra_GetDistance()<mindis){           //obstical in front
                        map[mybot.y-1][mybot.x]=1;
                        //check if it is on the edges
                        if(mybot.x==6){                         
                            turn90(1,mybot.x,dig);
                            mybot.head--;
                            vTaskDelay(200);//-------edit mark
                            if(Ultra_GetDistance()<mindis){
                                map[mybot.y][mybot.x-1]=1;
                                
                                turn90(2,mybot.x,dig);
                               
                                mybot.head++;
                                
                            }
                        }else if(mybot.x==0){
                            motor_forward(50,200);
                                turn90(2,mybot.x,dig);
                                mybot.head++;
                                vTaskDelay(200);//-------edit mark
                                if(Ultra_GetDistance()<mindis){
                                    map[mybot.y][mybot.x+1]=1;
                                    turn90(2,mybot.x,dig);
                                    mybot.head++;
                                
                                }
                                
                            
                         
                        }else if(map[mybot.y][mybot.x+1]==0){//if space avalible
                            motor_forward(50,200);
                            turn90(2,mybot.x,dig);
                            mybot.head++;
                            vTaskDelay(200);//-------edit mark
                           
                            //check obj right after turn
                            if(Ultra_GetDistance()<mindis){
                                map[mybot.y-1][mybot.x]=1;
                                lturn(3,dig);
                                mybot.head=mybot.head+2;
                                
                            }
                        }else if(map[mybot.y][mybot.x-1]==0){
                            motor_forward(50,200);
                                turn90(1,mybot.x,dig);
                                mybot.head--;
                                vTaskDelay(200);//-------edit mark
                                if(Ultra_GetDistance()<mindis){
                                map[mybot.y][mybot.x-1]=1;
                                turn90(1,mybot.x,dig);
                                mybot.head--;
                                
                            }
                                
                            
                        } 
                    }else if(mybot.x>3 && map[mybot.y][mybot.x-1]==0){
                        motor_forward(50,200);
                        turn90(1,mybot.x,dig);
                        mybot.head--;
                        vTaskDelay(200);//-------edit mark
                        if(Ultra_GetDistance()<mindis){
                                map[mybot.y][mybot.x-1]=1;
                                turn90(2,mybot.x,dig);
                                mybot.head++;
                                
                            }
                    }else if(mybot.x<3 && map[mybot.y][mybot.x+1]==0){
                        motor_forward(50,200);
                        turn90(2,mybot.x,dig);
                        mybot.head++;
                        vTaskDelay(200);//-------edit mark
                        if(Ultra_GetDistance()<mindis){
                                map[mybot.y][mybot.x+1]=1;
                                turn90(1,mybot.x,dig);
                                mybot.head--;
                                
                            }
                    }else if(mybot.y==2){
                        if(mybot.x>3){
                            motor_forward(50,100);
                            turn90(1,mybot.x,dig);
                            mybot.head--;
                        }else if(mybot.x<3){
                            motor_forward(50,100);
                            turn90(2,mybot.x,dig);
                            mybot.head++;
                            
                        }
                    }
                    else if(mybot.y==0){
                        motor_forward(50,2000);
                        motor_hardRight(255,500);
                        motor_forward(0,0);
                        end=xTaskGetTickCount();
                        print_mqtt("Zumo037/stop"," %lu",end);
                        print_mqtt("Zumo037/time"," %lu",end-start);
                       
                        
                    }

                }//end of mybot.head%4+1==1
                //heading to east
                else if(mybot.head%4+1==2){
                    vTaskDelay(200);//-------edit mark
                        if(Ultra_GetDistance()<mindis){
                            map[mybot.y][mybot.x+1]=1;
                            if(map[mybot.y-1][mybot.x]==1){
                                motor_forward(50,200);
                                turn90(2,mybot.x,dig);
                                mybot.head++;
                                
                            }else{
                                motor_forward(50,200);
                                turn90(1,mybot.x,dig);
                                mybot.head--;
                            }
                            
                        
                        }else if(mybot.x>3 && map[mybot.y-1][mybot.x]==0   &&mybot.x!=6 ){
                            motor_forward(50,200);
                            turn90(1,mybot.x,dig);
                            mybot.head--;
                            //read right after turn
                            vTaskDelay(200);//-------edit mark
                            if(Ultra_GetDistance()<mindis){
                                
                                 
                               
                                    map[mybot.y-1][mybot.x]=1;
                                    
                                    turn90(2,mybot.x,dig);
                                    mybot.head++;
                                 
                            
                            }
                        }else if(mybot.x==6){
                                    motor_forward(50,100);
                                     reflectance_digital(&dig);
                                    while(dig.r2!=1){
                                        
                                        reflectance_digital(&dig);
                                        motor_hardLeft(100,0);
                                        
                                        
                                    }
                                  motor_forward(0,0);
                                
                                    mybot.head--;
                                  
                                    
                       }else if(mybot.x==3 && mybot.y<=2){
                                motor_forward(50,200);
                                turn90(2,mybot.x,dig);
                                mybot.head++;
                                
                    }
                }//end of east
                //south
                else if(mybot.head%4+1==3){
                    Beep(255,100);
                    motor_forward(50,200);
                    turn90(2,mybot.x,dig);
                    mybot.head++;
                    
                    
                    
                }//end of south
                //west
                else if(mybot.head%4+1==4){
                    vTaskDelay(200);//-------edit mark
                    if(Ultra_GetDistance()<mindis){
                            map[mybot.y][mybot.x-1]=1;
                            if(map[mybot.y-1][mybot.x]==1){
                                motor_forward(50,200);
                                turn90(1,mybot.x,dig);
                                mybot.head--;
                                
                            }else{
                                motor_forward(50,200);
                                turn90(2,mybot.x,dig);
                                mybot.head++;
                                vTaskDelay(200);//-------edit mark
                                if(Ultra_GetDistance()<mindis){
                                    map[mybot.y-1][mybot.x]=1;
                                    turn90(2,mybot.x,dig);
                                    mybot.head++;
                                    
                                }
                            }
                            
                        
                        }else if(mybot.x==3){
                             motor_forward(50,200);
                                turn90(2,mybot.x,dig);
                                mybot.head++;
                                vTaskDelay(200);//-------edit mark
                                if(Ultra_GetDistance()<mindis){
                                    map[mybot.y-1][mybot.x]=1;
                                    turn90(1,mybot.x,dig);
                                    mybot.head--;
                                    
                                }
                            
                        }
                        if(mybot.x==0){
                            motor_forward(50,100);//-------edit mark
                                     reflectance_digital(&dig);
                                    while(dig.l2!=1){
                                        
                                        reflectance_digital(&dig);
                                        motor_hardRight(100,0);
                                        
                                        
                                    }
                                  motor_forward(0,0);
                                
                                    mybot.head++;
                        }else if(mybot.x<3){//-------edit mark
                             motor_forward(50,200);
                                turn90(2,mybot.x,dig);
                                mybot.head++;
                                vTaskDelay(200);
                                if(Ultra_GetDistance()<mindis){
                                    map[mybot.y-1][mybot.x]=1;
                                    turn90(1,mybot.x,dig);
                                    mybot.head--;
                                    
                                }
                            
                        }
                    
                }//end of west 
                    
               
               
            
            }
            
           
        }
         
                
    } 
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if 0 //line following 
    void zmain(){
        
        
        
        motor_start();
        reflectance_start();
        int speed=50;                                          //initial speed 
         struct accData_ data;                                  //acc_sensor data
        struct sensors_ dig,threshold;                          // reflection sensor data
        threshold=findThres();// find threshold
        
        reflectance_set_threshold(threshold.l3,threshold.l2,threshold.l1,threshold.r1,threshold.r2,threshold.r3);           // set thresthold
        
        if(LSM303D_Start()){
            while(true){
                reflectance_digital(&dig);
                if(dig.l3==1 && dig.l2==1 && dig.l1==1 && dig.r1==1 &&dig.r2==1 &&dig.r3==1 ){
                    
                }
                
                
                
            }
            
            
            
        }
        
    }
#endif
    
#if 0// Sumo wrestling
    void zmain(){
        motor_start();
        reflectance_start();
        Ultra_Start();
        IR_Start();
        IR_flush();
        int speed=150;
        int co=60; //how sensible it is
        int delay=230;// delay for turning
        int count=3;
        TickType_t start, end;//time count
        
       
        
        struct accData_ data, stable;
        
        struct sensors_ dig,threshold;
        threshold=findThres();
        Beep(255,100);
        
        reflectance_set_threshold(threshold.l3,threshold.l2,threshold.l1,threshold.r1,threshold.r2,threshold.r3);
        
      if(LSM303D_Start()){
        vTaskDelay(2000);
        stable=findAccStatus();
        print_mqtt("Zumo044/stable","stable %10d,%10d,%10d", stable.accX, stable.accY, stable.accZ );
        Beep(100,255);
        while(SW1_Read()==1){
            vTaskDelay(2);
        }
        int status=0;
        int temp=findStatus(dig);
        
        while(count>0){
            reflectance_digital(&dig);
                    temp=findStatus(dig);//get new status
                    motor_forward(50,2); 
                    if(temp!=status){
                        count--;
                        
                    }
                    status=temp;
            
        }
        motor_forward(0,0);
        print_mqtt("Zumo044/ready","zumo044");
        Beep(200,200);
        IR_wait();
        
        print_mqtt("Zumo044/start ","%d",start);
        motor_forward(100,50);
        
        
       
            
                while(SW1_Read()==1){
                    
                        LSM303D_Read_Acc(&data);
                        reflectance_digital(&dig);
                        start=xTaskGetTickCount();
                        
                        
                        motor_forward(speed,2);
                        
                        if(dig.l3==1 && dig.r3!=1){
                            speed=150;
                            motor_forward(0,0);
                                motor_backward(100,500);
                               
                              motor_hardRight(255,delay);
                            motor_forward(0,0);
                        }
                        if(dig.l3!=1 && dig.r3==1){
                            speed=150;
                            motor_forward(0,0);
                            motor_backward(100,500);
                            
                             motor_hardLeft(255,delay);
                            motor_forward(0,0);
                             
                        }
                       
                        
                        //printf("%10d,%10d,%10d\n", data.accX, data.accY, data.accZ );
                        if(data.accX<-co*stable.accX && data.accY<co*stable.accY){
                            speed=150;
                            Beep(100,255);
                            print_mqtt("Zumo044/hit ","Got hit from left front(135 degree) ");
                            motor_hardLeft(255,delay);
                            
                            
                        }
                        if(data.accX<-co*stable.accX && data.accY>co*abs(stable.accY)){
                            speed=150;
                            Beep(100,255);
                            print_mqtt("Zumo044/hit ","Got hit from right front(45degree)");
                            motor_hardRight(255,delay);
                             
                            
                        }
                        if(data.accX>co*stable.accX && data.accY>co*abs(stable.accY)){
                            speed=150;
                            Beep(100,255);
                            print_mqtt("Zumo044/hit ","Got hit from right back( 135 degree)");
                            motor_hardRight(255,delay+200);
                             
                            
                        }
                         if(data.accX>co*stable.accX && data.accY<-co*abs(stable.accY)){
                            speed=150;
                            Beep(100,255);
                            print_mqtt("Zumo044/hit ","Got hit from left back(135 degree)");
                            motor_hardRight(255,delay+200);
                            
                            
                        }
                        if(data.accX<-co*stable.accX && data.accY<abs(2000-stable.accY)){
                            
                            Beep(100,255);
                            
                            print_mqtt("Zumo044/hit ","Got hit from front(0 degree)");
                            
                            speed=255;
                            
                            
                        }
                        if(data.accX>25000  && data.accY<abs(3000-stable.accY)){
                            speed=100;
                            Beep(100,255);
                            print_mqtt("Zumo044/hit ","Got hit from back(180 degree)");
                            motor_hardRight(255,delay+200);
                            
                        }
                        if(data.accX<abs(500-stable.accX) && data.accY<-abs(co*stable.accY)){
                            speed=100;
                            Beep(200,100);//from left;
                            print_mqtt("Zumo044/hit ","Got hit from left(270 degree)");
                            motor_hardLeft(255,delay);
                        }
                        if(data.accX<abs(500-stable.accX) && data.accY>abs(co*stable.accY)){
                            speed=100;
                            Beep(200,100);//from right;
                            print_mqtt("Zumo044/hit ","Got hit from right(90 degree)");
                            motor_hardRight(255,delay);
                        }
                }
                end=xTaskGetTickCount();
                print_mqtt("Zumo044/stop ","%d",end);
                print_mqtt("Zumo044/time ","%d",end-start);
            
        
    }
    
        
        
    }
    
#endif
#if 0// 5.3
    void zmain(){
        reflectance_start();
        motor_start();
        RTC_Start();
        RTC_TIME_DATE now;
        
        
        IR_Start();
        IR_flush();
        struct sensors_ threshold,dig;
        threshold=findThres();
        
        reflectance_set_threshold(threshold.l3,threshold.l2,threshold.l1,threshold.r1,threshold.r2,threshold.r3);
        TickType_t start, end;
        int count=0;
        int status=0;
        int temp=findStatus(dig);//init the status 
        while(SW1_Read()==1){
            vTaskDelay(2);
        }
        int i=-1;
        while(i==-1){
            reflectance_digital(&dig);
            temp=findStatus(dig);
            motor_forward(50,2);
                    if(temp!=status){
                        Beep(100,255);
                        count++;
                        if(count==3){
                            motor_forward(0,0);
                            IR_wait();
                            start=xTaskGetTickCount();
                        }
                        
                        
                        motor_forward(50,2);
                        if(count==4){
                           end=xTaskGetTickCount();
                            print_mqtt("Zumo044/lap ","%d milliseconds",end-start);
                            motor_forward(0,0);
                            i=0;
                            
                        }
                        
                        status=temp;
                        
                    }
        }
        
        
        
    }
#endif
#if 0 //5.2 
    void zmain(){
        Ultra_Start();
        motor_start();
        Ultra_GetDistance();
        vTaskDelay(1000);
        Beep(100,255);
        while(SW1_Read()==1){
            vTaskDelay(2);
        }
        while(true){
        
        motor_forward(100,2);
        if(Ultra_GetDistance()<10){
            motor_forward(0,0);
            motor_backward(100,1000);
            int turn=rand()%2;
            if(turn==0){//turn left
                motor_hardLeft(100,500);
                print_mqtt("Zumo044/turn ","left");
                
            }else{          // turn right
                motor_hardRight(100,500);
                print_mqtt("Zumo044/turn ","right");
            }
            
        }
        
        }
        
        
        
        
        
    
       
        
        
        
        
         
    }
#endif
#if 0
    
    
    
       //5.1 
    void zmain(){
        int8 hour=0,minute=0,sec=0;
        RTC_Start();
        RTC_TIME_DATE now;
        printf("Please inpout a time hh:mm:ss\n");
        scanf("%d, %d, %d", &hour,&minute,&sec);
        now.Hour=hour;
        now.Min=minute;
        now.Sec=sec;
        RTC_WriteTime(&now);
        while(true){
            if(SW1_Read()==0){
                RTC_DisableInt();
                now=*RTC_ReadTime();
                RTC_EnableInt();
                print_mqtt("zumo114/time is:","%hd:%hd:%hd",now.Hour,now.Min,now.Sec);
                
            }
        }
    }
       
      /* int hour=0,minute=0;
        int result_minute=0;
        TickType_t start=0,end=0;
        vTaskDelay(5000);
        printf("please enter a time in format HH:MM ");
        scanf("%d:%d",&hour,&minute);
        start=xTaskGetTickCount(); 
       while(true){
            if(SW1_Read()==0){
                end=xTaskGetTickCount();
                break;
            }
        }
        result_minute=(((end-start)/1000)/60)+100;
         print_mqtt("zumo114/result is ", "%d", result_minute);
        if(result_minute>=60){
            hour=hour+result_minute/60;
         minute=result_minute%60;
        }else{
            minute=result_minute+minute;
        }
        print_mqtt("zumo114/Time now is", "%d:%d", hour,minute);*/
        
        
    
    
    //}

#endif

#if 0
    //4.3
    
     

void zmain(void){   
        
            
    
    
    
    struct sensors_ ref;
    struct sensors_ dig;
    IR_Start();
    IR_flush();
    
    
    reflectance_start(); 
     //dark is 1 white 0
    int count=0;//need to change 
    int status=0;
    motor_start();
    motor_forward(0,0);
    while(SW1_Read()==1){
        vTaskDelay(2);
    }
    struct sensors_ threshold;
    threshold=findThres();
    reflectance_set_threshold(threshold.l3,threshold.l2,threshold.l1,threshold.r1,threshold.r2,threshold.r3);
    
    Beep(100,255);
     vTaskDelay(1000);
    Beep(100,255);
    //reflectance_digital(&dig);
    while(true){
        reflectance_digital(&dig);
       
        motor_forward(50,2);
        int temp=findStatus(dig);
        if(temp!=status){
            Beep(100,255);
            count++;
            if(count==2){
                motor_forward(0,0);
                IR_wait();
            }
            if(count>=4 && temp==0){
                motor_forward(0,0);
                break;
            }
            status= temp;
            
        }
        if(count>2){
            if(dig.l3==1 && dig.r3!=1){        
                                       
                    motor_turn(0,220,50);
                }if( dig.r3==1 && dig.l3!=1){
                    motor_turn(220,0,50);
                                    
                }else{
                    motor_forward(50,2);
                }
               
                
                
                
                
            }
        }
}


#endif



#if 0
    
//4.2
    /*void motor_hardLeft(uint8 speed,uint32 delay)
{
    MotorDirLeft_Write(1);      // set LeftMotor forward mode
    MotorDirRight_Write(0);     // set RightMotor forward mode
    PWM_WriteCompare1(speed); 
    PWM_WriteCompare2(speed); 
    vTaskDelay(delay);
}
void motor_hardRight(uint8 speed,uint32 delay)
{
    MotorDirLeft_Write(0);      // set LeftMotor forward mode
    MotorDirRight_Write(1);     // set RightMotor forward mode
    PWM_WriteCompare1(speed); 
    PWM_WriteCompare2(speed); 
    vTaskDelay(delay);
}
int findStatus(struct sensors_ dig){
                int st=-1;
                if(dig.l1==1 && dig.l2==1 && dig.l3==1 && dig.r1==1 && dig.r2==1 && dig.r3==1){
                              st=0;
                        }else{
                            st=1;
                        }
                        return st;
}*/

void zmain(void)
{      
    struct sensors_ ref;
    struct sensors_ dig;
    IR_Start();
    IR_flush();
    
    
    reflectance_start(); 
    reflectance_set_threshold(8300, 7000, 6000, 6000, 6700, 9600); //dark is 1 white 0
    int count=0;//need to change 
    int status=0;
    motor_start();
    motor_forward(0,0);
    while(SW1_Read()==1){
        vTaskDelay(2);
    }
    vTaskDelay(2000);
    //reflectance_digital(&dig);
    while(true){
        reflectance_digital(&dig);
       
        motor_forward(50,2);
        int temp=findStatus(dig);
        if(temp!=status){
            Beep(100,255);
            count++;
            if(count==2){
                motor_forward(0,0);
                IR_wait();
            }
            if(count==5){
                motor_forward(0,0);
                motor_hardLeft(215,200);
            }
            if(count==7){
                motor_forward(0,0);
                motor_hardRight(220,200);
            }
            if(count== 9){
                motor_forward(0,0);
                motor_hardRight(220,200);
                
            }
            if(count==10){
                motor_forward(0,0);
                break;
                
            }
           
            status=temp;
        }
            
           
        
       
    }
    
                      
    
  
}   
#endif

#if 0
// 4.1
    int findStatus(struct sensors_ dig){
                int st=-1;
                if(dig.l1==1 && dig.l2==1 && dig.l3==1 && dig.r1==1 && dig.r2==1 && dig.r3==1){
                              st=0;
                        }else{
                            st=1;
                        }
                        return st;
}
void zmain(void)
{
    struct sensors_ ref;
    struct sensors_ dig;
    IR_Start();
    IR_flush();
    
    
    reflectance_start(); 
    reflectance_set_threshold(8300, 7000, 6000, 6000, 6700, 9600); //dark is 1 white 0
    int count=8;
    int status=0;
    motor_start();
    motor_forward(0,0);
    while(SW1_Read()==1){
        vTaskDelay(2);
    }
    //reflectance_digital(&dig);
    while(true){
        reflectance_digital(&dig);
       
        motor_forward(50,2);
        int temp=findStatus(dig);
        if(temp!=status){
            Beep(100,255);
            count--;
            if(count==6){
                motor_forward(0,0);
                IR_wait();
                
                
            }
            if(count==0){
                motor_forward(0,0);
                break;
            }
            status=temp;
        }
            
           
        
       
    }
    
    
 }  


#endif

#if 0
    
// Name and age
void zmain(void)
{
    int button=SW1_Read();
    
   while(button==0) {
        printf("You pressed button\n");
        printf("press again:");
        button=
    }

  
 }   
#endif


#if 0
//battery level//
void zmain(void)
{
    ADC_Battery_Start();        

    int16 adcresult =0;
    float volts = 0.0;

    printf("\nBoot\n");

    //BatteryLed_Write(1); // Switch led on 
    BatteryLed_Write(0); // Switch led off 
    //uint8 button;
    //button = SW1_Read(); // read SW1 on pSoC board
    // SW1_Read() returns zero when button is pressed
    // SW1_Read() returns one when button is not pressed

    while(true)
    {
        char msg[80];
        ADC_Battery_StartConvert(); // start sampling
        if(ADC_Battery_IsEndConversion(ADC_Battery_WAIT_FOR_RESULT)) {   // wait for ADC converted value
            adcresult = ADC_Battery_GetResult16();
            // get the ADC value (0 - 4095)
            // convert value to Volts
            // you need to implement the conversion
            
            // Print both ADC results and converted value
            printf("%d %f\r\n",adcresult, volts);
        }
        vTaskDelay(500);
    }
 }   
#endif

#if 0
// button
void zmain(void)
{
    while(true) {
        printf("Press button within 5 seconds!\n");
        int i = 50;
        while(i > 0) {
            if(SW1_Read() == 0) {
                break;
            }
            vTaskDelay(100);
            --i;
        }
        if(i > 0) {
            printf("Good work\n");
            while(SW1_Read() == 0) vTaskDelay(10); // wait until button is released
        }
        else {
            printf("You didn't press the button\n");
        }
    }
}
#endif

#if 0
// button
void zmain(void)
{
    printf("\nBoot\n");

    //BatteryLed_Write(1); // Switch led on 
    BatteryLed_Write(0); // Switch led off 
    
    //uint8 button;
    //button = SW1_Read(); // read SW1 on pSoC board
    // SW1_Read() returns zero when button is pressed
    // SW1_Read() returns one when button is not pressed
    
    bool led = false;
    
    while(true)
    {
        // toggle led state when button is pressed
        if(SW1_Read() == 0) {
            led = !led;
            BatteryLed_Write(led);
            if(led) printf("Led is ON\n");
            else printf("Led is OFF\n");
            Beep(1000, 150);
            while(SW1_Read() == 0) vTaskDelay(10); // wait while button is being pressed
        }        
    }
 }   
#endif


#if 0
//ultrasonic sensor//
void zmain(void)
{
    Ultra_Start();                          // Ultra Sonic Start function
    
    while(true) {
        int d = Ultra_GetDistance();
        // Print the detected distance (centimeters)
        printf("distance = %d\r\n", d);
        vTaskDelay(200);
    }
}   
#endif

#if 0
//IR receiverm - how to wait for IR remote commands
void zmain(void)
{
    IR_Start();
    
    printf("\n\nIR test\n");
    
    IR_flush(); // clear IR receive buffer
    printf("Buffer cleared\n");
    
    bool led = false;
    // Toggle led when IR signal is received
    while(true)
    {
        IR_wait();  // wait for IR command
        led = !led;
        BatteryLed_Write(led);
        if(led) printf("Led is ON\n");
        else printf("Led is OFF\n");
    }    
 }   
#endif



#if 0
//IR receiver - read raw data
void zmain(void)
{
    IR_Start();
    
    uint32_t IR_val; 
    
    printf("\n\nIR test\n");
    
    IR_flush(); // clear IR receive buffer
    printf("Buffer cleared\n");
    
    // print received IR pulses and their lengths
    while(true)
    {
        if(IR_get(&IR_val, portMAX_DELAY)) {
            int l = IR_val & IR_SIGNAL_MASK; // get pulse length
            int b = 0;
            if((IR_val & IR_SIGNAL_HIGH) != 0) b = 1; // get pulse state (0/1)
            printf("%d %d\r\n",b, l);
        }
    }    
 }   
#endif


#if 0
//reflectance
void zmain(void)
{
    struct sensors_ ref;
    struct sensors_ dig;

    reflectance_start();
    reflectance_set_threshold(9000, 9000, 11000, 11000, 9000, 9000); // set center sensor threshold to 11000 and others to 9000
    

    while(true)
    {
        // read raw sensor values
        reflectance_read(&ref);
        // print out each period of reflectance sensors
        printf("%5d %5d %5d %5d %5d %5d\r\n", ref.l3, ref.l2, ref.l1, ref.r1, ref.r2, ref.r3);       
        
        // read digital values that are based on threshold. 0 = white, 1 = black
        // when blackness value is over threshold the sensors reads 1, otherwise 0
        reflectance_digital(&dig); 
        //print out 0 or 1 according to results of reflectance period
        printf("%5d %5d %5d %5d %5d %5d \r\n", dig.l3, dig.l2, dig.l1, dig.r1, dig.r2, dig.r3);        
        
        vTaskDelay(200);
    }
}   
#endif


#if 0
//motor
void zmain(void)
{
    motor_start();              // enable motor controller
    motor_forward(0,0);         // set speed to zero to stop motors

    vTaskDelay(3000);
    
    motor_forward(100,2000);     // moving forward
    motor_turn(200,50,2000);     // turn
    motor_turn(50,200,2000);     // turn
    motor_backward(100,2000);    // moving backward
     
    motor_forward(0,0);         // stop motors

    motor_stop();               // disable motor controller
    
    while(true)
    {
        vTaskDelay(100);
    }
}
#endif

#if 0
/* Example of how to use te Accelerometer!!!*/
void zmain(void)
{
    
    struct accData_ data;
    struct accData_ initial;
    
    printf("Accelerometer test...\n");

    if(!LSM303D_Start()){
        printf("LSM303D failed to initialize!!! Program is Ending!!!\n");
      
    }
    else {
        printf("Device Ok...\n");
    }
    
    vTaskDelay(3000);
    LSM303D_Read_Acc(&data);
    initial=data;
    int turn;
    
    motor_start();
    motor_forward(0,0);
    
    turn=rand()%2;
            if(turn==0){
                motor_turn(0,100,500);
            }else{
                motor_turn(100,0,500);
            }
    
    while(true)
    {
         LSM303D_Read_Acc(&data);
          motor_forward(100,2);
        
            
            
        if(abs(data.accX-initial.accX)>15000 ){
            Beep(200,255);
            motor_forward(0,0);
            motor_backward(100,500);
     
        
            turn=rand()%2;
                if(turn==0){
                     motor_turn(0,100,500);
                }else{
                    motor_turn(100,0,500);
            }
        
       
    }
 }   
#endif    

#if 0
// MQTT test
void zmain(void)
{
    int ctr = 0;

    printf("\nBoot\n");
    send_mqtt("Zumo01/debug", "Boot");

    //BatteryLed_Write(1); // Switch led on 
    BatteryLed_Write(0); // Switch led off 

    while(true)
    {
        printf("Ctr: %d, Button: %d\n", ctr, SW1_Read());
        print_mqtt("Zumo01/debug", "Ctr: %d, Button: %d", ctr, SW1_Read());

        vTaskDelay(1000);
        ctr++;
    }
 }   
#endif


#if 0
void zmain(void)
{    
    struct accData_ data;
    struct sensors_ ref;
    struct sensors_ dig;
    
    printf("MQTT and sensor test...\n");

    if(!LSM303D_Start()){
        printf("LSM303D failed to initialize!!! Program is Ending!!!\n");
        vTaskSuspend(NULL);
    }
    else {
        printf("Accelerometer Ok...\n");
    }
    
    int ctr = 0;
    reflectance_start();
    while(true)
    {
        LSM303D_Read_Acc(&data);
        // send data when we detect a hit and at 10 second intervals
        if(data.accX > 1500 || ++ctr > 1000) {
            printf("Acc: %8d %8d %8d\n",data.accX, data.accY, data.accZ);
            print_mqtt("Zumo01/acc", "%d,%d,%d", data.accX, data.accY, data.accZ);
            reflectance_read(&ref);
            printf("Ref: %8d %8d %8d %8d %8d %8d\n", ref.l3, ref.l2, ref.l1, ref.r1, ref.r2, ref.r3);       
            print_mqtt("Zumo01/ref", "%d,%d,%d,%d,%d,%d", ref.l3, ref.l2, ref.l1, ref.r1, ref.r2, ref.r3);
            reflectance_digital(&dig);
            printf("Dig: %8d %8d %8d %8d %8d %8d\n", dig.l3, dig.l2, dig.l1, dig.r1, dig.r2, dig.r3);
            print_mqtt("Zumo01/dig", "%d,%d,%d,%d,%d,%d", dig.l3, dig.l2, dig.l1, dig.r1, dig.r2, dig.r3);
            ctr = 0;
        }
        vTaskDelay(10);
    }
 }   

#endif

#if 0
void zmain(void)
{    
    RTC_Start(); // start real time clock
    
    RTC_TIME_DATE now;

    // set current time
    now.Hour = 12;
    now.Min = 34;
    now.Sec = 56;
    now.DayOfMonth = 25;
    now.Month = 9;
    now.Year = 2018;
    RTC_WriteTime(&now); // write the time to real time clock

    while(true)
    {
        if(SW1_Read() == 0) {
            // read the current time
            RTC_DisableInt(); /* Disable Interrupt of RTC Component */
            now = *RTC_ReadTime(); /* copy the current time to a local variable */
            RTC_EnableInt(); /* Enable Interrupt of RTC Component */

            // print the current time
            printf("%2d:%02d.%02d\n", now.Hour, now.Min, now.Sec);
            
            // wait until button is released
            while(SW1_Read() == 0) vTaskDelay(50);
        }
        vTaskDelay(50);
    }
 }   
#endif

/* [] END OF FILE */
