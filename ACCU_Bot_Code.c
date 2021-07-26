/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#define COMPARE 9250 //PWM Duty Cycle
#define STOP 0
#define RSTSE 1
#define MTRSTP 6
#define WHEELFORWR 8
#define WHEELFORWL 16

#define SERVO 4

#define OPEN 12
#define GRIP 5

#define LIFT 20
#define DROP 6

#define FLICK 17
#define RETRACT 24

// Level
#define LEVEL1 3
#define LEVEL2 1
#define LEVEL3 2
#define LEVEL4 6
#define LEVEL5 8
#define LEVEL6 10
#define HMC5883L 0x0D

int cmp = 0;

char buff[70]; //UART buffer string
int hold = 0;

int directional = 1; //virtual North
int limit = 8;
//int distance = 3;

int state = 0;

int16_t countSE_R = 0; //Shaft Encoder Count 4 Right
int16_t countSE_L = 0; //Shaft Encoder Count 4 Left
int revoCounter = 0;

// Ultrasonic sensor
uint16 count1 = 0;
uint16 count2 = 0;
uint16 count3 = 0;
uint16 count4 = 0;
int countdown = 0;


int distance_measured1 = 0;
int distance_measured2 = 0;
int distance_measured3 = 0;
int distance_measured4 = 0;


//Colour Sensor
uint16_t R = 0 ;
uint16_t B = 0 ;
uint16_t G = 0 ;
int ColourType = 0;

// Front
CY_ISR(Timer_ISR_Handler1)
{
    Sonar_1_ReadStatusRegister();
    
    count1 = Sonar_1_ReadCounter();
    distance_measured1 = (65535 - count1)/58; // distance measured in cm
   
}

// Right
CY_ISR(Timer_ISR_Handler2)
{ 
    Sonar_2_ReadStatusRegister();
    
    count2 = Sonar_2_ReadCounter();
    distance_measured2 = (65535 - count2)/58; // distance measured in cm

}

// Back
CY_ISR(Timer_ISR_Handler3)
{
    Sonar_3_ReadStatusRegister();
    
    count3 = Sonar_3_ReadCounter();
    distance_measured3 = (65535 - count3)/58; // distance measured in cm
        
}

// Left
CY_ISR(Timer_ISR_Handler4)
{
    Sonar_4_ReadStatusRegister();
    
    count4 = Sonar_4_ReadCounter();
    distance_measured4 = (65535 - count4)/58; // distance measured in cm
    
}

// Reverse
void Reverse()
{
    Rst_Write(RSTSE);
    
    int error = 0;
    
    countSE_R = QuadDec_R_GetCounter();
    countSE_L = QuadDec_L_GetCounter();
    
    error = countSE_R - countSE_L;
    
    if (error >= 100)
    {
        cmp += 2;
        PWM_R_WriteCompare(cmp);
    }
    else if (error <= -100)
    {
        cmp -= 2;
        PWM_R_WriteCompare(cmp);
    }
    else
    {
        PWM_R_WriteCompare(cmp);   
    }

    if ((countSE_L <= -10000) || (countSE_R <= -10000))
    {
        Rst_Write(0);
        revoCounter += 1;
    }
    //sprintf(buff, "L = %d , R = %d, compare = %d, %d\n", countSE_L, countSE_R, cmp, state);
    //UART_1_PutString(buff);
}


//State 1: Speed of the right wheel
void SpeedCtrl()
{   
    Rst_Write(RSTSE + WHEELFORWR + WHEELFORWL);
    
    int error = 0;
    
    countSE_R = QuadDec_R_GetCounter();
    countSE_L = QuadDec_L_GetCounter();
    
    error = countSE_R - countSE_L;
    
    if (error >= 100)
    {
        cmp -= 2;
        PWM_R_WriteCompare(cmp);
    }
    else if (error <= -100)
    {
        cmp += 2;
        PWM_R_WriteCompare(cmp);
    }
    else
    {
        PWM_R_WriteCompare(cmp);   
    }

    if ((countSE_L >= 10000) || (countSE_R >= 10000))
    {
        revoCounter += 1;
        Rst_Write(0 + WHEELFORWL + WHEELFORWR);
    }
    
    //sprintf(buff, "L = %d , R = %d, compare = %d, %d\n", countSE_L, countSE_R, cmp, state);
    //UART_1_PutString(buff);
}


void LeftTurn()
{
    //UART_1_PutString("hi2\n");
    Rst_Write(0);
    CyDelay(10);
    Rst_Write(MTRSTP);
    CyDelay(50);

    PWM_R_WriteCompare(COMPARE);

    PWM_L_WriteCompare(COMPARE);
    Rst_Write(RSTSE + WHEELFORWR);
    countSE_R = QuadDec_R_GetCounter();
    //UART_1_PutString("LEFT\n");
    while (countSE_R <= 14300)
    {
        
        countSE_R = QuadDec_R_GetCounter();
        
        //sprintf(buff, "L = %d , R = %d, compare = %d, lt\n", countSE_L, countSE_R, compare);
        //UART_1_PutString(buff);
        CyDelay(1);
    }
    
    Rst_Write(0);
    
    CyDelay(10);

    Rst_Write(MTRSTP + WHEELFORWL + WHEELFORWR);

    CyDelay(50);
    
    directional -= 1;
    
    if (directional == 0)
    {
        directional = 4;   
    }
    
    //UART_1_PutString("kt\n");
    
    cmp = COMPARE;  // Reset

    CyDelay(1);
}

void RightTurn()
{
 
    Rst_Write(0);
    CyDelay(10);
    Rst_Write(MTRSTP);
    CyDelay(50);
    
    PWM_L_WriteCompare(COMPARE);
    
    PWM_R_WriteCompare(COMPARE);
    
    Rst_Write(RSTSE + WHEELFORWL);
    countSE_L = QuadDec_L_GetCounter();
    //UART_1_PutString("RIGHT\n");
    while (countSE_L <= 14650)
    {
        countSE_L = QuadDec_L_GetCounter();
        
        //sprintf(buff, "L = %d , R = %d, compare = %d, rt\n", countSE_L, countSE_R, compare);
        //UART_1_PutString(buff);
        CyDelay(1);
    }

    Rst_Write(0);
    
    CyDelay(10);
    
    Rst_Write(MTRSTP + WHEELFORWL + WHEELFORWR);
    
    CyDelay(50);
    
    directional += 1;
    
    if (directional == 5)
    {
        directional = 1;   
    }    
    //Rst_Write(3);
    cmp = COMPARE;  // Reset

    CyDelay(1);
}

void FrontSonar()
{
    Trigger_1_Write(1);
    CyDelayUs(10);
    Trigger_1_Write(0);

    Trigger_2_Write(1);
    CyDelayUs(10);
    Trigger_2_Write(0);
    
}

void SideSonar()
{
    Trigger_3_Write(1);
    CyDelayUs(10);
    Trigger_3_Write(0);

    Trigger_4_Write(1);
    CyDelayUs(10);
    Trigger_4_Write(0);
}

void ResetCounter()
{
    countdown = 0;
    revoCounter = 0;
    cmp = COMPARE;
    if (hold != 0)
    {
        revoCounter = hold;   
        hold = 0;
    }
}

void PuckLifting()
{
    Rst_Write(MTRSTP);
                        
    PWM_Grip_Start(); 
    PWM_Servo_Start(); 
    PWM_Grip_WriteCompare(OPEN);
    Colour_Rst_Write(0);
    PWM_Servo_WriteCompare(DROP);
    //CyDelay(500);
    //PWM_Servo_Stop();
    
    CyDelay(300);
    PWM_Grip_WriteCompare(GRIP);
    CyDelay(500);
    //PWM_Grip_Stop();
    //PWM_Servo_Start();
    
    if(state == 4)
    {
        PWM_Servo_WriteCompare(LIFT+1);
    }
    else
    {
        PWM_Servo_WriteCompare(LIFT);   
    }
    
    CyDelay(500);
    PWM_Servo_Stop();
    //PWM_Grip_Stop();
    
    //revoCounter = 0;   
}

void PuckLanding()
{
    PWM_Servo_Start();
    PWM_Grip_Start();
    PWM_Grip_WriteCompare(GRIP);
    PWM_Servo_WriteCompare(DROP);
    CyDelay(600);
    
    PWM_Grip_WriteCompare(OPEN);
    CyDelay(300);
    
    if (state == 11)
    {
        ResetCounter();
        PWM_Servo_WriteCompare(LIFT);
        CyDelay(500);
    }
    
    PWM_Grip_Stop();
    PWM_Servo_Stop();
    
    cmp = COMPARE;
    
    
    Blue_Write(0);
    Red_Write(0);
    Green_Write(0);   
}

void ObstacleAvoidance(int left, int right)
{
    int incremental = 0;
    int hold = 0;
    
    
    Rst_Write(MTRSTP);
//    PWM_Servo_Start();
    PWM_Grip_Start(); 
    PWM_Grip_WriteCompare(OPEN+6);
//    PWM_Servo_WriteCompare(LIFT);
    while ((distance_measured1 <= 20) || (distance_measured2 <= 20))
    {
        FrontSonar();
        incremental++;
//        sprintf(buff,"%d, %d\n",distance_measured1, distance_measured2);
//        UART_1_PutString(buff);
        if (incremental == 250)
        {
            hold = revoCounter;
            revoCounter = 0;
            limit-=3;
            while(revoCounter == 0)
            {
                Reverse();   
            }
            if(right>left)
            {
                RightTurn();
            }
            else
            {
                LeftTurn();   
            }
            PWM_Servo_Start();
            PWM_Servo_WriteCompare(DROP);
            PWM_Grip_WriteCompare(OPEN);
            CyDelay(300);
            PWM_Grip_Stop();
            PWM_Servo_Stop();
            
            while(1)
            {
                if (revoCounter > 3)
                {
                    Rst_Write(MTRSTP);
                    revoCounter = hold;
                    countdown = 0;
                    break;
                }
                else
                {
                    SpeedCtrl();
                    if (Infrared2_Read() == 0)
                    {
                        Rst_Write(0);
                        state = 4;
                        //revoCounter = 0;
//                        distance += 2;
                        Rst_Write(MTRSTP);
                        return;
                    }
                }
                
            }
            if(right > left)
            {
                LeftTurn();  
                
                
            }
            else
            {
                RightTurn();   
                
            }
            PWM_Servo_Start();
            PWM_Servo_WriteCompare(DROP);
            PWM_Grip_WriteCompare(OPEN);
            CyDelay(500);
            PWM_Grip_Stop();
            PWM_Servo_Stop();
            return;
        }
        CyDelay(10);
    }
    PWM_Servo_Start();
    PWM_Servo_WriteCompare(DROP);
    PWM_Grip_WriteCompare(OPEN);
    CyDelay(350);
    PWM_Grip_Stop();
    PWM_Servo_Stop();
    
}

int ObstacleAvoidance2(int right,int left, int level)
{
    int incremental = 0;
    int x = 0;
   
    
    Rst_Write(MTRSTP);
//    PWM_Servo_Start();
    PWM_Grip_Start(); 
    
    PWM_Grip_WriteCompare(OPEN+6);
//    PWM_Servo_WriteCompare(LIFT);
    
    while ((distance_measured1 <= 20) || (distance_measured2 <= 20))
    {
        incremental++;
        FrontSonar();
//        sprintf(buff,"%d, %d\n",distance_measured1, distance_measured2);
//        UART_1_PutString(buff);
        if (incremental == 200)
        {
            x = revoCounter;
            revoCounter = 0;
            while(revoCounter == 0)
            { 
                Reverse();
            }
            if(right < left)
            {
                countdown = 0;
                
                switch(level)
                {
                    case LEVEL1: LeftTurn(); x = -LEVEL1+revoCounter; break;
                    case LEVEL2: RightTurn(); x = -LEVEL2+revoCounter; break;
                    case LEVEL3: LeftTurn(); x = LEVEL3-revoCounter; break;
                    case LEVEL4: LeftTurn(); x = LEVEL4-revoCounter; break;
                    case LEVEL5: LeftTurn(); x = LEVEL5-revoCounter; break;
                    case LEVEL6: LeftTurn(); x = LEVEL6-revoCounter; break;
                }
                revoCounter = 20;
                cmp = COMPARE;
                state = 18;
            }
            else
            {
                countdown = 0;
                
                switch(level)
                {
                    case LEVEL1: RightTurn(); x = -LEVEL1+revoCounter; break;
                    case LEVEL2: LeftTurn(); x = -LEVEL2+revoCounter; break;
                    case LEVEL3: RightTurn(); x = LEVEL3-revoCounter; break;
                    case LEVEL4: RightTurn(); x = LEVEL4-revoCounter; break;
                    case LEVEL5: RightTurn(); x = LEVEL5-revoCounter; break;
                    case LEVEL6: RightTurn(); x = LEVEL6-revoCounter; break;
                }
                revoCounter = 20;
                cmp = COMPARE;
                state = 18;
            }
            break;
        }
        CyDelay(10);
    }
    PWM_Servo_Start();
    PWM_Servo_WriteCompare(DROP);
    PWM_Grip_WriteCompare(OPEN);
    CyDelay(300);
    PWM_Grip_Stop();
    PWM_Servo_Stop();
    
    return x;
}

void ObstacleAvoidance3()
{
    Rst_Write(MTRSTP);
    //PWM_Servo_Start();
    PWM_Grip_Start(); 
    PWM_Grip_WriteCompare(OPEN+6);
    //PWM_Servo_WriteCompare(LIFT);
    CyDelay(400);
    while ((distance_measured1 <= 21) || (distance_measured2 <= 21))
    {
        FrontSonar();
        Rst_Write(MTRSTP);
        CyDelay(100);
    }
    countdown = 0;
    PWM_Servo_Start();
    PWM_Servo_WriteCompare(DROP);
    PWM_Grip_WriteCompare(OPEN);
    CyDelay(300);
    PWM_Grip_Stop();
    PWM_Servo_Stop();
    
}



CY_ISR(BackStabR)
{
    while((AvoidL_Read() == 0) || (AvoidR_Read() == 0))
    {
        Rst_Write(MTRSTP);   
    }
}
CY_ISR(BackStabL)
{
    while((AvoidL_Read() == 0) || (AvoidR_Read() == 0))
    {
        Rst_Write(MTRSTP);   
    }
}


CY_ISR(Colour_isr)
{
    Colour_Hz_ReadStatusRegister();
    UART_1_PutString("Here\n");
    
    LED_Colour_Write(1);
    R = 0;
    B = 0;
    G = 0;
    int colour = 0;
    Colour_Rst_Write(0);
    CyDelay(1);
    for (int i = 1; i <= 18; i++)
    {
        Colour_Rst_Write(0);
        CyDelay(1);
        switch (colour)
        {
        case 0: S2_Write(0);
                S3_Write(0);
                R = Colour_Hz_ReadCounter();
                break;
        case 1: S2_Write(0);
                S3_Write(1);
                B = Colour_Hz_ReadCounter();
                break;
        case 2: S2_Write(1);
                S3_Write(1);
                G = Colour_Hz_ReadCounter();
                break;
                
        }
        Colour_Rst_Write(2);
        sprintf(buff, "%u, %u ,%u \n",R ,G, B);
        UART_1_PutString(buff);
        if ( i%6 == 0)
        {
          colour++;   
        }
        Colour_Rst_Write(0);
        CyDelay(40);
    }

    
    //Timer_ReadStatusRegister() ;
    //isr_1_ClearPending() ;
    
    if (R>B && R>G)
    {
        UART_1_PutString("Red\n");
        Red_Write(1);
        ColourType = 1;
    }   
    else if (B>R && B>G)
    {   
        UART_1_PutString("Blue\n");
        Blue_Write(1);
        ColourType = 2;
    }
    else if (G>R && G>B)
    {
        UART_1_PutString("Green\n");
        Green_Write(1);
        ColourType = 3;
    }
    
    CyDelay(10);
    
    Colour_Rst_Write(2) ;
    CyDelay(1) ;
    Colour_Rst_Write(0) ;
    LED_Colour_Write(0);
   
    //Rst_Write(0);
    
    CyDelay(10);
}


int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
      
    //UART
    UART_1_Start();
    
    //PWM 4 Servo
    PWM_Servo_Start();
    PWM_Grip_Start();
    Colour_Rst_Write(0);
    PWM_Grip_WriteCompare(OPEN);
    PWM_Servo_WriteCompare(LIFT);
    CyDelay(200);
    
    //PWM 4 Moving Wheels
    PWM_R_Start();
    PWM_L_Start();
    
    //Sonar
    Sonar_1_Start();
    Sonar_2_Start();
    Sonar_3_Start();
    Sonar_4_Start();
    isr_sonar_1_StartEx(Timer_ISR_Handler1); //Enable Interrupt
    isr_sonar_2_StartEx(Timer_ISR_Handler2); 
    isr_sonar_3_StartEx(Timer_ISR_Handler3);
    isr_sonar_4_StartEx(Timer_ISR_Handler4);
    
    //IR back interrupt
    
    //Colour Sensor
    
    LED_Colour_Write(0);
    Colour_Hz_Start();
    Colour_StartEx(Colour_isr) ;
    
    Colour_Rst_Write(2) ;
    CyDelay(1) ;
    Colour_Rst_Write(0) ;
    
    S0_Write(1);
    S1_Write(1);
    
    S2_Write(0);
    S3_Write(0);
    
    //Quadrature Decoder 4 Shaft Encoder
    QuadDec_R_Start();
    QuadDec_L_Start();
      
    //Interrupt
    //isr_1_StartEx(Timer_ISR_Handler); //Enable Interrupt 4 Sonar
    //isr_2_StartEx(IR_Handler);
    //ColourSense_StartEx(Colour); //Interrupts 4 Colour Sensing
    

    Rst_Write(RSTSE + MTRSTP); //Rst = 1 to avoid reseting QuadDec and 1 to kill PWM
    PWM_R_WriteCompare(COMPARE);//Set PWM Duty Cycle
    //PWM_L_WriteCompare(1000);//Set PWM Duty Cycle
    
    // Compass
    //Compass_Start();
    
//    Compass_MasterSendStart(HMC5883L, Compass_WRITE_XFER_MODE);
//    Compass_MasterWriteByte(0x09);
//    Compass_MasterWriteByte(0x1D);
//    Compass_MasterSendStop();
    
    

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    cmp = COMPARE;
    state = 0;
    revoCounter = 0;
    countdown = 0;
    
    int level = LEVEL1;
    int rightPos = 0;
    int leftPos = 0;
    int revoLimit = 7 ;
    int countdown2 = 0;
    
    
    
    //state = 200;
    
    
    CyDelay(300);
    PWM_Grip_Stop();
    PWM_Servo_Stop();
    
    for(;;)
    {
        /* Place your application code here. */

        switch (state)
        {            
            default:
            
                        
                        
    PWM_Grip_Start(); 
    PWM_Servo_Start(); 
    
    //CyDelay(500);
    //PWM_Servo_Stop();
    
    
    PWM_Grip_WriteCompare(GRIP);
    CyDelay(500);
    //PWM_Grip_Stop();
    //PWM_Servo_Start();
    
   
   
        PWM_Servo_WriteCompare(LIFT+1);
   
    
    CyDelay(500);
    PWM_Servo_Stop();
                    
                    Colour_Rst_Write(1);
                    CyDelay(1);
                    Colour_Rst_Write(2);
                    CyDelay(1);
                    Colour_Rst_Write(0);
                    
                    
                    
                    CyDelay(5000);
                    
                    Red_Write(0);
                    Blue_Write(0);
                    Green_Write(0);
            
                    break;
            
            case 0: //Warm Up. 0 - 1
                    FrontSonar();

                    SideSonar();

                    // Initial Position

                    if (countdown == 50)
                    {
                        if (distance_measured4 > distance_measured3)
                        {
                            leftPos = 143;
                            rightPos = 72;
                            Red_Write(1);
                        }
                        else if (distance_measured3 > distance_measured4)
                        {
                            rightPos = 143;   
                            leftPos = 72;
                            Blue_Write(1);
                        }
                        //sprintf(buff,"%d, %d, %d, %d\n",distance_measured1, distance_measured2, distance_measured3, distance_measured4);
                        //UART_1_PutString(buff);
                    }
                    countdown++;
                    
                    if ((LimitSWBackL_Read() == 1) && (LimitSWBackR_Read() == 1))
                    { 
                        PWM_Servo_Start();
                        PWM_Servo_WriteCompare(DROP); // 
                        PWM_Grip_Start();
                        PWM_Grip_WriteCompare(OPEN);
                        CyDelay(300);
                        Rst_Write(0);
                        state = 1;
                        ResetCounter();
                        Red_Write(0);
                        Blue_Write(0);
                        Rst_Write(MTRSTP);
                        PWM_Servo_Stop();
                     }       
                    
                    
                    
                    //sprintf(buff, "1) %d cm, 2) %d cm, 3) %d cm, 4) %d cm\n", distance_measured1, distance_measured2, distance_measured3, distance_measured4);
                    //UART_1_PutString(buff);
                   
                    break;       
                    
            case 1: // Move forward 1 - 3, Puck Detected 1 - 4
                    SpeedCtrl();
                    //Ultrasonic sensor   8 - level 1, 10 - level 2
                    FrontSonar();

                    SideSonar();
                    
                    countSE_L = QuadDec_L_GetCounter();
                    
                    if ((revoCounter > revoLimit) && (countSE_L > 5000) && (countSE_L < 9200))
                    {
                        //sprintf(buff, "1) %lf cm, 2) %lf cm, 3) %lf cm, 4) %lf cm\n", distance_measured1, distance_measured2, distance_measured3, distance_measured4);
                        
                        //sprintf(buff,"%d count\n",countdown);
                        //UART_1_PutString(buff);
                        ResetCounter();
                        if(rightPos < leftPos)
                        {
                            LeftTurn();
                        }
                        else
                        {
                            RightTurn();
                        }
                        cmp = COMPARE;
                        
                        if(limit == 8)
                        {
                            limit = 6;
                        }
                        state = 3;
                        
                    }
                    else
                    {       
                        if ((distance_measured1 <= 20) || (distance_measured2 <= 20))
                        {
                            countdown++;
                            if (countdown == 20)
                            {
                                Rst_Write(MTRSTP);
                                ObstacleAvoidance(leftPos,rightPos);
                            }
                        }
                        else
                        {
                            countdown = 0;   
                        }
                        
                        if (Infrared2_Read() == 0)
                        {
                            Rst_Write(0);
                            state = 4;
                            //revoCounter = 0;
                            
                            Rst_Write(MTRSTP);
                        }
                        
                    }
                    break;
                    
            case 2: 
                    SpeedCtrl();
                    
                    //Ultrasonic sensor 
                    FrontSonar();
                    
                    if ((revoCounter >= level) && (countSE_L > 5000) && (countSE_L < 9500))
                    {
                        //sprintf(buff, "1) %lf cm, 2) %lf cm, 3) %lf cm, 4) %lf cm\n", distance_measured1, distance_measured2, distance_measured3, distance_measured4);
                        
                        //sprintf(buff,"%d count\n",countdown);
                        //UART_1_PutString(buff);
                        
                        if(rightPos < leftPos)
                        {
                            countdown = 0;
                            revoCounter = 20;
                            switch(level)
                            {
                                case LEVEL1: LeftTurn(); break;
                                case LEVEL2: RightTurn(); break;
                                case LEVEL3: LeftTurn(); break;
                                case LEVEL4: LeftTurn(); break;
                                case LEVEL5: LeftTurn(); break;
                                case LEVEL6: LeftTurn(); break;
                            }
                            cmp = COMPARE;
                            state = 18;
                        }
                        else
                        {
                            countdown = 0;
                            revoCounter = 20;
                            switch(level)
                            {
                                case LEVEL1: RightTurn(); break;
                                case LEVEL2: LeftTurn(); break;
                                case LEVEL3: RightTurn(); break;
                                case LEVEL4: RightTurn(); break;
                                case LEVEL5: RightTurn(); break;
                                case LEVEL6: RightTurn(); break;
                            }
                            cmp = COMPARE;
                            state = 18;
                        }
                        PWM_Servo_Start();
                        PWM_Grip_Start(); 
                        PWM_Servo_WriteCompare(DROP);
                        PWM_Grip_WriteCompare(OPEN);
                        CyDelay(300);
                        PWM_Grip_Stop();
                        PWM_Servo_Stop();
                    }
                    else
                    {
                        if (Infrared2_Read() == 0)
                        {
                         
                            Rst_Write(0);
                            state = 4;
                            //revoCounter = 0;
//                            if (directional == 3)
//                            {
//                                distance = 8;   
//                            }
//                            else 
//                            {
//                                distance = 0;
//                            }
//                           
                            Rst_Write(MTRSTP);
                        }
                        
                        if ((distance_measured1 <= 20) || (distance_measured2 <= 20))
                        {
                            countdown++;
                            if (countdown == 20)
                            {
                                Rst_Write(MTRSTP);
                                hold = ObstacleAvoidance2(rightPos, leftPos, level);
                            }   
                        }
                        else
                        {
                            countdown = 0;   
                        }
                    }
                    //sprintf(buff, "1) %lf cm, 2) %lf cm, 3) %lf cm, 4) %lf cm\n", distance_measured1, distance_measured2, distance_measured3, distance_measured4);
                    //UART_1_PutString(buff);
                    break;
                    
            case 3: 
                    SpeedCtrl();                
                    
                    FrontSonar();

                    SideSonar();
                    
                    if (((distance_measured1 <= 19) && (distance_measured2 <= 19)) && (revoCounter > limit))
                    {
                        //sprintf(buff, "1) %lf cm, 2) %lf cm, 3) %lf cm, 4) %lf cm\n", distance_measured1, distance_measured2, distance_measured3, distance_measured4);
                        countdown++;
                        //sprintf(buff,"%d count\n",countdown);
                        //UART_1_PutString(buff);
                        
                        if((countdown >= 5) && (directional == 2))
                        {
                            switch(level)
                            {
                                case LEVEL1: level = LEVEL2; LeftTurn();  break;
                                case LEVEL2: level = LEVEL3; LeftTurn();  break;
                                case LEVEL3: level = LEVEL4; RightTurn(); break;
                                case LEVEL4: level = LEVEL5; LeftTurn();  break;
                                case LEVEL5: level = LEVEL6; RightTurn(); break;
                                case LEVEL6: level = LEVEL1; LeftTurn(); break;
                                
                            }
                            
                            ResetCounter();
                            limit = 8;
                            cmp = COMPARE;
                            state = 2;
                            
                        }
                        else if((countdown >= 5) && (directional == 4))
                        {
                            switch(level)
                            {
                                case LEVEL1: level = LEVEL2; RightTurn();  break;
                                case LEVEL2: level = LEVEL3; RightTurn();  break;
                                case LEVEL3: level = LEVEL4; LeftTurn(); break;
                                case LEVEL4: level = LEVEL5; RightTurn(); break;
                                case LEVEL5: level = LEVEL6; LeftTurn(); break;
                                case LEVEL6: level = LEVEL1; RightTurn(); break;
                                
                            }
                            limit = 8;
                            ResetCounter();
                            cmp = COMPARE;
                            state = 2;
                            
                        }
                    }
                    else
                    {
                        if (Infrared2_Read() == 0)
                        {
                            Rst_Write(0);
                            state = 4;
                            //revoCounter = 0;
//                            distance = revoCounter - 2;
                            Rst_Write(MTRSTP);   
                        }
                        
                        if ((distance_measured1 <= 19) || (distance_measured2 <= 19))
                        {
                            countdown++;
                            if (countdown == 20)
                            {   
                                ObstacleAvoidance3();   
                            }
                        }
                        else
                        {
                            countdown = 0;
                        }
                    }
                    //sprintf(buff, "1) %lf cm, 2) %lf cm, 3) %lf cm, 4) %lf cm\n", distance_measured1, distance_measured2, distance_measured3, distance_measured4);   
                    //sprintf(buff,"%d\n",level);
                    //UART_1_PutString(buff);
                    
                    break;
                     
            
                    
            case 4:  // Performing the Task
                    Reverse();   
                    AvoidL_isr_StartEx(BackStabL);
                    AvoidR_isr_StartEx(BackStabR);
                    if ((countSE_L  <= -5065) && (countSE_L > -9000))
                    {
                        AvoidL_isr_Stop();
                        AvoidR_isr_Stop();
                        PuckLifting();
                        state = 5;
                        
                        
                    }
                    
                    break;
                    
            case 5: 
                    Colour_Rst_Write(1);
                    CyDelay(1);
                    Colour_Rst_Write(2);
                    CyDelay(1);
                    Colour_Rst_Write(0); 
                    
                    if (Infrared3_Read() == 1)
                    {
                        if((level == LEVEL1) && (directional == 1))
                        {
                            state = 1;
                        }
                        else if ((directional == 1) || (directional == 3))
                        {
                            state = 2;
                        }
                        else if (countdown == 150)
                        {
                            state = 15;
                        }
                        else
                        {
                            state = 3;
                        }
                        
                        PuckLanding();
                    }
                    else
                    {
                        if (countdown == 150)
                        {
                            revoCounter = 80;
                            state = 18;
                        }
                        else
                        {
                            state = 6;
                        }
                    }
                    
                    break;
            
            case 6: 
                    if (directional == 2)
                    {
                     
                        RightTurn();
                        
                        state = 7;
                        
                    }
                    else if (directional == 4)
                    {
                     
                        LeftTurn();
                        
                        state = 7;
                        
                    }
                    else if (directional == 1)
                    {
                        
                        LeftTurn();
                        LeftTurn();
                        
                        state = 7;
                        
                    }
                    else 
                    {
                        
                        state = 7;
                    }
                    
                    
                    
                    break;
                    
            case 7: 
                    FrontSonar();
                    
                    
                    if ((distance_measured1 <= 52) && (distance_measured2 <= 52) && (countdown <= 10))
                    {
                        countdown++;
                        if (countdown == 11)
                        {
                            if (rightPos > leftPos)
                            {
                                RightTurn();   
                            }
                            else
                            {
                                LeftTurn();   
                            }
                            countdown2 = 0;
                            ResetCounter();
                            state = 8;
                        }
                    }
                    else 
                    {
                        if((distance_measured1 < 11) || (distance_measured2 < 11))
                        {
                            countdown2++;
                            if(countdown2 >= 10)
                            {
                                Rst_Write(RSTSE + MTRSTP);   
                            }
                        }
                        else
                        {   
                            countdown2 = 0;
                            SpeedCtrl();
                        }
                        countdown = 0;
                    }
                    break;
                    
            case 8:
                    FrontSonar();
                    
                    if ((distance_measured1 <= 20) && (distance_measured2 <= 20) && (countdown <= 10))
                    {
                        countdown++;
                        if (countdown == 11)
                        {
                            LeftTurn();
                            LeftTurn();
                            revoCounter = 80;
                            state = 18;
                            countdown2 = 0;
                        }
                    }
                    else
                    {
                        if ((distance_measured1 < 11) || (distance_measured2 < 11))
                        {
                            countdown2++;
                            if (countdown2 >= 10)
                            {
                                Rst_Write(MTRSTP+RSTSE);   
                            }
                        }
                        else
                        {
                            countdown2 = 0;
                            SpeedCtrl();
                        }
                        countdown = 0;
                    }
                    
                    
                    break;
                    
            case 9:
                    
                    FrontSonar();

                    if (rightPos < leftPos)
                    {
                        
                        SpeedCtrl();
                        if (revoCounter == 2)
                        {
                            LeftTurn();

                            ResetCounter();
                            state = 10;
                            
                        }  
                        
                    }
                    else
                    {
                                                
                        SpeedCtrl();
                        
                        if (revoCounter == 2)
                        {
                            RightTurn();
    
                            ResetCounter();
                            state = 10;
                            
                        }  
                    
                    }
                    
                    //sprintf(buff," %d,,,,,%d, %d\n",distance_measured3, distance_measured4, countdown);
                    //UART_1_PutString(buff);
                    
                    break;
                    
            case 10: 
                    FrontSonar();
                        
                    
                    
                    if ((distance_measured1 > 39) && (distance_measured2 > 39) && (countdown < 30))
                    {
                        countdown++;
                        if (countdown == 30)
                        {
                            state = 17;   
                        }
                        
                    }
                    
                    if ((distance_measured1 <= 39) && (distance_measured2 <= 39) && (countdown < 20))
                    {
                        countdown++;
                        if (countdown == 20)
                        {
                            countdown = 50;
                            state = 17;
                        }
                       
                    }
                    
                    

                    //sprintf(buff,"%d, %d, %d\n",distance_measured1, distance_measured2, countdown);
                    //UART_1_PutString(buff);
                    break;
                    
            case 11:
                    
                    PuckLanding();
                    state = 12;
                    break;
                    
            case 12: 
                    SpeedCtrl();
                    
                    if (Infrared_Read() == 0)
                    {
                        //Rst_Write(MTRSTP);
                        //CyDelay(100);
                        Colour_Rst_Write(SERVO);
                        PWM_Servo_Start();
                        
                        
                        PWM_Servo_WriteCompare(RETRACT);
                        
                        CyDelay(50);
                        //CyDelay(300);
                        
                        if (ColourType == 3)
                        {
                            Rst_Write(MTRSTP);
                            PWM_Servo_WriteCompare(FLICK+1);   
                        }
                        else
                        {
                            CyDelay(20);
                            PWM_Servo_WriteCompare(FLICK);
                        }
                        
                        CyDelay(300);
                        
                        Rst_Write(MTRSTP);
                        
                        PWM_Servo_WriteCompare(RETRACT);
                        CyDelay(300);
                        
                        PWM_Servo_Stop();
                        Colour_Rst_Write(0);
                        
                        countdown++;
                        
                        revoCounter = 0;
                        
                        state = 13;
                    }
                    else 
                    {
                        if (revoCounter > 2)
                        {
                            ResetCounter();
                            state = 14;   
                        }
                        
                    }
                    
                    break;
                    
            case 13: 
                    if (ColourType == 1)
                    {
                        state = 14;
                        if (countdown < 2)
                        {
                            state = 12;   
                        }
                        
                    }
                    else if(ColourType == 2)
                    {
                        state = 14;
                    }
                    else if(ColourType == 3)
                    {
                        state = 14;
                    }
                    
                    if (state == 14)
                    {
                        ResetCounter();   
                    }
                    
                    
                    break;
            
            case 14: 
                    Reverse();
                    AvoidL_isr_StartEx(BackStabL);
                    AvoidR_isr_StartEx(BackStabR);
                    if (ColourType == 1)
                    {
                        if (revoCounter > 2)
                        {
                            Rst_Write(MTRSTP);
                            countdown++;
                        }
                        
                    }
                    else if (ColourType == 2)
                    {
                        if (revoCounter > 2)
                        {
                            countdown++;
                        }
                    }
                    else if (ColourType == 3)
                    {
                        if (revoCounter > 1)
                        {
                            countdown++;
                        }
                    }
                    
                    if (countdown > 0)
                    {
                        if (rightPos > leftPos)
                        {
                            LeftTurn();
                            revoCounter = 140;
                            state = 18;
                        }
                        else 
                        {
                            RightTurn();
                            revoCounter = 140;
                            state = 18;
                        }
                        AvoidL_isr_Stop();
                        AvoidR_isr_Stop();
                    }

                    break;
                    
            case 15:
                    FrontSonar();
                    
                    level = LEVEL1;
                    if (revoCounter > 2)
                    {
                        if (rightPos > leftPos)
                        {
                            LeftTurn();
                            state = 1;
                        }
                        else
                        {
                            RightTurn();
                            state = 1;
                        }
                        revoCounter += 1;
                    }
                    
                    if ((distance_measured1 <= 10) || (distance_measured2 <= 10))
                    {
                        countdown++;   
                        
                        if(countdown == 5)
                        {
                            
                            Rst_Write(MTRSTP);
                            countdown = 0;
                        }
                    }
                    else
                    {
                        countdown = 0; 
                        
                        SpeedCtrl();
                    }
                    if(revoCounter > 0)
                    {
                         if (Infrared2_Read() == 0)
                        {
                            
                            Rst_Write(0);
                            state = 4;
                            countdown = 150;
    //                        distance = 0;
                            Rst_Write(MTRSTP);
                        }
                    }
                    break;
                    
            case 16:
                    
                    
                    
                    break;
                    
            case 17: 
                    FrontSonar();
                    
                    if ((countdown >= 30) && (countdown < 49))
                    {
                        if ((distance_measured1 >= 38) && (distance_measured2 >= 38))
                        {
                            SpeedCtrl();   
                        }
                        else if ((distance_measured1 > 35) & (distance_measured2 > 35))
                        {
                            countdown++;
                            Rst_Write(MTRSTP);
                            if (countdown == 45)
                            {
                                ResetCounter();
                                state = 11; 
                            }
                        }
                    }
                    else if(countdown >= 50) 
                    {
                        if ((distance_measured1 < 38) && (distance_measured2 < 38))
                        {
                            Reverse();
                        }
                        else
                        {
                            countdown++;
                            Rst_Write(MTRSTP);
                            if (countdown == 65)
                            {
                                ResetCounter();
                                state = 11; 
                            }
                            
                        }
                    }
                    
                    break;
            
            
            case 18:
                    AvoidL_isr_StartEx(BackStabL);
                    AvoidR_isr_StartEx(BackStabR);
                    
                    
                    if((LimitSWBackL_Read() == 1) && (LimitSWBackR_Read() == 1))
                    {
                        Rst_Write(MTRSTP);
                        
                        state = 19;
                        
                    }
                    else if ((LimitSWBackL_Read() == 1) && (LimitSWBackR_Read() == 0))
                    {
                        Rst_Write(4);
                    }
                    else if ((LimitSWBackR_Read() == 1) && (LimitSWBackL_Read() == 0))
                    {
                        Rst_Write(2);
                    }
                    else 
                    {
                        Rst_Write(0);
                    }
                    break;
                    
            case 19: 
                    
                    AvoidR_isr_Stop();
                    AvoidL_isr_Stop();
                    
                    state = 2;
                        
                    if (revoCounter == 20)
                    {
                        state = 3;
                    }
                    else if (revoCounter == 80)
                    {
                        state = 9;
                    }
                    else if (revoCounter == 140)
                    {
                        ColourType = 0;
                        
                        PWM_Servo_Start();
                        PWM_Grip_Start();
                        PWM_Grip_WriteCompare(OPEN);
                        PWM_Servo_WriteCompare(DROP);
                        CyDelay(300);
                        
                        state = 15;
                    }
                    
                    ResetCounter();
                    

 
                    
                    break;
                      
        }
        
        
        //sprintf(buff, "L = %d , R = %d, compare = %d, %d\n", countSE_L, countSE_R, cmp, state);
        //UART_1_PutString(buff);

        //sprintf(buff, "1) %lf cm, 2) %lf cm, 3) %lf cm, 4) %lf cm\n", distance_measured1, distance_measured2, distance_measured3, distance_measured4);
        //UART_1_PutString(buff);
        CyDelay(10);
    }
        
    CyDelay(10);   
    }


/* [] END OF FILE */