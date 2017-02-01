/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.c
 * Author: HP
 * Kinesin Motor 2-Head (Other Head walking on Detachment)V1.1 Changes: Use of Arrays and Function for all Motors 19/1/16
 * Created on February 1, 2017, 9:18 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include<math.h>

/*
 * This is the complete model of kinesin-5 with 2 heads on track and cargo MT. 
 */
int main(int argc, char** argv) {
    FILE *fp, *ft, *fz;     //Declaring File Pointers
    fp= fopen("velocity_10Ksliding_noexclusion_random.dat","w"); ft= fopen("Test_MultiKMotor.dat","w");     fz= fopen("Test_KWarmTime.dat","w");    //Creating Files associated with those Pointers
    int N; printf("Please enter the No. of motors (either 1 or 2)"); scanf("%d",&N);
    //Declaring Variables           //TRY USING TYPECASTING (double)((int)VALUE) for double precision values
    double stepsize=8.000000, restlength=90.000000, unloadvelocity=24.000000, config=0.000000, Detachratezeroload=1.000000, rateswitchdiff=0.1, ratereengage=10, MPositionTrack=0.000000, lengthcargo=1000.000000, PositionCargo=0.000000, stiffness=0.320000, detachforce=4.000000, stallforce=1.500000, motortail_initial[N],motortail_tentative[N],motortail_final[N], Tail_Diffuse[N], motorhead_initial[N], motorhead_tentative[N], motorhead_final[N], Head_Diffuse[N], motor_stretch[N], ForceExt=0.000000, Time=0.000000, Random=0.000000, Random1=0.000000, x= 0.000000, netx=0.000000, v= 0.000000, xav= 0.000000, vav= 0.000000, e=0.000000, tstep=0.000500, probabstep[N], probabdetach[N], probabstationary[N], probabretach[N], probabswitchdiff[N], probabengage[N], probabdiffuse1[N], probabdiffuse2[N], Fnet=0.000000, temp=0.000000, maxconfig=50000.000000, attachcondition[N], MotorAttached=0.000000, retachrate=5.000000, fmotor[N], fmotors=0.000000, Maximum=-restlength, Minimum=restlength, Maxtail=0.000000, Mintail=0.000000, AvgMotorAttached=N, TentativeAttachments=0.000000, FAvgMotorAttached=0.000000,netxstart=0.000000,netxend=0.000000,warmtime=1.000000, maxtime=2.000000;
    int NumMotor=0,status=0,relconfig=0,Temper=295,Boltzman=.0138;
    for(ForceExt=0.000000;ForceExt<=20.000000/*4.000000*N*/; ForceExt= ForceExt+0.050000) //Force Loop
    {
        xav=0.000000,vav=0.000000,FAvgMotorAttached=0.000000,relconfig=0;       //fprintf(ft,"eForce:%lf\n",ForceExt);
        for (config=1.000000;config<=maxconfig;config++)        //Configuration Loop
        {//***************************************************************************************************//
            MotorAttached=0.000000; //fprintf(ft,"Config:%lf\n",config);
        /*  motortail[NumMotor]=((int)((double)rand()/(RAND_MAX)*(lengthcargo)+1.000000)-1.000000)*stepsize;  // Motor tail position on Cargo Microtubule on Discrete Site  ****
            motorhead_initial[NumMotor]=((int)((double)rand()/(RAND_MAX)*(2*((int)(restlength/stepsize))+1)-((int)(restlength/stepsize))+1.000000)*stepsize; // Motor head position on Track Microtubule wrt Cargo  ***/
            for(NumMotor=0;NumMotor<=N-1;NumMotor++)
            {
                motortail_initial[NumMotor]=(int)((double)rand()/(RAND_MAX)*lengthcargo);  // Motor tail position on Cargo Microtubule Random Arrangement
                motor_stretch[NumMotor]=(int)((double)rand()/(RAND_MAX)*(2.000000*restlength+1.000000)); // Motor head position on Track Microtubule wrt Cargo
                if(motor_stretch[NumMotor]>restlength)  //To bind the motor within +-60nm restlength to the tr++ack wrt cargo
                {
                        motor_stretch[NumMotor]=motor_stretch[NumMotor]-(2.000000*restlength+1.000000);
                }
                motorhead_initial[NumMotor]=motortail_initial[NumMotor]+motor_stretch[NumMotor];
                attachcondition[NumMotor]=1;    // Motor is attached
                MotorAttached++;        //fprintf(ft,"Tail:%lf\t Head:%lf\t AttachCon:%lf\t TotalAttach:%lf\n",motortail[NumMotor], motorhead_initial[NumMotor],attachcondition[NumMotor], MotorAttached);//Total attached motors
            } // Motor loop ends
            fmotors=0.000000;
            Fnet= fmotors-ForceExt; //fprintf(ft,"Fnetperturb: %lf\n",Fnet);//Intial Force (Change in Load & Configuration)
            if (Fnet== 0)   //Use the concept of Potential stretch (Find P.stretch, Find min p.stretch of forward and backward motors & average
            {
                for (NumMotor=0;NumMotor<=N-1;NumMotor++)
                {
                    if (motor_stretch[NumMotor]>=0.000000)
                    {
                        if(motor_stretch[NumMotor]-restlength> Maximum)         //Motor strech is positive but less than restlength (ms-rl is negative)
                        {
                                Maximum=motor_stretch[NumMotor]-restlength;
                        }
                    }
                    else if (motor_stretch[NumMotor]< 0.000000)
                    {
                        if (motor_stretch[NumMotor]+restlength< Minimum)                //Motor strech is negative but less than restlength (ms+rl is positive)
                        {
                                Minimum=motor_stretch[NumMotor]+restlength;
                        }
                    }
                }
                PositionCargo=(Maximum+Minimum)/2;      //fprintf(ft,"Max:%lf\t Min:%lf\t Cargo:%lf\n",Maximum, Minimum, PositionCargo);//Cargo COM (will be the average of Most distant Motors
                for(NumMotor=0;NumMotor<=N-1;NumMotor++)        /***************REVIEW THIS SEGMENT**DONE************/
                {
                    motortail_initial[NumMotor]= motortail_initial[NumMotor]+ (PositionCargo);      //Absolute position
                    motor_stretch[NumMotor]=motorhead_initial[NumMotor]-motortail_initial[NumMotor];        // Updated Motor head position on Track Microtubule wrt Cargo
                }
            }
            // PositionCargo=0.000000;      //CHECK IT****  //fprintf(ft,"Headof1:%lf\t Cargo:%lf\n",motorhead_initial[0], PositionCargo); //New Cargo COM referenced as 0  //fprintf(ft,"Headof1:%lf\t Headof2:%lf\t Headof3:%lf\t Cargo:%lf\n",motorhead_initial[0], motorhead_initial[1], motorhead_initial[2], PositionCargo);  */
    /*                      for(NumMotor=0;NumMotor<=N-1;NumMotor++)
            {
                    if (motor_stretch[NumMotor]>restlength) //Motor is stretched backward, Motor is experiencing load and will try to pull cargo forward
                    {
                            fmotor[NumMotor]= stiffness * (motor_stretch[NumMotor]-restlength);     //Force by Motor
                            fmotors=fmotors+fmotor[NumMotor];
                    }
                    else if (motor_stretch[NumMotor]<-restlength)   //Motor is stretched Forward, Motor is experiencing load and will try to pull cargo forward
                    {
                            fmotor[NumMotor]= stiffness* (motor_stretch[NumMotor]+restlength);      //Force by Motor
                            fmotors=fmotors+fmotor[NumMotor];
                    }
                    else
                    {
                            fmotor[NumMotor]=0.000000;
                            fmotors=fmotors+fmotor[NumMotor];
                    }
            }
            Fnet= fmotors-ForceExt; //Check Fnet            */
            if (Fnet!= 0)
            {
                while(fabs(Fnet)> 0.00000005) // For Attainment of Equibrillium
                {
                    x=Fnet/(MotorAttached*stiffness);       //New Equibrillium Position                                             netx=netx+x;
                    PositionCargo= PositionCargo+x; //fprintf(ft,"NewPos: %lf\t CargoProgression: %lf\t CarPos: %lf\n",x,netx,PositionCargo);
                    for(NumMotor=0;NumMotor<=N-1;NumMotor++)
                    {
                        motortail_initial[NumMotor]=motortail_initial[NumMotor]+x;      //Motortail position Update
                        motor_stretch[NumMotor]= motor_stretch[NumMotor]-x;     //PositionCargo;        // Updated Motor stretch
                    }       //fprintf(ft,"NewHeadMot1: %lf\t NewHeadMot2: %lf\t NewHeadMot3: %lf\n",motorhead_initial[0],motorhead_initial[1],motorhead_initial[2]);//Updated Cargo Positions
                    //PositionCargo=0.000000; //New Cargo COM referenced as 0 (Relative)
                    fmotors=0.000000;       //fprintf(ft,"NewPosCarRef: %lf\t fmot: %lf\n",PositionCargo,fmotors);//To calculate Force by the Motor
                    for(NumMotor=0;NumMotor<=N-1;NumMotor++)        //For New Fnet Calculations
                    {
                        if (attachcondition[NumMotor]==1.000000)
                        {
                            if (motor_stretch[NumMotor]>restlength) //Motor is stretched backward, Motor is experiencing load
                            {
                                fmotor[NumMotor]= stiffness * (motor_stretch[NumMotor]-restlength);     //Force by Motor
                                fmotors=fmotors+fmotor[NumMotor];
                            }
                            else if (motor_stretch[NumMotor]<-restlength)   //Motor is stretched Forward, Motor is experiencing load
                            {
                                fmotor[NumMotor]= stiffness* (motor_stretch[NumMotor]+restlength);      //Force by Motor
                                fmotors=fmotors+fmotor[NumMotor];
                            }
                            else
                            {
                                fmotor[NumMotor]=0.000000;
                                fmotors=fmotors+fmotor[NumMotor];
                            }       //fprintf(ft,"Fmot: %lf\t Fmotall: %lf\n",fmotor[NumMotor],fmotors);
                        }
                    }       //// Force By Motor Calculation Complete
                    Fnet= fmotors-ForceExt; //fprintf(ft,"Fnetiteration: %lf\n",Fnet);//Fnet after Current Iterative step
                }
            }
            for (NumMotor=0;NumMotor<=N-1;NumMotor++)       //Copying the Initial Postion of Motor
            {
                motorhead_tentative[NumMotor]= motorhead_initial[NumMotor];
                motorhead_final[NumMotor]= motorhead_initial[NumMotor];
                motortail_tentative[NumMotor]= motortail_initial[NumMotor];
                motortail_final[NumMotor]= motortail_initial[NumMotor];
            }
            // Initialization Complete
            //***************************************************************************************************//
            x=0.000000, netx=0.000000, v=0.000000, netxstart=0.000000, netxend=0.000000, status=0.000000, AvgMotorAttached=N; //printf("Config:%lf\n",config);
            for (Time=tstep;Time<=maxtime;Time=Time+tstep)//Time Loop
            {       //fprintf(ft,"Time:%lf\n",Time);        //printf("Time:%lf\n",Time);
                for(NumMotor=0;NumMotor<=N-1;NumMotor++)        //Motor Loop for Event Calculation
                {       //fprintf(ft,"MotorNo.:%d\n",NumMotor);
                    if (attachcondition[NumMotor]==1.000000)        // For Attached Motors, Probability Calculation
                    {
                        if (motor_stretch[NumMotor]>restlength) //Motor is stretched forward, Load experienced by motor
                        {
                                fmotor[NumMotor]= stiffness* (motor_stretch[NumMotor]-restlength);
                        }
                        else if(motor_stretch[NumMotor]<-restlength)    //Motor is stretched backward, Load experienced by motor
                        {
                                fmotor[NumMotor]= stiffness* (motor_stretch[NumMotor]+restlength);
                        }
                        else    //Motor is compressed:  No force load experienced by motor
                        {
                                fmotor[NumMotor]=0.000000;
                        }       //fprintf(ft,"fmotor:%lf\n",fmotor[NumMotor]);
                        if((fmotor[NumMotor])< stallforce)      // fabs is used instead of abs(output is integer value) for float type data
                        {
                                e=Detachratezeroload*exp(fabs(fmotor[NumMotor])*1/(Boltzman*Temper));
                                probabdetach[NumMotor]=e*tstep;
                                if (fmotor[NumMotor]<=1.000000) //(fmotor[NumMotor]>=0.000000)
                                {
                                        v=unloadvelocity;

                                }
                                else
                                {
                                        v=unloadvelocity*(1-(fabs(fmotor[NumMotor])/stallforce));
                                }
                                probabstep[NumMotor]=v*tstep/stepsize;
                        } //Force Less than stallforce
                        else
                        {
                                probabdetach[NumMotor]=tstep*Detachratezeroload*exp(fabs(fmotor[NumMotor])*1/(Boltzman*Temper)); //     /(0.254000*(1.000000-1.000000*exp(-fabs(fmotor[NumMotor])/1.966460)));  //Detachment rate(e)*tstep
                                probabstep[NumMotor]=unloadvelocity*(1-(fabs(fmotor[NumMotor])/stallforce))*tstep/stepsize;//0.000000;  Back Step obseved here
                                probabswitchdiff[NumMotor]=rateswitchdiff*tstep;
                        }       //Force Greater than stallforce
                        probabstationary[NumMotor]=1-probabdetach[NumMotor]-probabstep[NumMotor];
                        probabretach[NumMotor]=0.000000;
                        //fprintf(ft,"Detrate:%lf\t PDetach:%lf\t Vel:%lf\t PStep:%lf\t Pstation:%lf\n",e,probabdetach[NumMotor],v,probabstep[NumMotor],probabstationary[NumMotor]);
                    }
                    else    //For Detached Motors, Probablity Calculations
                    {
                        probabretach[NumMotor]=retachrate*tstep;
                        probabstep[NumMotor]=0.000000;
                        probabdetach[NumMotor]=0.000000;
                        probabstationary[NumMotor]=0.000000;    //fprintf(ft,"PRetach:%lf\n",probabretach[NumMotor]);
                    }
                    Random=(double)rand()/(RAND_MAX);       //fprintf(ft,"Random:%lf\n",Random);//Probablity of occurance of a random event
                    Random1=(double)rand()/(RAND_MAX);      //Probablity of occurance of a random event for motor tail
                    if (attachcondition[NumMotor]==1.000000)        // For Attached Motors, Occurance of an Event
                    //*********************DO WE TWO FLAGS FOR HEAD & TAIL OR CAN 1 DO THE TRICK!!*****************
                    {
                        if(Random<=probabstep[NumMotor])        //Stepping occurs
                        {
                            motorhead_tentative[NumMotor]=motorhead_initial[NumMotor]+stepsize;
                        }
                        else if (Random>probabstep[NumMotor] && Random<= (probabstep[NumMotor]+probabdetach[NumMotor])) //Motor Detaches
                        {
                            attachcondition[NumMotor]= 0.000000;    // Motor is detached
                            motorhead_initial[NumMotor]=motortail_initial[NumMotor];        //Assumed Default Position as it has no effect on Calculations                  ******************CHECK IT**************
                            motor_stretch[NumMotor]=0.000000;               //Assumed Default Position as it has no effect on Calculations
                            fmotor[NumMotor]=0.000000;
                            MotorAttached--;        //Total attached motors
                            if (MotorAttached==0)
                            {
                                break;  //fprintf(ft,"Time Exit: %lf\t MAttached:%lf\n",Time,MotorAttached);    //Break Motor Loop
                            }
                        }
                        else    //Motor remains Stationary
                        {
                            //Nothing Happens :P
                        }       //fprintf(ft,"Head:%lf\t AttCon:%lf\t Fmot:%lf\t MAttached:%lf\n",motorhead_initial[NumMotor],attachcondition[NumMotor],fmotor[NumMotor],MotorAttached);
                        if(Random1<=probabstep[NumMotor])       //Stepping occurs
                        {
                            motortail_tentative[NumMotor]=motortail_initial[NumMotor]-stepsize;     //CHECK WITH CARGOLENGTH
                            if (motortail_tentative[NumMotor]-PositionCargo<0)
                            {
                                //Stepping not Possible or Detachment may occur
                                attachcondition[NumMotor]= -1.000000;   // Motor is detached
                                motortail_initial[NumMotor]=motorhead_initial[NumMotor];        //Assumed Default Position as it has no effect on Calculations  ******************CHECK IT**************
                                motor_stretch[NumMotor]=0.000000;               //Assumed Default Position as it has no effect on Calculations
                                fmotor[NumMotor]=0.000000;
                                MotorAttached--;        //Total attached motors
                                if (MotorAttached==0)
                                {
                                    break;  //fprintf(ft,"Time Exit: %lf\t MAttached:%lf\n",Time,MotorAttached);    //Break Motor Loop
                                }
                            }
                        }
                        else if (Random1>probabstep[NumMotor] && Random1<= (probabstep[NumMotor]+probabdetach[NumMotor]))       //Motor Detaches
                        {
                            attachcondition[NumMotor]= -1.000000;   // Motor is detached
                            motortail_initial[NumMotor]=motorhead_initial[NumMotor];        //Assumed Default Position as it has no effect on Calculations  ******************CHECK IT**************
                            motor_stretch[NumMotor]=0.000000;               //Assumed Default Position as it has no effect on Calculations
                            fmotor[NumMotor]=0.000000;
                            MotorAttached--;        //Total attached motors
                            if (MotorAttached==0)
                            {
                                break;  //fprintf(ft,"Time Exit: %lf\t MAttached:%lf\n",Time,MotorAttached);    //Break Motor Loop
                            }
                        }
                        else    //Motor remains Stationary
                        {
                            //Nothing Happens :P
                        }       //fprintf(ft,"Head:%lf\t AttCon:%lf\t Fmot:%lf\t MAttached:%lf\n",motorhead_initial[NumMotor],attachcondition[NumMotor],fmotor[NumMotor],MotorAttached);
                    }
                    else    //For Unattached Motors, Occurance of an Event
                    {
                        if (attachcondition[NumMotor]== 0.000000)
                        {
                            if(Random<=probabretach[NumMotor])      //MODIFICATION REQ.
                            {
                                motor_stretch[NumMotor]=(int)((double)rand()/(RAND_MAX)*(2.000000*restlength+1.000000)); // Motor head position on Track Microtubule wrt Cargo
                                if(motor_stretch[NumMotor]>restlength)  //To bind the motor within +-60nm restlength to the track wrt cargo
                                {
                                    motor_stretch[NumMotor]=motor_stretch[NumMotor]-(2.000000*restlength+1.000000);
                                }
                                motorhead_tentative[NumMotor]=motortail_initial[NumMotor]+motor_stretch[NumMotor];      //OK if coordiantes are ABSOLUTE..
                                attachcondition[NumMotor]=1;    // Motor is attached
                                MotorAttached++;        /*motorhead_tentative[NumMotor]=(int)((double)rand()/(RAND_MAX)*(2.000000*restlength+1.000000)); // Motor head position on Track Microtubule wrt Cargo
                                if(motorhead_tentative[NumMotor]>restlength)    //To bind the motor within +-60nm restlength to the track wrt cargo
                                {
                                    motorhead_tentative[NumMotor]=motortail_initial[NumMotor]+motorhead_tentative[NumMotor]-(2.000000*restlength+1.000000);
                                }
                                attachcondition[NumMotor]=1.000000;     // Motor is attached
                                MotorAttached;  *///fprintf(ft,"Head: %lf\t AttCon:%lf\t MAttached:%lf\n",motorhead_initial[NumMotor],attachcondition[NumMotor],MotorAttached);//Total attached motors
                            }
                            if(Random1<=probabstep[NumMotor])       //Stepping occurs               **********THINK ABOUT UPDATION***********
                            {
                                motortail_tentative[NumMotor]=motortail_initial[NumMotor]-stepsize;     //CHECK WITH CARGOLENGTH
                                if (motortail_tentative[NumMotor]-PositionCargo<0)
                                {
                                    //Stepping not Possible or Detachment may occur
                                    attachcondition[NumMotor]= -1.000000;   // Motor is detached
                                    motortail_initial[NumMotor]=motorhead_initial[NumMotor];        //Assumed Default Position as it has no effect on Calculations  ******************CHECK IT**************
                                    motor_stretch[NumMotor]=0.000000;               //Assumed Default Position as it has no effect on Calculations
                                    fmotor[NumMotor]=0.000000;
                                    MotorAttached--;        //Total attached motors
                                    if (MotorAttached==0)
                                    {
                                        break;  //fprintf(ft,"Time Exit: %lf\t MAttached:%lf\n",Time,MotorAttached);    //Break Motor Loop
                                    }
                                }
                            }
                            else if (Random1>probabstep[NumMotor] && Random1<= (probabstep[NumMotor]+probabdetach[NumMotor]))       //Motor Detaches
                            {
                                attachcondition[NumMotor]= -1.000000;   // Motor is detached
                                motortail_initial[NumMotor]=motorhead_initial[NumMotor];        //Assumed Default Position as it has no effect on Calculations  ******************CHECK IT**************
                                motor_stretch[NumMotor]=0.000000;               //Assumed Default Position as it has no effect on Calculations
                                fmotor[NumMotor]=0.000000;
                                MotorAttached--;        //Total attached motors
                                if (MotorAttached==0)
                                {
                                    break;  //fprintf(ft,"Time Exit: %lf\t MAttached:%lf\n",Time,MotorAttached);    //Break Motor Loop
                                }
                            }
                            else    //Motor remains Stationary
                            {
                                //Nothing Happens :P
                            }       //fprintf(ft,"Head:%lf\t AttCon:%lf\t Fmot:%lf\t MAttached:%lf\n",motorhead_initial[NumMotor],attachcondition[NumMotor],fmotor[NumMotor],MotorAttached);
                        }
                        if (attachcondition[NumMotor]==-1.000000)
                        {
                            if(Random1<=probabretach[NumMotor] && attachcondition[NumMotor]==-1.000000 )    //THINK ABOUT THE CONDITION FOR THE CHECKING THE CARGO ENDING (4 conditions)
                            {
                                TentativeAttachments=PositionCargo-motorhead_initial[NumMotor];
                                if(TentativeAttachments>restlength || TentativeAttachments<-lengthcargo-restlength)
                                {
                                    //Nothing Happens.. No Possible Retachment Site.
                                }
                                else if (TentativeAttachments=<restlength || TentativeAttachments>-restlength)  //SIGN CONVENTION       (will vary from -restlength to -TentativeAttachments)
                                {
                                    motor_stretch[NumMotor]=(int)((double)rand()/(RAND_MAX)*(restlength-TentativeAttachments))+1; // Motor head position on Track Microtubule wrt Cargo
                                    if(motor_stretch[NumMotor]>0.000000)    //To bind the motor within +-90nm restlength to the track wrt cargo             //CHECK THE CHANGE      ********DONE*******
                                    {
                                        motor_stretch[NumMotor]=motor_stretch[NumMotor]-(restlength+1.000000);
                                    }
                                    motortail_tentative[NumMotor]=motorhead_initial[NumMotor]-motor_stretch[NumMotor];
                                    attachcondition[NumMotor]=1;    // Motor is attached
                                    MotorAttached++;        //fprintf(ft,"Head: %lf\t AttCon:%lf\t MAttached:%lf\n",motorhead_initial[NumMotor],attachcondition[NumMotor],MotorAttached);//Total attached motors
                                }
                                else if (TentativeAttachments<=-restlength && TentativeAttachments>=-lengthcargo+restlength)
                                {
                                    motor_stretch[NumMotor]=(int)((double)rand()/(RAND_MAX)*(2.000000*restlength+1.000000)); // Motor head position on Track Microtubule wrt Cargo
                                    if(motor_stretch[NumMotor]>restlength)  //To bind the motor within +-90nm restlength to the track wrt cargo
                                    {
                                        motor_stretch[NumMotor]=motor_stretch[NumMotor]-(2.000000*restlength+1.000000);
                                    }
                                    motortail_tentative[NumMotor]=motorhead_initial[NumMotor]-motor_stretch[NumMotor];
                                    attachcondition[NumMotor]=1;    // Motor is attached
                                    MotorAttached++;        //fprintf(ft,"Head: %lf\t AttCon:%lf\t MAttached:%lf\n",motorhead_initial[NumMotor],attachcondition[NumMotor],MotorAttached);//Total attached motors
                                }
                                else if (TentativeAttachments<-lengthcargo+restlength && TentativeAttachments>=-lengthcargo-restlength)         //(will vary from -TentativeAttachments to Restlength)
                                {
                                    motor_stretch[NumMotor]=(int)((double)rand()/(RAND_MAX)*(restlength+lengthcargo+TentativeAttachments))+1; // Motor head position on Track Microtubule wrt Cargo
                                    if (motor_stretch[NumMotor]>0)
                                    {
                                        motor_stretch[NumMotor]=restlength+1.000000-motor_stretch[NumMotor];
                                    }
                                    motortail_tentative[NumMotor]=motorhead_initial[NumMotor]-motor_stretch[NumMotor];
                                    attachcondition[NumMotor]=1;    // Motor is attached
                                    MotorAttached++;        //fprintf(ft,"Head: %lf\t AttCon:%lf\t MAttached:%lf\n",motorhead_initial[NumMotor],attachcondition[NumMotor],MotorAttached);//Total attached motors
                                }
                            }
                            if(Random<=probabstep[NumMotor])        //Stepping occurs
                            {
                                motorhead_tentative[NumMotor]=motorhead_initial[NumMotor]+stepsize;
                            }
                            else if (Random>probabstep[NumMotor] && Random<= (probabstep[NumMotor]+probabdetach[NumMotor])) //Motor Detaches
                            {
                                attachcondition[NumMotor]= 0.000000;    // Motor is detached
                                motorhead_initial[NumMotor]=motortail_initial[NumMotor];        //Assumed Default Position as it has no effect on Calculations                  ******************CHECK IT**************
                                motor_stretch[NumMotor]=0.000000;               //Assumed Default Position as it has no effect on Calculations
                                fmotor[NumMotor]=0.000000;
                                MotorAttached--;        //Total attached motors
                                if (MotorAttached==0)
                                {
                                    break;  //fprintf(ft,"Time Exit: %lf\t MAttached:%lf\n",Time,MotorAttached);    //Break Motor Loop
                                }
                            }
                            else    //Motor remains Stationary
                            {
                                //Nothing Happens :P
                            }       //fprintf(ft,"Head:%lf\t AttCon:%lf\t Fmot:%lf\t MAttached:%lf\n",motorhead_initial[NumMotor],attachcondition[NumMotor],fmotor[NumMotor],MotorAttached);
                        }
                    }
                }       //Motor Loop Ends
                AvgMotorAttached=AvgMotorAttached+MotorAttached;
                //fprintf(fz,"%lf %lf\n",Time, MotorAttached);
                if (MotorAttached==0)
                {
                    break;  //Break Time Loop
                }
                fmotors=0.000000;       //fprintf(ft,"DefaultFmotall: %lf\n",fmotors);//To calculate Force by the Motor
                for(NumMotor=0;NumMotor<=N-1;NumMotor++)        //Parallel Update of Motor Head Positon
                {
                    motorhead_final[NumMotor]=motorhead_tentative[NumMotor];
                    motortail_final[NumMotor]=motortail_tentative[NumMotor];
                    motorhead_final[NumMotor]=motorhead_initial[NumMotor];
                    motortail_final[NumMotor]=motortail_initial[NumMotor];
                    motor_stretch[NumMotor]=motorhead_final[NumMotor]-motortail_final[NumMotor];
                }
                for(NumMotor=0;NumMotor<=N-1;NumMotor++)        //For Net Force Calculations
                {
                    if (attachcondition[NumMotor]==1.000000)
                    {
                        if (motor_stretch[NumMotor]>restlength)  //Motor is stretched forward, Load experienced by motor
                        {
                            fmotor[NumMotor]= stiffness* (motor_stretch[NumMotor]-restlength);
                            fmotors=fmotors+fmotor[NumMotor];
                        }
                        else if (motor_stretch[NumMotor]<-restlength)    //Motor is stretched backward, Load experienced by motor
                        {
                            fmotor[NumMotor]= stiffness* (motorhead_final[NumMotor]+restlength);
                            fmotors=fmotors+fmotor[NumMotor];
                        }
                        else
                        {
                            fmotor[NumMotor]=0.0;
                            fmotors=fmotors+fmotor[NumMotor];
                        }       //fprintf(ft,"Fmot: %lf\t Fmotall: %lf\n",fmotor[NumMotor],fmotors);
                    }
                }       // Force By Motor Calculation Complete
                Fnet= fmotors-ForceExt; //fprintf(ft,"Fnetperturb: %lf\n",Fnet);//Intial Force after Perturbation( Occurance of Event)
                while(fabs(Fnet)> 0.00000005) // For Attainment of Equibrillium
                {
                    x=Fnet/(MotorAttached*stiffness);       //New Equibrillium Position                                             netx=netx+x;
                    PositionCargo= PositionCargo+x; //fprintf(ft,"NewPos: %lf\t CargoProgression: %lf\t CarPos: %lf\n",x,netx,PositionCargo);                    
                    for(NumMotor=0;NumMotor<=N-1;NumMotor++)
                    {
                        motortail_initial[NumMotor]=motortail_initial[NumMotor]+x;  //Motortail position Update
                        motor_stretch[NumMotor]= motor_stretch[NumMotor]-x;     //PositionCargo;        // Updated Motor stretch
                    }       //fprintf(ft,"NewHeadMot1: %lf\t NewHeadMot2: %lf\t NewHeadMot3: %lf\n",motorhead_initial[0],motorhead_initial[1],motorhead_initial[2]);//Updated Cargo Positions
                    //PositionCargo=0.000000; //New Cargo COM referenced as 0 (Relative)
                    fmotors=0.000000;       //fprintf(ft,"NewPosCarRef: %lf\t fmot: %lf\n",PositionCargo,fmotors);//To calculate Force by the Motor
                    for(NumMotor=0;NumMotor<=N-1;NumMotor++)        //For New Fnet Calculations
                    {
                        if (attachcondition[NumMotor]==1.000000)
                        {
                            if (motor_stretch[NumMotor]>restlength) //Motor is stretched backward, Motor is experiencing load
                            {
                                fmotor[NumMotor]= stiffness * (motor_stretch[NumMotor]-restlength);     //Force by Motor
                                fmotors=fmotors+fmotor[NumMotor];
                            }
                            else if (motor_stretch[NumMotor]<-restlength)   //Motor is stretched Forward, Motor is experiencing load
                            {
                                fmotor[NumMotor]= stiffness* (motor_stretch[NumMotor]+restlength);      //Force by Motor
                                fmotors=fmotors+fmotor[NumMotor];
                            }
                            else
                            {
                                fmotor[NumMotor]=0.000000;
                                fmotors=fmotors+fmotor[NumMotor];
                            }       //fprintf(ft,"Fmot: %lf\t Fmotall: %lf\n",fmotor[NumMotor],fmotors);
                        }
                    }       //// Force By Motor Calculation Complete
                    Fnet= fmotors-ForceExt; //fprintf(ft,"Fnetiteration: %lf\n",Fnet);//Fnet after Current Iterative step
                }//Equibrillium Loop Ends
                for(NumMotor=0;NumMotor<=N-1;NumMotor++)        //Parallel Update of Motor Head Positon
                {
                    motorhead_initial[NumMotor]=motorhead_final[NumMotor];
                    motorhead_tentative[NumMotor]=motorhead_final[NumMotor];
                }
                if(Time>warmtime-tstep && Time<warmtime)
                {
                    netxstart=netx;
                    //printf("%1.9lf\n",Time);
                }
                if(Time>maxtime-tstep && Time<maxtime)
                {
                    netxend=netx;
                    //printf("%1.9lf\n",Time);
                    relconfig=relconfig+1;
                    status=1;
                }
            }       //Time loop ends
            if (status==1)
            {
                xav = xav+(netxend-netxstart);  //netx;
                //temp=temp+Time;
                //FAvgMotorAttached=FAvgMotorAttached+(AvgMotorAttached*tstep/Time);
                vav = vav+ ((netxend-netxstart)/(maxtime-warmtime));  //fprintf(ft,"Xav: %lf\t Vav:%lf\n",xav,vav);
            }
            if(config>=200 && (int)config%200==0)
            {
                printf("%lf %lf %d %lf 'NoExcl,R'\n",config,ForceExt,relconfig,vav/relconfig);
            }
        }       //Configuration loop ends
        xav=xav/relconfig;
        //temp=temp/maxconfig;
        //FAvgMotorAttached=FAvgMotorAttached/maxconfig;
        vav=vav/relconfig;  //fprintf(ft,"Xav: %lf\t Vav:%lf\n",xav,vav);
        printf("%lf %lf\n",ForceExt,vav);
        fprintf(fp,"%lf %lf %lf\n",ForceExt,xav*0.001,vav*0.001);       //,temp,FAvgMotorAttached
    }       //Force Loop Ends
    fclose(fp);fclose(ft);fclose(fz);
    return (EXIT_SUCCESS);       //Main Ends
}

