///////////////////////////////////////////////////////////////////////////////////
/*
ZeroRobotics-2017- 3D Phase
Final Version: Team Westlake ZRHS

Does the following:
1. Goes to an assigned square [4,4, 5] if blue, [-4, -4, 5] if red
2. Drills until geyser hits it
3. Once hit, it goes to base station to a set point (through multiple trials and viewings of other teams, we invented this experimental point) 
and drops off item
3. Reorients back from z axis before going to next assigned square to pickup a sample.
4. This process repeats til end of simulation.

Score Range: 33 - 44 points normal, can go up to 55-60 points if we coincidentally drill in high concentration
squares right off the bat.
*/
///////////////////////////////////////////////////////////////////////////////////

//Option passed to functions indicating if they can increment the step counter
#define STEP_INC    1
#define STEP_NO_INC 0

//Current State
float myState[12];

//Current Attitude, index 6,7,8 of myState
float currAtt[3];
float myPos[3];
//Target attitude
float attitude[3];
// Target quat
float quat[4];
// Used for rotation along z-axis
float torques[3];
//Target position
//Position tolerance
float tolerance ; 
float vectorbetween[3];
float distance;


float posn[3];

//Target square
int square[3];
float targetRate[3];
//step counter
int step ;
int cycle;
int numsamples;
float stop[3];
int squarex[6];
int squarey[1];
float zoneposn[3];

//This function is called once when your code is first loaded.
void init() {
    step = 1;
    cycle = 0;
    attitude[0] = -1;
    attitude[1] = 0;
    attitude[2] = 0;
    quat[0] = 1.0;
    quat[1] = 0.0;
    quat[2] = 0.0;
    quat[3] = 0.0;
    stop[0] = 0.0;
    stop[1] = 0.0;
    stop[2] = 0.0;
     targetRate[0] = 0.0;
    targetRate[1] = 0.0;
    targetRate[2] = 50 * PI/180;
    tolerance = 0.04;
    numsamples = 0;
    square[2] = 5;
    api.getMyZRState(myState);
    if(myState[1] >= 0.2)
    {
    zoneposn[0] = 0.056;
    zoneposn[1] = 0.098;
    zoneposn[2] = 0.197;
     square[0] = 4;
     square[1] = 4;
     squarey[0] = 4;
     squarex[0] = 4;
     squarex[1] = 3;
     squarex[2] = 2;
     squarex[3] = 1;
     squarex[4] =  -1;
     squarex[5] = -2;
    }
    else {
        zoneposn[0] = -0.056;
        zoneposn[1] = -0.098;
        zoneposn[2] = 0.197;
        square[0] = -4;
        square[1] = -4;
        squarey[0] = -4;
        squarex[0] = -4;
     squarex[1] = -3;
     squarex[2] = -2;
     squarex[3] = -1;
     squarex[4] =  1;
     squarex[5] = 2;
         
    }
}

//go to position in X,Y coord
void goToPosition( float posn[] , float tolerance , int inc ) {
    float err = fabsf(myState[0]-posn[0]) + fabsf(myState[1]-posn[1]) + fabsf(myState[2] - posn[2]);
    //DEBUG(("myState[0]=%f, myState[1]=%f",myState[0],myState[1]));
    //DEBUG(("posn[0]=%f, posn[1]=%f, err=%f",posn[0],posn[1],err));
    if (err > tolerance)
        api.setPositionTarget(posn);
    else if (inc)
            step++;
}

//rotate to target attitude 
void rotatePosition( float attitude[], float tolerance, int inc) {
    float err = fabsf(myState[6]-attitude[0]) + fabsf(myState[7]-attitude[1]) + fabsf(myState[8] - attitude[2]);
    
    //DEBUG(("myState[6]=%f, myState[7]=%f",myState[6],myState[7]));
    //DEBUG(("attitude[0]=%f, attitude[1]=%f, err=%f",attitude[0],attitude[1],err));
    if (err > tolerance)
       api.setAttitudeTarget(attitude);
        
    else if (inc)
            step++;
}

float computeDistance(float myPosn[], float location[])
{
    float separation = sqrtf(powf((myPosn[0] - location[0]), 2) + powf((myPosn[1] - location[1]), 2) + powf((myPosn[2] - location[2]), 2));
    return separation;
}


//Returns true if sum of absolute values of angular velocity
//components is less than tol
int notRotating(float tol) {
    float err = fabsf(myState[9]) + fabsf(myState[10]) + fabsf(myState[11]);
    return err < tol ;
}

//Main loop , called every second.
void loop() {
    
    //Update myState array
    api.getMyZRState(myState);
    
    //Current attitude is in elements 6,7,8 of myState
    for(int i=0; i<3; i++) {
        currAtt[i] = myState[i+6];
         myPos[i] = myState[i];
        
    }
    if(game.isGeyserHere(square))
    {
        
        game.stopDrill();
        api.setAttRateTarget(stop);
        step = 4;
    }
    
    //api.setControlMode(CTRL_PD,CTRL_PID);
	//P, I and D values for position controller
	//High P value will increase speed of movement
	//resulting in overshoot, counteract with a higher
	//D value.
   // api.setPosGains(0.6,0.01,3.0);
    
    DEBUG(("Step %d", step));
    switch(step) {
        
        //Goto position
        case 1: 
        if (cycle >= 1)
        {
            square[0] = squarex[cycle];
        square[1] = squarey[0];
        square[2] = 5;
        //Convert square grid to XY coordinates (posn)
        game.square2pos(square,posn);
        mathVecSubtract(vectorbetween, posn, myPos,3 );
        distance = mathVecMagnitude(vectorbetween, 3);
     /*   if (distance > 0.28)// 0.2346
       {
            api.setVelocityTarget(vectorbetween);
        } */
        
        goToPosition(posn,tolerance,STEP_INC);
        api.setQuatTarget(quat);
        
        //rotatePosition(attitude, 0.01, STEP_NO_INC);
        }
        
        else {
        square[2] = 5;
        //Convert square grid to XY coordinates (posn)
        game.square2pos(square,posn);
        
        //mathVecSubtract(vectorbetween,origin,myPos, 3);
        //mathVecNormalize(vectorbetween, 3); 
        
        mathVecSubtract(vectorbetween, posn, myPos,3 );
        distance = mathVecMagnitude(vectorbetween, 3);
      /*  if (distance > 0.28)// 0.2346
        {
            api.setVelocityTarget(vectorbetween);
        }*/
        
            goToPosition(posn,tolerance,STEP_INC);
        
        
        }
        
        //rotatePosition(attitude,0.01,STEP_NO_INC);
        break;
        
        //Start drill
        case 2:
        goToPosition(posn,tolerance,STEP_NO_INC);
        
        if (  game.getDrillEnabled() ) {
            game.stopDrill();
        }
        else {
            game.startDrill();
            
            step++;
        }
        break;
        
        //Rotate by 90 around Z axis for sample pickup
        case 3:
        goToPosition(posn,tolerance,STEP_NO_INC);//To hold position
        if (game.getNumSamplesHeld() < 5)
        {
            api.setAttRateTarget(targetRate);
            if ( game.checkSample() )
            {
                goToPosition(posn,tolerance,STEP_NO_INC);
                game.pickupSample();
            }
            
        }
       //Rotate
        if ( game.checkSample() )
          step++;
        break;
        
        //Pick up sample
        
        //Go to base station
        case 4:
        //posn[0] =0.2;
        posn[0] = zoneposn[0];
        posn[1] = zoneposn[1];
        posn[2] = zoneposn[2];
        
        // Reorient in zsuch a way that x face of sphere rotates to face the -Z direction
        attitude[0] = 0; 
        attitude[1] = 0;
        attitude[2] = -1;
        mathVecSubtract(vectorbetween, posn, myPos,3 );
        distance = mathVecMagnitude(vectorbetween, 3);
        if (distance > 0.2346)// 0.2346
        {
            api.setVelocityTarget(vectorbetween);
        }
        else {
        goToPosition(posn,0.1,STEP_INC);
        rotatePosition(attitude,0.01,STEP_NO_INC);
        }
        break;
        
        //Drop sample
        case 5:
        
        if ( game.atBaseStation() ) {
            numsamples = game.getNumSamplesHeld();
            for (int i = 0; i < numsamples; i++)
            {
               game.dropSample(i);   
            }
            step++;
        }
        else {
            //Hold position
            goToPosition(posn,tolerance,STEP_NO_INC);
            api.setAttRateTarget(stop);
            //Stop rotating
            api.setAttitudeTarget(attitude);
        }
        break;
        
        //Done
        case 6:
       cycle++;
       step = 1;
       break;
    }
    
}
