#define maxA            0.0080f                     // used in void stop()
#define A2F             5.0f                        // used in void stop()
#define mySpeed         mathVecMagnitude(myVel,3)   // my speed in this second
#define myPos           myState
#define myVel           &myState[3]
#define myAtt           &myState[6]
#define myAngSpeed      mathVecMagnitude(&myState[9],3)
#define copyVec(to, from) memcpy(to, from, 3*sizeof(float))
#define sign(x)         x < 0? -1: 1
#define stopRotate()    api.setAttRateTarget(NULLVEC)
#define angle(a, b)     (mathVecInner(a, b, 3) / (mathVecMagnitude(a, 3) * mathVecMagnitude(b, 3)))

float UTIL[4];
#define NULLVEC         UTIL
#define ZNEG            &UTIL[1]
#define GET_ANALYZER    0                           // just a number to identify the step
#define SEARCH          1                           // just a number to identify the step
#define FIRST_DRILL     2                           // the same
#define CHOOSE          3
#define DRILL           4                           // just a number to identify the step
#define DELIVER         5                           // just a number to identify the step
#define DRILL_AROUND    6
#define MAX_HEIGHT      0.40f                       // the maximum heigth the terrain can be

float myBase[3];
float myState[12];                              // my ZR state
float pos[3];                                   // position target
float analyzer[3];                              // coordinates of the closest analyzer
float maxAnalysis;
int step;                                       // specifies what I'm doing
int lookAt[6][2];                               // array of points where I want to drill
int lookAround[2];
char s;                                         // 1 -> blue, -1 -> red
int n;                                          // how many drills I've completed, how many square I've analyzed
int timeCounter;

void init() {
    //we update all the variables
    timeCounter = 0;
    api.getMyZRState(myState);
    s = myState[0] > 0? 1: -1;
    DEBUG(("SFF: %s SPHERE code: v. IV", s == 1? "BLUE": "RED"));
    step = CHOOSE;
    
    analyzer[0] = -0.30f*s;
    analyzer[1] =  0.48f*s;
    analyzer[2] = -0.16f;
    
    //these are the point we drill at
    /*
    for(n = 0; n < 6; n++) {
        game.square2pos(lookAt[n], UTIL);
        choose(lookAt[n], UTIL);
    }*/
    
    n = 0;
    maxAnalysis = 0.0f;
    myBase[0]=0.0f;
    myBase[1]=0.0f;
    myBase[2]=0.0f;
    UTIL[0] = 0.0f;
    UTIL[1] = 0.0f;
    UTIL[2] = 0.0f;
    UTIL[3] =-1.0f;
}

void loop() {
    //we update all the variables
    api.getMyZRState(myState);
    
   /* if(step == GET_ANALYZER) {
        //DEBUG(("step: GET_ANALYZER"));
        goTo(analyzer);
        if(game.hasAnalyzer() > 0) {
            step = DRILL;
        }
    } */
    
  /*  if(game.analyzeTerrain() > maxAnalysis && game.analyzeTerrain() > 0.1f) {
        DEBUG(("Found something interesting"));
        maxAnalysis = game.analyzeTerrain();
        choose(lookAround, myPos);
        step = DRILL_AROUND;
    }*/
    if (step == CHOOSE){
        choose(lookAt[n], myPos);
        step = DRILL;
    }
    if(step == DRILL) {
        DEBUG(("step: DRILL"));
        if(getSample(lookAt[n])) {
            if (game.getNumSamplesHeld() == 4 || (timeCounter>150 && game.getNumSamplesHeld() == 2) )
                step = DELIVER;
            else
                step = CHOOSE;
            n++;
            n %=6 ;
        }
    }
    /*
    if(step == DRILL_AROUND) {
        DEBUG(("step: DRILL_AROUND"));
        if(getSample(lookAround)) {
            choose(lookAround, myPos);
        }
    }
    */
    if(step == DELIVER) {
        if (game.getNumSamplesHeld() == 0)
            step = CHOOSE;
        DEBUG(("step: DELIVER"));
        goTo(NULLVEC);
        api.setAttitudeTarget(ZNEG);
        //stopRotate();
        //DEBUG(("delivering"));
        if(mathVecMagnitude(myPos, 3) < 0.20f) {
            stop();
            if(game.atBaseStation()) {
                for(int i = 0; i < 5; i++) {
                    game.dropSample(i);
                }
                choose(lookAt[n],myBase);
                step=DRILL;
            }
        }
    }
    timeCounter = timeCounter+1;
    if (game.isGeyserHere(myPos)){
        for (int i=0;i<3;i++)
            myPos[i]=myState[i];
        float a[3]={1.0f,1.0f,myPos[2]};
        float b[3];
        int sq[2];
        game.pos2square(myPos, sq);
        game.square2pos(sq, b);
        int nextSQ[2];
        choose(nextSQ, pos);
        int x = 1;
        int y = 1;
        if (b[0]>myPos[0])
            x = -1;
        if (b[1]>myPos[1])
            y = -1;
        
        a[0] = a[0] * x;
        a[1] = a[1] * y;
        api.setPositionTarget(a);
    }
}

bool conditionsDrill(int square[2]) {
    game.square2pos(square, pos);
    pos[2] = myPos[2];
    
    if(myAngSpeed > 0.04f && !game.getDrillEnabled()) {
        DEBUG(("Spin too fast"));
    } else if(abs(myState[8]) > 0.195f) {
        DEBUG(("Not aligned"));
    } else if(mySpeed > 0.01f) {
        DEBUG(("Too fast %f", mySpeed));
    } else if(dist(myPos, pos) > 0.04f) {
        DEBUG(("Too far"));
    } else if(!(game.getTerrainHeight(square) - 0.15f < myPos[2] && myPos[2] < game.getTerrainHeight(square) - 0.11f)) {
        DEBUG(("Wrong height"));
    }
    
    return ((myAngSpeed < 0.04f || game.getDrillEnabled())
            && abs(myState[8]) < 0.195f && mySpeed < 0.01f
            && dist(myPos, pos) < 0.04f
            && (game.getTerrainHeight(square) - 0.15f) < myPos[2] && myPos[2] < (game.getTerrainHeight(square) - 0.11f));
}

bool getSample(int square[2]) {
    
    if(game.checkSample()) {
        game.pickupSample();
    }
    if(game.getDrills(square) >= 2 || game.isGeyserHere(square)) {
        game.stopDrill();
        DEBUG(("Next square"));
        return true;
    }
    if(game.getNumSamplesHeld() >= 4 || (timeCounter>150 && game.getNumSamplesHeld() == 2) ) {
        step = DELIVER;
        game.stopDrill();
        DEBUG(("Start delivering"));
        return true;
    }
    goTo(square);
    
    myState[8] = 0.0f;
    api.setAttitudeTarget(myAtt);
    
    float perp[3];
    mathVecCross(perp, myAtt, ZNEG);
    
    if(game.getDrillError()) {
        game.stopDrill();
        DEBUG(("ERROR STOP at [%d, %d]", square[0], square[1]));
        return false;
    }
    
    if(!conditionsDrill(square)) {
        api.setAttitudeTarget(myAtt);
        DEBUG(("Reaching [%d, %d]", square[0], square[1]));
        return false;
    }
    game.startDrill();
    api.setAttitudeTarget(perp);
    DEBUG(("Drilling at [%d, %d]"));
    return false;
}

float analyze(int square[2]) {
    float pos[3];
    game.square2pos(square, pos);
    pos[2] = -0.16;
    api.setPositionTarget(pos);
    if(dist(myPos, pos) < 0.04f) {
        if(mySpeed < 0.04f) {
            return game.analyzeTerrain();
        } else {
            stop();
        }
    }
    return 0.0f;
}

void choose(int square[2], float center[3]) {
    int x = 0, y = 0;
    float mindist = -1.0f;
    int sq[2];
    float sq_tmp[3];
    for(int i = -6; i <= 6; i++) {
        for(int j = -6; j <= 6; j++) {
            sq[0] = i;
            sq[1] = j;
            game.square2pos(sq, sq_tmp);
            sq_tmp[2] = center[2];
            if(game.getTerrainHeight(sq) <= 0.48f && !(j>=-2 && j<=2 && i>=-2 && i<=2)) {
                if(game.getDrills(sq) == 0) {
                    if(mindist == -1.0f || mindist > dist(center, sq_tmp)) {
                        x = i;
                        y = j;
                        mindist = dist(center, sq_tmp);
                    }
                }
                    
            }
        }
    }
    square[0] = x;
    square[1] = y;
}

void goTo(float to[3]) {
    float pos[3];
    pos[0] = to[0];
    pos[1] = to[1];
    pos[2] = myPos[2];
    
    if(dist(myPos, pos) < 0.04f || (to[2] < 0.28f && myPos[2] < 0.28f)) {
        api.setPositionTarget(to);
        //DEBUG(("Straight to target"));
        return;
    }
    if(myPos[2] > 0.28f) {
        pos[0] = myPos[0];
        pos[1] = myPos[1];
        pos[2] = 0.27f;
        //DEBUG(("Up"));
    } else {
        pos[2] = 0.27f;
        //DEBUG(("Hover"));
    }

    //DEBUG(("Going to: %f, %f, %f", pos[0], pos[1], pos[2]));
    api.setPositionTarget(pos);
}


void goTo(int square[2]) {
    game.square2pos(square, pos);
    int nextSQ[2];
    choose(nextSQ, pos);
    int x=1;
    int y=1;
    if (nextSQ[0]<square[0])
        x=-1;
    if (nextSQ[1]<square[1])
        y=-1;
        
    pos[0] = pos[0]+(0.02*x);
    pos[1] = pos[1]+(0.02*y);
    pos[2] = game.getTerrainHeight(square) - 0.13f;
    goTo(pos);
}

float dist(float* from, float* to) {
    float v[3];
    mathVecSubtract(v, to, from, 3);
    return mathVecMagnitude(v, 3);
}

void stop() {
    float v[3];
    if(mySpeed > 0.003f) {
        setLength(v, myVel, -A2F * (mySpeed<maxA ? mySpeed : maxA));
        api.setForces(v);
    }
}

void setLength(float c[3], float* a, float b) {
    copyVec(c, a);
    mathVecNormalize(c, 3);
    mul(c, c, b);
}

void mul(float c[3], float* a, float b) {
    for(int i = 0; i < 3; i++) {
        c[i] = a[i]*b;
    }
}
