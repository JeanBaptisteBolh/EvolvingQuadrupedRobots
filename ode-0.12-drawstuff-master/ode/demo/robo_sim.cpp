//-------------------------------------------------------
//
// CPG-controlled bounding quadruped robot
// For variable search of CPG, it also describes a simple genetic algorithm
// Please understand by studying their own so there.
//
// [How to use]
// Those who want to explore the variables for the CPG by the genetic algorithm,
// Please set to "GA_SIM == 0". So it is a text file
// "Double cpg2ga [LEN-1] = {- 0.26, -0.21,1.00,0.69,0.61,0.80, -0.90,0.25, -0.75}; // 4.31"
// (And you may have wrote the location below) Please Copy and paste this within the program.
// It should be noted that this program, it utilizes the CPG variable obtained from genetic algorithm already.
// Demonstration also set in the "GA_SIM == 1", I can be seen walking.
//--------------------------------------------------------  

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include <math.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <ctime>
using namespace std;

//#include "texturepath.h"
#ifndef DRAWSTUFF_TEXTURE_PATH
#define DRAWSTUFF_TEXTURE_PATH "../../drawstuff/textures"
#endif
//

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif


//---------------------------------
// Setting of the physical world
//---------------------------------
#define PI		3.14159		// PI
#define DT		0.005		// Sampling Time
#define GKP		30000		// Modulus of elasticity of the object
#define GKD		30000		// Viscosity coefficient of the object
#define VIEW_HEIGHT 2.0		//camera height
//#define DRAW 1
//#define HEADNOHEAD 1 //1:head 0:no head
double DRAW = 0;
double HEADNOHEAD = 1; //1:head 0:no head
#define LOG 1 //outputs in files
#define INFILE 0 //inputs from file input.txt
#define SETUP 0 //get setup info from setup.txt
//#define SurfaceMue dInfinity
//#define SurfaceMue2 dInfinity
//#define SurfaceERP 0.1
//#define SurfaceCFM 1e-6
//#define gravity -9.8
dReal SurfaceMue = 1;//dInfinity;
dReal SurfaceMue2 = 1;//dInfinity;
dReal SurfaceERP = 0.8;
dReal SurfaceCFM = 1e-6;
double gravity = -9.8;


//---------------------------------
//CPG default parameters
//---------------------------------
//#define b		100
//#define F		1000
//#define alpha	5
//#define beta	50
//#define PGAIN	100
double b = 100;
double F = 1000;
double alpha = 5;
double beta = 50;
double PGAIN = 100;

//---------------------------------
//Morphology parameters
//---------------------------------
#define initialHeight 0.1
#define maxBodySeg 20
#define maxLegSeg 20

// Setting of the physical world
dWorldID world;
dSpaceID space;
dBodyID body[maxBodySeg];
dBodyID FRLeg[maxLegSeg];
dBodyID FLLeg[maxLegSeg];
dBodyID RRLeg[maxLegSeg];
dBodyID RLLeg[maxLegSeg];
dJointID bodyJoints[maxBodySeg];
dJointID FRLegJoints[maxLegSeg];
dJointID FLLegJoints[maxLegSeg];
dJointID RRLegJoints[maxLegSeg];
dJointID RLLegJoints[maxLegSeg];
dJointGroupID contactgroup;
dGeomID ground;
dSpaceID geom_group,frontRight,frontLeft,rearRight,rearLeft,bodyGeom;
dGeomID bodyBox[maxBodySeg];
dGeomID FRLegBox[maxLegSeg];
dGeomID FLLegBox[maxLegSeg];
dGeomID RRLegBox[maxLegSeg];
dGeomID RLLegBox[maxLegSeg];
dGeomID ground_box;

//head
dBodyID headBody;
dJointID neck;
dGeomID headGeom;
double headMass = 3;
double headLength = 0.1;

// reaction force
int collCounter1;
bool whichOne1[100];
dJointFeedback* collisionFB1 = new dJointFeedback[100];

int collCounter2;
bool whichOne2[100];
dJointFeedback* collisionFB2 = new dJointFeedback[100];

// sensor
dBodyID sensor[4];
dJointID fixedJoint[4];
dGeomID sensorBox[4];
dJointFeedback* sfb1 = new dJointFeedback;
dJointFeedback* sfb2 = new dJointFeedback;
dJointFeedback* sfb3 = new dJointFeedback;
dJointFeedback* sfb4 = new dJointFeedback;

dMass sensorMass;
double sensorM = 0.001;
double sensorx = 0.001;
double sensory = 0.001;
double sensorz = 0.001;
double sensorL = 0.001;
double sensorR = 0.01;

// writing CPG outputs to a file
ofstream outputFile1;
ofstream outputFile2;
ofstream GRF1;
ofstream GRF2;
ofstream outputParams;
ofstream bodyPos;

//---------------------------------
//Structural parameters
//---------------------------------
// time and terminal condition
double t=0.;
double endSim=0;
int dflag_key2=0;

double kp = 10000;
double kd = 10000;
double springERP = DT*kp / (DT*kp + kd);
double springCFM = 1 / (DT*kp + kd);

// CPG parameters
// coupling matrix for bounding gait
double A[4][4] = {0, 1, -1, -1,
				  1, 0, -1, -1,
				  -1, -1, 0, 1,
				  -1, -1, 1, 0};
// amplitude of oscillations
double mue = 0.5;//0.6;//0.8581;//0.6653;//0.43;//0.25;
// frequency of swing phase
double wSwing = 12;//22;//44.4753;//27.5098;//7.017;//2.9080;
//frequency of stance phase
double wStance = wSwing*0.5;//0.5633;//0.791;//0.8252;
double x[4] = {-0.05*sqrt(mue), -0.05*sqrt(mue), 0.05*sqrt(mue), 0.05*sqrt(mue)};
double y[4] = {0.0 , 0.0, -0.0, -0.0};
double xpre[4] = {0.0, 0.0, 0.0, 0.0};
double ypre[4] = {0.0, 0.0, 0.0, 0.0};

// parameters
double mass=0;

// robot's angle and angle rate
int rf=0,lf=0,rr=0,lr=0;
double tpos[3]={0,0,0};
double jaBody[maxBodySeg];
double jaRateBody[maxBodySeg];
double jaFR[maxLegSeg];
double jaRateFR[maxLegSeg];
double jaFL[maxLegSeg];
double jaRateFL[maxLegSeg];
double jaRR[maxLegSeg];
double jaRateRR[maxLegSeg];
double jaRL[maxLegSeg];
double jaRateRL[maxLegSeg];
double trFR, trFL, trRR, trRL;

// Quadruped morphology parameters
int bodySegNum = 1;
double* bodySegParams;
double* bodyJointsParams;

double frontRightLegContact[4] = {0, 0.4, 0.5, 0.5}; //index of body segment + relative position to body's center of mass
double frontLeftLegContact[4] = {0, 0.4, 0.5, 0.5};
double rearRightLegContact[4] = {bodySegNum-1, 0.5, 0.5, 0.5};
double rearLeftLegContact[4] = {bodySegNum-1, 0.5, 0.5, 0.5};

int frontLegsSegNum = 2;
int rearLegsSegNum = 2;
double* frontLegsSegParams;
double* frontLegsJointsParams;
double* rearLegsSegParams;
double* rearLegsJointsParams;

double damping = 0.5;

// Variables for calculating cost function
double cost;
double speed;
double travelledDistance;

//dJointFeedback feedbacks[4];
dJointFeedback*  fb1 = new dJointFeedback;
dJointFeedback*  fb2 = new dJointFeedback;
dJointFeedback*  fb3 = new dJointFeedback;
dJointFeedback*  fb4 = new dJointFeedback;

double dE[4];
double FRAngleRate;
double FRAngleRatePre;
double jaFRPre;
double jaFLPre;
double jaRRPre;
double jaRLPre;
double startTime;
double stopTime;
double startDist;
double startE;
double E;
double first, second, third, forth, fifth;
double t1, t2, t3, t4, t5;
double fallenU, fallenL;
double minTravelledDistance;

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

//----------------------------------------------------
// Initial setting of the camera position
//----------------------------------------------------
static void start()
{
  float xyz[3] = {1.0f,-2.00f,0.600f};
  float hpr[3] = {90.0000f,-0.000f,0.0000f};
  double heading;
  dVector3 camera_w;
  static int count = 0;

 // if (DRAW == 1)
	//dsSetViewpoint (xyz,hpr);
  // view management
  /*dBodyVectorToWorld( body[0], 0.0, 0.0, 0.0, car_w );
  heading = atan2( car_w[1], car_w[0] );
  hpr[0] = heading*(180/3.14);
  hpr[1] = 0.0f;
  hpr[2] = 0.0;*/

  if (DRAW == 1)
  {
	  dBodyGetRelPointPos( body[0], 0.0, 0.0, 0.0, camera_w );
	  for (int i = 0; i < 3; i++ )
		xyz[i] = camera_w[i];
	  xyz[1] = xyz[1]-2;
	  xyz[2] = xyz[2]+0.3;
	  //hpr[0] = atan2(camera_w[1], camera_w[0])*(180/3.14);
	  //hpr[1] = 0.0f;
	  //hpr[2] = 0.0;
	  dsSetViewpoint (xyz,hpr);
  }
  /*static float xyz[3] = {1.0f,-2.00f,0.600f};
  static float hpr[3] = {90.0000f,-0.000f,0.0000f};
  dsSetViewpoint (xyz,hpr);*/
}

//----------------------------------------------------
// collision calculation loop
//----------------------------------------------------
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  int i,n;

  //return if they are connected by a joint
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) 
	  return;

  const int N = 30;
  dContact contact[N];
  n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (n > 0) {
    for (i=0; i<n; i++) {
		//Physical phenomena setting part of the object between
		contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1;
		contact[i].surface.mu = SurfaceMue;//dInfinity;//1;
		contact[i].surface.mu2 = SurfaceMue2;//dInfinity;
		contact[i].surface.soft_erp = SurfaceERP;//0.1;//DT*GKP/(DT*GKP+GKD);
		contact[i].surface.soft_cfm = SurfaceCFM;//1e-6;//1/(DT*GKP+GKD);
		dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
		dJointAttach (c,
			dGeomGetBody(contact[i].geom.g1),
			dGeomGetBody(contact[i].geom.g2));
		if ((b1 == FRLeg[frontLegsSegNum-1]) || (b2 == FRLeg[frontLegsSegNum-1]))
		{
			if (collCounter1<99)
			{
				collCounter1++;
				dJointSetFeedback(c, &(collisionFB1[collCounter1]));
				whichOne1[collCounter1] = b1 == FRLeg[frontLegsSegNum-1];
			}
		}
		if ((b1 == RRLeg[rearLegsSegNum-1]) || (b2 == RRLeg[rearLegsSegNum-1]))
		{
			if (collCounter2<99)
			{
				collCounter2++;
				dJointSetFeedback(c, &(collisionFB2[collCounter2]));
				whichOne2[collCounter2] = b1 == RRLeg[rearLegsSegNum-1];
			}
		}
	}
  }
}



// called when a key pressed
static void command (int cmd)
{
	switch (cmd) {
  case 'a': case 'A':
    break;
	}
}

//----------------------------------------------------
//validate input parameters to check if the structure is feasible
//----------------------------------------------------
int validateInputParameters()
{
	return 1;
}

//----------------------------------------------------
//computing the total mass of the structure
//----------------------------------------------------
double totalMass()
{
	double mass1 = 0;
	for(int i=0; i<bodySegNum; i++)
		mass1 += bodySegParams[4*i+3];
	for(int i=0; i<frontLegsSegNum; i++)
		mass1 += 2*frontLegsSegParams[4*i+3];
	for(int i=0; i<rearLegsSegNum; i++)
		mass1 += 2*rearLegsSegParams[4*i+3];
	return mass1;
}

//----------------------------------------------------
//computing the total mass of the structure
//----------------------------------------------------
double totalBodyLength()
{
	double length = 0;
	for(int i=0; i<bodySegNum; i++)
		length += bodySegParams[4*i];
	return length;
}
//----------------------------------------------------
//reset robot's configuration to its initial
//----------------------------------------------------
static void reset(void){
  
  int i;
  dMass m;
  if (SETUP == 1)
  {
	  ifstream setupFile;
	  setupFile.open("setup.txt");
	  double input;
	  if (setupFile.is_open()) {
		  setupFile>>DRAW;
		  setupFile>>HEADNOHEAD;
		  setupFile>>b>>F>>alpha>>beta>>PGAIN;
		  setupFile>>gravity;
		  setupFile>>damping;
		  setupFile>>SurfaceMue>>SurfaceMue2>>SurfaceERP>>SurfaceCFM;
	  }
	  else{
		  cout<<"error openning setup.txt"<<endl;
	  }
	  setupFile.close();
  }
  //-------------------
  //creating physical world with initial setting
  //-------------------
  dInitODE2(0);
  world = dWorldCreate();
  space = dHashSpaceCreate (0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,0,0,gravity);
  ground = dCreatePlane (space,0,0,1,0);
  dWorldSetCFM(world, 1e-3);
  dWorldSetERP(world, 0.9);

  if (INFILE == 1)
  {
	  ifstream inputFile;
	  inputFile.open("input.txt");
	  double input;
	  if (inputFile.is_open()) {
		  inputFile>>bodySegNum;
		  bodySegParams = new double[4*bodySegNum];
		  bodyJointsParams = new double[2*bodySegNum];
		  for (int i=0; i<bodySegNum; i++)
		  {
			  inputFile>>bodySegParams[4*i]>>bodySegParams[4*i+3]>>bodyJointsParams[2*i]>>bodyJointsParams[2*i+1];
			  bodySegParams[4*i+1] = 0.2;
			  bodySegParams[4*i+2] = 0.1;
		  }
		  inputFile>>frontLegsSegNum;
		  frontLegsSegParams = new double[4*frontLegsSegNum];
		  frontLegsJointsParams = new double[2*frontLegsSegNum];
		  for (int i=0; i<frontLegsSegNum; i++)
		  {
			  inputFile>>frontLegsSegParams[4*i+1]>>frontLegsSegParams[4*i+2]>>frontLegsSegParams[4*i+3]; //r, l, m
			  frontLegsSegParams[4*i] = 0.05;
			  inputFile>>frontLegsJointsParams[2*i]>>frontLegsJointsParams[2*i+1]; //k, theta0
		  }
		  inputFile>>rearLegsSegNum;
		  rearLegsSegParams = new double[4*rearLegsSegNum];
		  rearLegsJointsParams = new double[2*rearLegsSegNum];
		  for (int i=0; i<rearLegsSegNum; i++)
		  {
			  inputFile>>rearLegsSegParams[4*i+1]>>rearLegsSegParams[4*i+2]>>rearLegsSegParams[4*i+3]; //r, l, m
			  rearLegsSegParams[4*i] = 0.05;
			  inputFile>>rearLegsJointsParams[2*i]>>rearLegsJointsParams[2*i+1];
		  }
		  inputFile>>mue;
		  inputFile>>wSwing;
		  inputFile>>input;
		  wStance = input*wSwing;
	  }
	  inputFile.close();
  }
  //define inputs here
  else if (INFILE == 2)
  {
	  bodySegNum = 1;
	  bodySegParams = new double[4];// = {0.5, 0.2, 0.1, 10};
	  bodySegParams[0] = 0.5; bodySegParams[1] = 0.2; bodySegParams[2] = 0.1; bodySegParams[3] = 10;
	  bodyJointsParams = new double[2];// = {1000, 0};
	  bodyJointsParams[0] = 1000; bodyJointsParams[1] = 0;

	  frontLegsSegNum = 2;
	  frontLegsSegParams = new double[8];// = {0.05, 0.03, 0.14, 1, 0.05, 0.03, 0.14, 1};
	  frontLegsSegParams[0] = 0.05; frontLegsSegParams[1] = 0.03; frontLegsSegParams[2] = 0.14; frontLegsSegParams[3] = 1;
	  frontLegsSegParams[4] = 0.05; frontLegsSegParams[5] = 0.03; frontLegsSegParams[6] = 0.14; frontLegsSegParams[7] = 1;
	  frontLegsJointsParams = new double[4];// = {0, 0, 100, -PI/6};
	  frontLegsJointsParams[0] = 0; frontLegsJointsParams[1] = 0;
	  frontLegsJointsParams[2] = 100; frontLegsJointsParams[3] = -PI/6;

	  rearLegsSegNum = 2;
	  rearLegsSegParams = new double[8];// = {0.05, 0.03, 0.14, 1, 0.05, 0.03, 0.14, 1};
	  rearLegsSegParams[0] = 0.05; rearLegsSegParams[1] = 0.03; rearLegsSegParams[2] = 0.14; rearLegsSegParams[3] = 1;
	  rearLegsSegParams[4] = 0.05; rearLegsSegParams[5] = 0.03; rearLegsSegParams[6] = 0.14; rearLegsSegParams[7] = 1;
	  rearLegsJointsParams = new double [4];// = {0, 0, 100, -PI/6};
	  rearLegsJointsParams[0] = 0; rearLegsJointsParams[1] = 0;
	  rearLegsJointsParams[2] = 100; rearLegsJointsParams[3] = -PI/6;

	  mue = 0.3;
	  wSwing = 12;
	  wStance = 12*0.5;
  }

  rearRightLegContact[0] = bodySegNum-1;
  rearLeftLegContact[0] = bodySegNum-1;

  if (LOG == 1)
  {
	  outputFile1.open("CPGAngles.txt");
	  outputFile2.open("RobotAngles.txt");
	  GRF1.open("GRF-FrontRight.txt");
	  GRF2.open("GRF-RearRight.txt");
	  bodyPos.open("bodyPosition.txt");
	  outputParams.open("outputParams.txt");
  }


	// creating robot body parts

	dMatrix3 R;
	double bodyInitPos[3] = {0, 0, 0};
	double FRLegInitPos[3] = {0, 0, 0};
	double FLLegInitPos[3] = {0, 0, 0};
	double RRLegInitPos[3] = {0, 0, 0};
	double RLLegInitPos[3] = {0, 0, 0};
	double index = 0;
	double temp1[3] = {0, 0, 0};
	double temp2[3] = {0, 0, 0};
	double headPos[3] = {0, 0, 0};
	double temp11 = 0;
	double temp22 = 0;
	  
	// body
	for(int i=0; i<bodySegNum; i++)
	{
		body[i] = dBodyCreate (world);
		if (i > 0)
		{
			bodyInitPos[0] = bodyInitPos[0] - 0.5 * bodySegParams[(i-1)*4] * cos(bodyJointsParams[(i-1)*2+1]) - 0.5 * bodySegParams[(i)*4] * cos(bodyJointsParams[(i)*2+1]);
			bodyInitPos[1] = 0;
			bodyInitPos[2] = bodyInitPos[2] + 0.5 * bodySegParams[(i-1)*4] * sin(bodyJointsParams[(i-1)*2+1]) + 0.5 * bodySegParams[(i)*4] * sin(bodyJointsParams[(i)*2+1]);
		}
		else
		{
			bodyInitPos[2] = initialHeight + 0.5*bodySegParams[i*4+2];
			for (int j=0; j<frontLegsSegNum; j++)
			{
				temp11 += frontLegsSegParams[j*4+2] * cos(frontLegsJointsParams[j*2+1]);
			}
			for (int j=0; j<rearLegsSegNum; j++)
			{
				temp22 += rearLegsSegParams[j*4+2] * cos(rearLegsJointsParams[j*2+1]);
			}
			fallenL = (1/3)*((temp11>temp22) * temp11 + (temp11<=temp22) * temp22);
			fallenU = 3*((temp11>temp22) * temp11 + (temp11<=temp22) * temp22);
			bodyInitPos[2] += (temp11>temp22) * temp11 + (temp11<=temp22) * temp22;
		}
		if (i == frontRightLegContact[0])
		{
			temp1[0] = bodyInitPos[0];
			temp1[1] = bodyInitPos[1];
			temp1[2] = bodyInitPos[2];
		}
		if (i == rearRightLegContact[0])
		{
			temp2[0] = bodyInitPos[0];
			temp2[1] = bodyInitPos[1];
			temp2[2] = bodyInitPos[2];
		}
		if (i == 0)
		{
			headPos[0] = bodyInitPos[0];
			headPos[1] = bodyInitPos[1];
			headPos[2] = bodyInitPos[2];
		}
		dBodySetPosition (body[i], bodyInitPos[0], bodyInitPos[1], bodyInitPos[2]);
		dRFromAxisAndAngle(R, 0, 1, 0, bodyJointsParams[i*2+1]);
		dBodySetRotation(body[i], R);
		dMassSetBox (&m, 1, bodySegParams[i*4], bodySegParams[i*4+1], bodySegParams[i*4+2]);
		dMassAdjust (&m, bodySegParams[i*4+3]);
		dBodySetMass (body[i], &m);
		bodyBox[i] = dCreateBox (space, bodySegParams[i*4], bodySegParams[i*4+1], bodySegParams[i*4+2]);
		dGeomSetBody (bodyBox[i], body[i]);
	}

	// head
	if (HEADNOHEAD == 1)
	{
		headMass = 0.1*totalMass();
		headBody = dBodyCreate(world);
		headLength = 0.5*bodySegParams[1];
		headPos[0] = headPos[0] + 0.5*bodySegParams[0];// - 0.5*headLength;
		headPos[2] = headPos[2] + 0.5*bodySegParams[2] + 0.5*headLength;
		dBodySetPosition (headBody, headPos[0], headPos[1], headPos[2]);
		dRFromAxisAndAngle(R, 0, 1, 0, 0);
		dBodySetRotation(headBody, R);
		dMassSetBox (&m, 1, headLength, headLength, headLength);
		dMassAdjust (&m, headMass);
		dBodySetMass (headBody, &m);
		headGeom = dCreateBox (space, headLength, headLength, headLength);
		dGeomSetBody (headGeom, headBody);
	}
 
	// front right and left legs
	for (int i=0; i<frontLegsSegNum; i++)
	{
		//right leg
		FRLeg[i] = dBodyCreate (world);
		if (i > 0)
		{
			FRLegInitPos[0] = FRLegInitPos[0] - 0.5 * frontLegsSegParams[(i-1)*4+2] * sin(frontLegsJointsParams[(i-1)*2+1]) - 0.5 * frontLegsSegParams[i*4+2] * sin(frontLegsJointsParams[i*2+1]);
			FRLegInitPos[1] = FRLegInitPos[1];
			FRLegInitPos[2] = FRLegInitPos[2] - 0.5 * frontLegsSegParams[(i-1)*4+2] * cos(frontLegsJointsParams[(i-1)*2+1]) - 0.5 * frontLegsSegParams[i*4+2] * cos(frontLegsJointsParams[i*2+1]);
		}
		else
		{
			index = frontRightLegContact[0];
			FRLegInitPos[0] = temp1[0] + bodySegParams[(int)index*4] * frontRightLegContact[1] - 0.5 * frontLegsSegParams[i*4+2] * sin(frontLegsJointsParams[i*2+1]);
			FRLegInitPos[1] = temp1[1] - bodySegParams[(int)index*4+1] * frontRightLegContact[2];
			FRLegInitPos[2] = temp1[2] - bodySegParams[(int)index*4+2] * frontRightLegContact[3] - 0.5 * frontLegsSegParams[i*4+2] * cos(frontLegsJointsParams[i*2+1]);
		}
		  
		dBodySetPosition (FRLeg[i], FRLegInitPos[0], FRLegInitPos[1], FRLegInitPos[2]);
		dRFromAxisAndAngle(R, 0, 1, 0, frontLegsJointsParams[i*2+1]);
		dBodySetRotation(FRLeg[i], R);
		dMassSetCapsuleTotal (&m, frontLegsSegParams[i*4+3], 3, frontLegsSegParams[i*4+1], frontLegsSegParams[i*4+2]);
		dBodySetMass (FRLeg[i], &m);
		FRLegBox[i] = dCreateCapsule (space, frontLegsSegParams[i*4+1], frontLegsSegParams[i*4+2]);
		dGeomSetBody (FRLegBox[i], FRLeg[i]);

		// left leg
		FLLeg[i] = dBodyCreate (world);
		if (i > 0)
		{
			FLLegInitPos[0] = FLLegInitPos[0] - 0.5 * frontLegsSegParams[(i-1)*4+2] * sin(frontLegsJointsParams[(i-1)*2+1]) - 0.5 * frontLegsSegParams[i*4+2] * sin(frontLegsJointsParams[i*2+1]);
			FLLegInitPos[1] = FLLegInitPos[1];
			FLLegInitPos[2] = FLLegInitPos[2] - 0.5 * frontLegsSegParams[(i-1)*4+2] * cos(frontLegsJointsParams[(i-1)*2+1]) - 0.5 * frontLegsSegParams[i*4+2] * cos(frontLegsJointsParams[i*2+1]);
		}
		else
		{
			index = frontLeftLegContact[0];
			FLLegInitPos[0] = temp1[0] + bodySegParams[(int)index*4] * frontLeftLegContact[1] - 0.5 * frontLegsSegParams[i*4+2] * sin(frontLegsJointsParams[i*2+1]);
			FLLegInitPos[1] = temp1[1] + bodySegParams[(int)index*4+1] * frontLeftLegContact[2];
			FLLegInitPos[2] = temp1[2] - bodySegParams[(int)index*4+2] * frontLeftLegContact[3] - 0.5 * frontLegsSegParams[i*4+2] * cos(frontLegsJointsParams[i*2+1]);
		}
		  
		dBodySetPosition (FLLeg[i], FLLegInitPos[0], FLLegInitPos[1], FLLegInitPos[2]);
		dRFromAxisAndAngle(R, 0, 1, 0, frontLegsJointsParams[i*2+1]);
		dBodySetRotation(FLLeg[i], R);
		dMassSetCapsuleTotal (&m, frontLegsSegParams[i*4+3], 3, frontLegsSegParams[i*4+1], frontLegsSegParams[i*4+2]);
		dBodySetMass (FLLeg[i], &m);
		FLLegBox[i] = dCreateCapsule (space, frontLegsSegParams[i*4+1], frontLegsSegParams[i*4+2]);
		dGeomSetBody (FLLegBox[i], FLLeg[i]);
	}
	
	// rear right and left legs
	for (int i=0; i<rearLegsSegNum; i++)
	{
		//right leg
		RRLeg[i] = dBodyCreate (world);
		if (i > 0)
		{
			RRLegInitPos[0] = RRLegInitPos[0] - 0.5 * rearLegsSegParams[(i-1)*4+2] * sin(rearLegsJointsParams[(i-1)*2+1]) - 0.5 * rearLegsSegParams[i*4+2] * sin(rearLegsJointsParams[i*2+1]);
			RRLegInitPos[1] = RRLegInitPos[1];
			RRLegInitPos[2] = RRLegInitPos[2] - 0.5 * rearLegsSegParams[(i-1)*4+2] * cos(rearLegsJointsParams[(i-1)*2+1]) - 0.5 * rearLegsSegParams[i*4+2] * cos(rearLegsJointsParams[i*2+1]);
		}
		else
		{
			index = rearRightLegContact[0];
			RRLegInitPos[0] = temp2[0] - bodySegParams[(int)index*4] * rearRightLegContact[1] - 0.5 * rearLegsSegParams[i*4+2] * sin(rearLegsJointsParams[i*2+1]);
			RRLegInitPos[1] = temp2[1] - bodySegParams[(int)index*4+1] * rearRightLegContact[2];
			RRLegInitPos[2] = temp2[2] - bodySegParams[(int)index*4+2] * rearRightLegContact[3] - 0.5 * rearLegsSegParams[i*4+2] * cos(rearLegsJointsParams[i*2+1]);
		}
		  
		dBodySetPosition (RRLeg[i], RRLegInitPos[0], RRLegInitPos[1], RRLegInitPos[2]);
		dRFromAxisAndAngle(R, 0, 1, 0, rearLegsJointsParams[i*2+1]);
		dBodySetRotation(RRLeg[i], R);
		dMassSetCapsuleTotal (&m, rearLegsSegParams[i*4+3], 3, rearLegsSegParams[i*4+1], rearLegsSegParams[i*4+2]);
		dBodySetMass (RRLeg[i], &m);
		RRLegBox[i] = dCreateCapsule (space, rearLegsSegParams[i*4+1], rearLegsSegParams[i*4+2]);
		dGeomSetBody (RRLegBox[i], RRLeg[i]);

		// left leg
		RLLeg[i] = dBodyCreate (world);
		if (i > 0)
		{
			RLLegInitPos[0] = RLLegInitPos[0] - 0.5 * rearLegsSegParams[(i-1)*4+2] * sin(rearLegsJointsParams[(i-1)*2+1]) - 0.5 * rearLegsSegParams[i*4+2] * sin(rearLegsJointsParams[i*2+1]);
			RLLegInitPos[1] = RLLegInitPos[1];
			RLLegInitPos[2] = RLLegInitPos[2] - 0.5 * rearLegsSegParams[(i-1)*4+2] * cos(rearLegsJointsParams[(i-1)*2+1]) - 0.5 * rearLegsSegParams[i*4+2] * cos(rearLegsJointsParams[i*2+1]);
		}
		else
		{
			index = rearLeftLegContact[0];
			RLLegInitPos[0] = temp2[0] - bodySegParams[(int)index*4] * rearLeftLegContact[1] - 0.5 * rearLegsSegParams[i*4+2] * sin(rearLegsJointsParams[i*2+1]);
			RLLegInitPos[1] = temp2[1] + bodySegParams[(int)index*4+1] * rearLeftLegContact[2];
			RLLegInitPos[2] = temp2[2] - bodySegParams[(int)index*4+2] * rearLeftLegContact[3] - 0.5 * rearLegsSegParams[i*4+2] * cos(rearLegsJointsParams[i*2+1]);
		}
		  
		dBodySetPosition (RLLeg[i], RLLegInitPos[0], RLLegInitPos[1], RLLegInitPos[2]);
		dRFromAxisAndAngle(R, 0, 1, 0, rearLegsJointsParams[i*2+1]);
		dBodySetRotation(RLLeg[i], R);
		dMassSetCapsuleTotal (&m, rearLegsSegParams[i*4+3], 3, rearLegsSegParams[i*4+1], rearLegsSegParams[i*4+2]);
		dBodySetMass (RLLeg[i], &m);
		RLLegBox[i] = dCreateCapsule (space, rearLegsSegParams[i*4+1], rearLegsSegParams[i*4+2]);
		dGeomSetBody (RLLegBox[i], RLLeg[i]);

	}

	// adding joints between body parts

	double myCFM = 0;
	double myERP = 0;

	//body
	for (int i=0; i<bodySegNum-1; i++)
	{
		bodyJoints[i] = dJointCreateHinge (world, 0);
		dJointAttach (bodyJoints[i], body[i], body[i+1]);
		const dReal *a1 = dBodyGetPosition (body[i+1]);
		dJointSetHingeAnchor (bodyJoints[i], a1[0]+0.5*bodySegParams[(i+1)*4]*cos(bodyJointsParams[(i+1)*2+1]), a1[1], a1[2]-0.5*bodySegParams[(i+1)*4]*sin(bodyJointsParams[(i+1)*2+1]));
		dJointSetHingeAxis (bodyJoints[i], 0, 1, 0);
		//dJointSetHingeParam (bodyJoints[i], dParamLoStop, bodyJointsParams[(i+1)*2+1]);
		//dJointSetHingeParam (bodyJoints[i], dParamHiStop, bodyJointsParams[(i+1)*2+1]);
		dJointSetHingeParam (bodyJoints[i], dParamFudgeFactor, 0.1);
		//myERP = DT*bodyJointsParams[(i+1)*2] / (DT*bodyJointsParams[(i+1)*2] + damping);
		//myCFM = 1 / (DT*bodyJointsParams[(i+1)*2] + damping);
		//dJointSetHingeParam (bodyJoints[i], dParamCFM, myCFM);
		//dJointSetHingeParam (bodyJoints[i], dParamERP, myERP);
	}

	if (HEADNOHEAD == 1)
	{
		neck = dJointCreateFixed(world, 0);
		dJointAttach(neck, headBody, body[0]);
		dJointSetFixed(neck);
	}

	// front right
	for (int i=0; i<frontLegsSegNum; i++)
	{
		FRLegJoints[i] = dJointCreateHinge (world, 0);
		if (i == 0)
		{
			dJointAttach (FRLegJoints[i], body[(int)frontRightLegContact[0]], FRLeg[i]);
			const dReal *a1 = dBodyGetPosition (FRLeg[i]);
			dJointSetHingeAnchor (FRLegJoints[i], a1[0]+0.5*frontLegsSegParams[i*4+2]*sin(frontLegsJointsParams[i*2+1]), a1[1]+0.5*frontLegsSegParams[i*4+1], a1[2]+0.5*frontLegsSegParams[i*4+2]*cos(frontLegsJointsParams[i*2+1]));
		}
		else
		{
			dJointAttach (FRLegJoints[i], FRLeg[i-1], FRLeg[i]);
			const dReal *a1 = dBodyGetPosition (FRLeg[i]);
			dJointSetHingeAnchor (FRLegJoints[i], a1[0]+0.5*frontLegsSegParams[i*4+2]*sin(frontLegsJointsParams[i*2+1]), a1[1]+0.5*frontLegsSegParams[i*4+1], a1[2]+0.5*frontLegsSegParams[i*4+2]*cos(frontLegsJointsParams[i*2+1]));
		}
		dJointSetHingeAxis (FRLegJoints[i], 0, 1, 0);
		//dJointSetHingeParam (FRLegJoints[i], dParamLoStop, frontLegsJointsParams[i*2+1]);
		//dJointSetHingeParam (FRLegJoints[i], dParamHiStop, frontLegsJointsParams[i*2+1]);
		dJointSetHingeParam (FRLegJoints[i], dParamFudgeFactor, 0.1);
		//myERP = DT*frontLegsJointsParams[i*2] / (DT*frontLegsJointsParams[i*2] + damping);
		//myCFM = 1 / (DT*frontLegsJointsParams[i*2] + damping);
		//dJointSetHingeParam (FRLegJoints[i], dParamCFM, myCFM);
		//dJointSetHingeParam (FRLegJoints[i], dParamERP, myERP);
	}
	// front left
	for (int i=0; i<frontLegsSegNum; i++)
	{
		FLLegJoints[i] = dJointCreateHinge (world, 0);
		if (i == 0)
		{
			dJointAttach (FLLegJoints[i], body[(int)frontLeftLegContact[0]], FLLeg[i]);
			const dReal *a1 = dBodyGetPosition (FLLeg[i]);
			dJointSetHingeAnchor (FLLegJoints[i], a1[0]+0.5*frontLegsSegParams[i*4+2]*sin(frontLegsJointsParams[i*2+1]), a1[1]+0.5*frontLegsSegParams[i*4+1], a1[2]+0.5*frontLegsSegParams[i*4+2]*cos(frontLegsJointsParams[i*2+1]));
		}
		else
		{
			dJointAttach (FLLegJoints[i], FLLeg[i-1], FLLeg[i]);
			const dReal *a1 = dBodyGetPosition (FLLeg[i]);
			dJointSetHingeAnchor (FLLegJoints[i], a1[0]+0.5*frontLegsSegParams[i*4+2]*sin(frontLegsJointsParams[i*2+1]), a1[1]-0.5*frontLegsSegParams[i*4+1], a1[2]+0.5*frontLegsSegParams[i*4+2]*cos(frontLegsJointsParams[i*2+1]));
		}
		dJointSetHingeAxis (FLLegJoints[i], 0, 1, 0);
		//dJointSetHingeParam (FLLegJoints[i], dParamLoStop, frontLegsJointsParams[i*2+1]);
		//dJointSetHingeParam (FLLegJoints[i], dParamHiStop, frontLegsJointsParams[i*2+1]);
		dJointSetHingeParam (FLLegJoints[i], dParamFudgeFactor, 0.1);
		//myERP = DT*frontLegsJointsParams[i*2] / (DT*frontLegsJointsParams[i*2] + damping);
		//myCFM = 1 / (DT*frontLegsJointsParams[i*2] + damping);
		//dJointSetHingeParam (FLLegJoints[i], dParamCFM, myCFM);
		//dJointSetHingeParam (FLLegJoints[i], dParamERP, myERP);
	}
	// rear right
	for (int i=0; i<rearLegsSegNum; i++)
	{
		RRLegJoints[i] = dJointCreateHinge (world, 0);
		if (i == 0)
		{
			dJointAttach (RRLegJoints[i], body[(int)rearRightLegContact[0]], RRLeg[i]);
			const dReal *a1 = dBodyGetPosition (RRLeg[i]);
			dJointSetHingeAnchor (RRLegJoints[i], a1[0]+0.5*rearLegsSegParams[i*4+2]*sin(rearLegsJointsParams[i*2+1]), a1[1]+0.5*rearLegsSegParams[i*4+1], a1[2]+0.5*rearLegsSegParams[i*4+2]*cos(rearLegsJointsParams[i*2+1]));
		}
		else
		{
			dJointAttach (RRLegJoints[i], RRLeg[i-1], RRLeg[i]);
			const dReal *a1 = dBodyGetPosition (RRLeg[i]);
			dJointSetHingeAnchor (RRLegJoints[i], a1[0]+0.5*rearLegsSegParams[i*4+2]*sin(rearLegsJointsParams[i*2+1]), a1[1]+0.5*rearLegsSegParams[i*4+1], a1[2]+0.5*rearLegsSegParams[i*4+2]*cos(rearLegsJointsParams[i*2+1]));
		}
		dJointSetHingeAxis (RRLegJoints[i], 0, 1, 0);
		//dJointSetHingeParam (RRLegJoints[i], dParamLoStop, rearLegsJointsParams[i*2+1]);
		//dJointSetHingeParam (RRLegJoints[i], dParamHiStop, rearLegsJointsParams[i*2+1]);
		dJointSetHingeParam (RRLegJoints[i], dParamFudgeFactor, 0.1);
		//myERP = DT*rearLegsJointsParams[i*2] / (DT*rearLegsJointsParams[i*2] + damping);
		//myCFM = 1 / (DT*rearLegsJointsParams[i*2] + damping);
		//dJointSetHingeParam (RRLegJoints[i], dParamCFM, myCFM);
		//dJointSetHingeParam (RRLegJoints[i], dParamERP, myERP);
	}

	// rear left
	for (int i=0; i<rearLegsSegNum; i++)
	{
		RLLegJoints[i] = dJointCreateHinge (world, 0);
		if (i == 0)
		{
			dJointAttach (RLLegJoints[i], body[(int)rearLeftLegContact[0]], RLLeg[i]);
			const dReal *a1 = dBodyGetPosition (RLLeg[i]);
			dJointSetHingeAnchor (RLLegJoints[i], a1[0]+0.5*rearLegsSegParams[i*4+2]*sin(rearLegsJointsParams[i*2+1]), a1[1]+0.5*rearLegsSegParams[i*4+1], a1[2]+0.5*rearLegsSegParams[i*4+2]*cos(rearLegsJointsParams[i*2+1]));
		}
		else
		{
			dJointAttach (RLLegJoints[i], RLLeg[i-1], RLLeg[i]);
			const dReal *a1 = dBodyGetPosition (RLLeg[i]);
			dJointSetHingeAnchor (RLLegJoints[i], a1[0]+0.5*rearLegsSegParams[i*4+2]*sin(rearLegsJointsParams[i*2+1]), a1[1]-0.5*rearLegsSegParams[i*4+1], a1[2]+0.5*rearLegsSegParams[i*4+2]*cos(rearLegsJointsParams[i*2+1]));
		}
		dJointSetHingeAxis (RLLegJoints[i], 0, 1, 0);
		//dJointSetHingeParam (RLLegJoints[i], dParamLoStop, rearLegsJointsParams[i*2+1]);
		//dJointSetHingeParam (RLLegJoints[i], dParamHiStop, rearLegsJointsParams[i*2+1]);
		dJointSetHingeParam (RLLegJoints[i], dParamFudgeFactor, 0.1);
		//myERP = DT*rearLegsJointsParams[i*2] / (DT*rearLegsJointsParams[i*2] + damping);
		//myCFM = 1 / (DT*rearLegsJointsParams[i*2] + damping);
		//dJointSetHingeParam (RLLegJoints[i], dParamCFM, myCFM);
		//dJointSetHingeParam (RLLegJoints[i], dParamERP, myERP);
	}


	// get joint feedback for motor torques
	dJointSetFeedback(FRLegJoints[0], fb1);
	dJointSetFeedback(FLLegJoints[0], fb2);
	dJointSetFeedback(RRLegJoints[0], fb3);
	dJointSetFeedback(RLLegJoints[0], fb4);

    //initialization of variables
	for (int i=0; i<maxBodySeg; i++)
	{
		jaBody[i]=0;
		jaRateBody[i]=0;
	}

	for (int i=0; i<maxLegSeg; i++)
	{
		jaFR[i]=0;
		jaFL[i]=0;
		jaRR[i]=0;
		jaRL[i]=0;
		jaRateFR[i]=0;
		jaRateFL[i]=0;
		jaRateRR[i]=0;
		jaRateRL[i]=0;

	}
  trFR = 0;
  trFL = 0;
  trRR = 0;
  trRL = 0;

  x[0] = -0.05*sqrt(mue);
  x[1] = -0.05*sqrt(mue);
  x[2] = 0.05*sqrt(mue);
  x[3] = 0.05*sqrt(mue);
  y[0] = 0.0;
  y[1] = 0.0;
  y[2] = -0.0;
  y[3] = -0.0;
  for(i=0; i<4; i++){
	xpre[i] = 0;
	ypre[i] = 0;
  }
  rf=0; lf=0;

  tpos[0]=0; 
  tpos[1]=0; 
  tpos[2]=0;
  t=0;
  cost = 1000;
  speed = 0;
  travelledDistance = 0;

  for(i=0; i<4; i++)
	  dE[i] = 0;
  FRAngleRate = 0;
  FRAngleRatePre = 0;
  jaFRPre = 0;
  jaFLPre = 0;
  jaRRPre = 0;
  jaRLPre = 0;
  startTime = 0;
  stopTime = 0;
  startDist = 0;;
  startE = 0;
  E = 0;
  first = 0;
  second = 0;
  third = 0;
  forth = 0;
  fifth = 0;
  t1 = 100;
  t2 = 100;
  t3 = 100;
  t4 = 100;
  t5 = 100;
  
  collCounter1 = 0;
  collCounter2 = 0;

  minTravelledDistance = 0;
  for (int i=0; i<bodySegNum; i++)
  {
	minTravelledDistance = minTravelledDistance + bodySegParams[4*i];
  }
  minTravelledDistance = 2*minTravelledDistance;
}

//----------------------------------------------------
// end of one batch simulation
//----------------------------------------------------
static void destroy(void){
	int i;
	dJointGroupDestroy (contactgroup);
	for(i=0;i<bodySegNum;i++){
	  dGeomDestroy (bodyBox[i]);
	}
	for(i=0;i<frontLegsSegNum;i++){
	  dGeomDestroy (FRLegBox[i]);
	  dGeomDestroy (FLLegBox[i]);
	}
	for(i=0;i<rearLegsSegNum;i++){
	  dGeomDestroy (RRLegBox[i]);
	  dGeomDestroy (RLLegBox[i]);
	}

	dSpaceDestroy (space);
	dWorldDestroy (world);

	if (LOG == 1)
	{
		outputFile1.close();
		outputFile2.close();
		GRF1.close();
		GRF2.close();
		bodyPos.close();
		outputParams.close();
	}
}

// get pitch angle of a body
dReal getBodyPitch(dBodyID body)
{
	const dReal *rot = dBodyGetRotation(body);
	dReal r11,r12,r13,r21,r22,r23,r31,r32,r33;
	dReal pitch, yaw, roll;

	r11 = rot[0];    r12 = rot[1];    r13 = rot[2];
	r21 = rot[4];    r22 = rot[5];    r23 = rot[6];
	r31 = rot[8];    r32 = rot[9];    r33 = rot[10];

	pitch = atan2(-r31, sqrt(r32*r32+r33*r33));
	yaw   = atan2(r21,r11);
	roll  = atan2(r32,r33);

	return pitch;
}

//-----------------------------
// CPG
//-----------------------------
void cpgp(void){
	for(int i=0; i<4; i++)
	{
		xpre[i] = x[i];
		ypre[i] = y[i];
	}
	double r[4];
	double w[4];
	double xDot[4];

	r[0] = sqrt(pow(xpre[0], 2) + pow(ypre[0], 2));
	w[0] = wStance/(1+exp(-b*ypre[0]))+wSwing/(1+exp(b*ypre[0]));
	xDot[0] = alpha*(mue-pow(r[0], 2))*xpre[0]-w[0]*ypre[0];
	x[0] = xpre[0]+DT*xDot[0];
	y[0] = ypre[0]+DT*((beta*(mue-pow(r[0], 2)))*ypre[0]+w[0]*xpre[0]+(A[0][0]*ypre[0]+A[0][1]*ypre[1]+A[0][2]*ypre[2]+A[0][3]*ypre[3]));

	r[1] = sqrt(pow(xpre[1], 2) + pow(ypre[1], 2));
	w[1] = wStance/(1+exp(-b*ypre[1]))+wSwing/(1+exp(b*ypre[1]));
	xDot[1] = alpha*(mue-pow(r[1], 2))*xpre[1]-w[1]*ypre[1];
	x[1] = xpre[1]+DT*xDot[1];
	y[1] = ypre[1]+DT*((beta*(mue-pow(r[1], 2)))*ypre[1]+w[1]*xpre[1]+(A[1][0]*ypre[0]+A[1][1]*ypre[1]+A[1][2]*ypre[2]+A[1][3]*ypre[3]));

	r[2] = sqrt(pow(xpre[2], 2) + pow(ypre[2], 2));
	w[2] = wStance/(1+exp(-b*ypre[2]))+wSwing/(1+exp(b*ypre[2]));
	xDot[2] = alpha*(mue-pow(r[2], 2))*xpre[2]-w[2]*ypre[2];
	x[2] = xpre[2]+DT*xDot[2];
	y[2] = ypre[2]+DT*((beta*(mue-pow(r[2], 2)))*ypre[2]+w[2]*xpre[2]+(A[2][0]*ypre[2]+A[2][1]*ypre[1]+A[2][2]*ypre[2]+A[2][3]*ypre[3]));

	r[3] = sqrt(pow(xpre[3], 2) + pow(ypre[3], 2));
	w[3] = wStance/(1+exp(-b*ypre[3]))+wSwing/(1+exp(b*ypre[3]));
	xDot[3] = alpha*(mue-pow(r[3], 2))*xpre[3]-w[3]*ypre[3];
	x[3] = xpre[3]+DT*xDot[3];
	y[3] = ypre[3]+DT*((beta*(mue-pow(r[3], 2)))*ypre[3]+w[3]*xpre[3]+(A[3][0]*ypre[3]+A[3][1]*ypre[1]+A[3][2]*ypre[2]+A[3][3]*ypre[3]));


	// calculating torque
	trFR=PGAIN*(xpre[1]-jaFR[0]);	
	//tr[1]=kfr*ja[1];//(-thetafr0-ja[1]);	
	trRR=PGAIN*(xpre[3]-jaRR[0]);	
	//tr[3]=krr*ja[3];//(-thetarr0-(ja[3]));	
	trFL=PGAIN*(xpre[0]-jaFL[0]);
	//tr[5]=kfl*ja[5];//(-thetafl0-ja[5]);
	trRL=PGAIN*(xpre[2]-jaRL[0]);
	//tr[7]=krl*ja[7];//(-thetarl0-ja[7]);

	//cout<<"time:"<<t<<","<<tr[0]<<","<<tr[1]<<","<<tr[2]<<","<<tr[3]<<","<<tr[4]<<","<<tr[5]<<","<<tr[6]<<","<<tr[7]<<endl;
}



//----------------------------------------------------
// Simulation Loop
//----------------------------------------------------
static void simLoop (int pause)
{
	dReal torque;

	start();

	// Getting the joint angles
	for (int i=0; i<bodySegNum-1; i++)
	{
		jaBody[i] = dJointGetHingeAngle (bodyJoints[i]);
		//jaBody[i]=getBodyPitch(body[i+1]);
		jaRateBody[i] = dJointGetHingeAngleRate (bodyJoints[i]);
	}

	jaFRPre = jaFR[0];
	jaFLPre = jaFL[0];
	jaRRPre = jaRR[0];
	jaRLPre = jaRL[0];

	for (int i=0; i<frontLegsSegNum; i++)
	{
		jaFR[i]=dJointGetHingeAngle (FRLegJoints[i]);
		//jaFR[i] = getBodyPitch(FRLeg[i]);
		jaRateFR[i] = dJointGetHingeAngleRate (FRLegJoints[i]);
		jaFL[i]=dJointGetHingeAngle (FLLegJoints[i]);
		//jaFL[i] = getBodyPitch(FLLeg[i]);
		jaRateFL[i] = dJointGetHingeAngleRate (FLLegJoints[i]);
	}
	//jaFR[0]=dJointGetHingeAngle (FRLegJoints[0]);
	//jaFL[0]=dJointGetHingeAngle (FLLegJoints[0]);
	for (int i=0; i<rearLegsSegNum; i++)
	{
		jaRR[i]=dJointGetHingeAngle (RRLegJoints[i]);
		//jaRR[i] = getBodyPitch(RRLeg[i]);
		jaRateRR[i] = dJointGetHingeAngleRate (RRLegJoints[i]);
		jaRL[i]=dJointGetHingeAngle (RLLegJoints[i]);
		//jaRL[i] = getBodyPitch(RLLeg[i]);
		jaRateRL[i] = dJointGetHingeAngleRate (RLLegJoints[i]);
	}
	//jaRR[0]=dJointGetHingeAngle (RRLegJoints[0]);
	//jaRL[0]=dJointGetHingeAngle (RLLegJoints[0]);
	// Get the angle rate of the front right leg
	FRAngleRatePre = FRAngleRate;
	FRAngleRate = dJointGetHingeAngleRate(FRLegJoints[0]);

	// Get the main body position
	const dReal *pos0= dBodyGetPosition(body[0]);

	// Absolute coordinate position of the body
	tpos[0]=pos0[0];
	tpos[1]=pos0[1];
	tpos[2]=pos0[2];

	// Resetting the joint torques
	trFR=0;
	trFL=0;
	trRR=0;
	trRL=0;

	// Calculation of CPG
	cpgp();

	if (LOG == 1)
	{
		// Writing angles to file
		outputFile1<<x[1]<<", "<<x[0]<<", "<<x[3]<<", "<<x[2]<<";"<<endl;
		outputFile2<<jaFR[0]<<", "<<jaFL[0]<<", "<<jaRR[0]<<", "<<jaRL[0]<<";"<<endl;
		bodyPos<<pos0[0]<<", "<<pos0[1]<<", "<<pos0[2]<<", "<<getBodyPitch(body[0])<<";"<<endl;
	}

	// Motor command
	double damp = damping;
	if (t>=DT)
	{

		for (int i=0; i<bodySegNum-1; i++)
		{
			torque = -bodyJointsParams[2*(i+1)]*(jaBody[i])-(damp)*jaRateBody[i];//-bodyJointsParams[2*i+1])-(damp)*jaRateBody[i];
			dJointAddHingeTorque (bodyJoints[i], torque);

		}

		for (int i=1; i<frontLegsSegNum; i++)
		{
			torque = -frontLegsJointsParams[2*i]*(jaFR[i])-(damp)*jaRateFR[i];//-frontLegsJointsParams[2*i+1])-(damp)*jaRateFR[i];
			dJointAddHingeTorque (FRLegJoints[i], torque);

			torque = -frontLegsJointsParams[2*i]*(jaFL[i])-(damp)*jaRateFL[i];//-frontLegsJointsParams[2*i+1])-(damp)*jaRateFL[i];
			dJointAddHingeTorque (FLLegJoints[i], torque);
		}

		for (int i=1; i<rearLegsSegNum; i++)
		{
			torque = -rearLegsJointsParams[2*i]*(jaRR[i])-(damp)*jaRateRR[i];//-rearLegsJointsParams[2*i+1])-(damp)*jaRateRR[i];
			dJointAddHingeTorque (RRLegJoints[i], torque);

			torque = -rearLegsJointsParams[2*i]*(jaRL[i])-(damp)*jaRateRL[i];//-rearLegsJointsParams[2*i+1])-(damp)*jaRateRL[i];
			dJointAddHingeTorque (RLLegJoints[i], torque);
		}

		torque = -frontLegsJointsParams[0]*(jaFR[0]);//-frontLegsJointsParams[1]);
		trFR = trFR + torque;
		torque = -frontLegsJointsParams[0]*(jaFL[0]);//-frontLegsJointsParams[1]);
		trFL = trFL + torque;
		torque = -rearLegsJointsParams[0]*(jaRR[0]);//-rearLegsJointsParams[1]);
		trRR = trRR + torque;
		torque = -rearLegsJointsParams[0]*(jaRL[0]);//-rearLegsJointsParams[1]);
		trRL = trRL + torque;

		dJointSetHingeParam (FRLegJoints[0],dParamVel,trFR);
		dJointSetHingeParam (FRLegJoints[0],dParamFMax,500.0);
		dJointSetHingeParam (FLLegJoints[0],dParamVel,trFL);
		dJointSetHingeParam (FLLegJoints[0],dParamFMax,500.0);
		dJointSetHingeParam (RRLegJoints[0],dParamVel,trRR);
		dJointSetHingeParam (RRLegJoints[0],dParamFMax,500.0);
		dJointSetHingeParam (RLLegJoints[0],dParamVel,trRL);
		dJointSetHingeParam (RLLegJoints[0],dParamFMax,500.0);

		//dJointAddHingeTorque (FRLegJoints[0], trFR);
		//dJointAddHingeTorque (FLLegJoints[0], trFL);
		//dJointAddHingeTorque (RRLegJoints[0], trRR);
		//dJointAddHingeTorque (RLLegJoints[0], trRL);

		//damp oscillations
		for (int i=0; i<bodySegNum; i++)
		{
			const dReal * vel = dBodyGetAngularVel (body[i]);
			double scale = bodySegParams[4*i+3] * 0.01; //mass
			dBodyAddTorque(body[i], -vel[0] * scale, -vel[1] * scale, -vel[2] * scale);
		}

		if (HEADNOHEAD == 1)
		{
			const dReal * vel = dBodyGetAngularVel (headBody);
			double scale =headMass * 0.01; //mass
			dBodyAddTorque(headBody, -vel[0] * scale, -vel[1] * scale, -vel[2] * scale);
		}

		for (int i=0; i<frontLegsSegNum; i++)
		{
			const dReal * vel1 = dBodyGetAngularVel (FRLeg[i]);
			double scale1 = frontLegsSegParams[4*i+3] * 0.01;
			dBodyAddTorque(FRLeg[i], -vel1[0] * scale1, -vel1[1] * scale1, -vel1[2] * scale1);
			const dReal * vel2 = dBodyGetAngularVel (FLLeg[i]);
			double scale2 = frontLegsSegParams[4*i+3] * 0.01;
			dBodyAddTorque(FLLeg[i], -vel2[0] * scale2, -vel2[1] * scale2, -vel2[2] * scale2);
		}

		for (int i=0; i<rearLegsSegNum; i++)
		{
			const dReal * vel1 = dBodyGetAngularVel (RRLeg[i]);
			double scale1 = rearLegsSegParams[4*i+3] * 0.01;
			dBodyAddTorque(RRLeg[i], -vel1[0] * scale1, -vel1[1] * scale1, -vel1[2] * scale1);
			const dReal * vel2 = dBodyGetAngularVel (RLLeg[i]);
			double scale2 = rearLegsSegParams[4*i+3] * 0.01;
			dBodyAddTorque(RLLeg[i], -vel2[0] * scale2, -vel2[1] * scale2, -vel2[2] * scale2);
		}
	}

	// Dynamics computation & 1 step elapsed
	rf=0;lf=0;rr=0;lr=0;
    dSpaceCollide (space,0,&nearCallback);	// Computation of Collision Detect

	if (LOG == 1)
	{
		double Fx = 0;
		double Fy = 0;
		double Fz = 0;
		for (int i=0; i<collCounter1; i++)
		{
			//dJointGetFeedback(collision[i]);
			Fx = Fx + (whichOne1[i])?collisionFB1[i].f1[0]:collisionFB1[i].f2[0];
			Fy = Fy + (whichOne1[i])?collisionFB1[i].f1[1]:collisionFB1[i].f2[1];
			Fz = Fz + (whichOne1[i])?collisionFB1[i].f1[2]:collisionFB1[i].f2[2];
		}
		GRF1 << Fx << ", "<<Fy<<", "<<Fz<<" ;"<<endl;

		Fx = 0;
		Fy = 0;
		Fz = 0;
		for (int i=0; i<collCounter2; i++)
		{
			//dJointGetFeedback(collision[i]);
			Fx = Fx + (whichOne2[i])?collisionFB2[i].f1[0]:collisionFB2[i].f2[0];
			Fy = Fy + (whichOne2[i])?collisionFB2[i].f1[1]:collisionFB2[i].f2[1];
			Fz = Fz + (whichOne2[i])?collisionFB2[i].f1[2]:collisionFB2[i].f2[2];
		}
		GRF2 << Fx << ", "<<Fy<<", "<<Fz<<" ;"<<endl;
	}
    //dWorldStep (world,DT);					// Sampling time  
	dWorldQuickStep(world, DT);
	collCounter1 = 0;
	collCounter2 = 0;
    dJointGroupEmpty (contactgroup);		// remove all contact joints
	t=t+DT;

	if (DRAW == 1)
	{
	// draw body
	for (int i=0; i<bodySegNum; i++)
	{
		dReal sides1[3] = {bodySegParams[i*4],bodySegParams[i*4+1],bodySegParams[i*4+2]};
		dsSetColorAlpha (1,0,0,1);
		dsDrawBox (dBodyGetPosition(body[i]),dBodyGetRotation(body[i]),sides1);
	}

	//draw head
	if (HEADNOHEAD == 1)
	{
		dReal sides[3] = {headLength,headLength,headLength};
		dsSetColorAlpha (1,0,1,1);
		dsDrawBox (dBodyGetPosition(headBody),dBodyGetRotation(headBody),sides);
	}

	// draw front legs
	for (int i=0; i<frontLegsSegNum; i++)
	{
		dsSetColorAlpha (1,0,1,0.5);	dsDrawCapsule (dBodyGetPosition(FRLeg[i]),dBodyGetRotation(FRLeg[i]),frontLegsSegParams[i*4+2], frontLegsSegParams[i*4+1]);
		dsSetColorAlpha (0,0,1,0.5);	dsDrawCapsule (dBodyGetPosition(FLLeg[i]),dBodyGetRotation(FLLeg[i]),frontLegsSegParams[i*4+2], frontLegsSegParams[i*4+1]);
	}

	// draw rear legs
	for (int i=0; i<rearLegsSegNum; i++)
	{
		dsSetColorAlpha (1,0,1,0.5);	dsDrawCapsule (dBodyGetPosition(RRLeg[i]),dBodyGetRotation(RRLeg[i]),rearLegsSegParams[i*4+2], rearLegsSegParams[i*4+1]);
		dsSetColorAlpha (0,0,1,0.5);	dsDrawCapsule (dBodyGetPosition(RLLeg[i]),dBodyGetRotation(RLLeg[i]),rearLegsSegParams[i*4+2], rearLegsSegParams[i*4+1]);
	}

	}
	// get pressure sensor data
	//sfb1 = dJointGetFeedback(fixedJoint[0]);
	//GRF<< sfb1->f1[0] << ", ";
	//GRF<< sfb1->f1[1] << ", ";
	//GRF<< sfb1->f1[2] << ", ";
	//sfb2 = dJointGetFeedback(fixedJoint[1]);
	//GRF<< sfb2->f1[0] << ", ";
	//GRF<< sfb2->f1[1] << ", ";
	//GRF<< sfb2->f1[2] << ", ";
	//sfb3 = dJointGetFeedback(fixedJoint[2]);
	//GRF<< sfb3->f1[0] << ", ";
	//GRF<< sfb3->f1[1] << ", ";
	//GRF<< sfb3->f1[2] << ", ";
	//sfb4 = dJointGetFeedback(fixedJoint[3]);
	//GRF<< sfb4->f1[0] << ", ";
	//GRF<< sfb4->f1[1] << ", ";
	//GRF<< sfb4->f1[2] << ", ";

	//GRF << t << ";" << endl;

	// compute cost function
	fb1 =  dJointGetFeedback(FRLegJoints[0]);
	fb2 =  dJointGetFeedback(FLLegJoints[0]);
	fb3 =  dJointGetFeedback(RRLegJoints[0]);
	fb4 =  dJointGetFeedback(RLLegJoints[0]);

	dE[0] = abs((fb1->t1[0]+fb1->t1[1]+fb1->t1[2])*(jaFR[0]-jaFRPre));
	dE[1] = abs((fb2->t1[0]+fb2->t1[1]+fb2->t1[2])*(jaFL[0]-jaFLPre));
	dE[2] = abs((fb3->t1[0]+fb3->t1[1]+fb3->t1[2])*(jaRR[0]-jaRRPre));
	dE[3] = abs((fb4->t1[0]+fb4->t1[1]+fb4->t1[2])*(jaRL[0]-jaRLPre));

	E = E + dE[0] + dE[1] + dE[2] + dE[3];

	//Termination condition of the individual and determining the cost function in cost
	//if the robot has fallen
	if((tpos[2]<fallenL)||(tpos[2]>fallenU)){
		cost=1000;
		travelledDistance = abs(tpos[0]-startDist);
		endSim = 1;
		dsStop();
		return;
	}
	//if the body has moved backwards
	if(tpos[0]<-0.5){
		cost=1000;
		travelledDistance = abs(tpos[0]-startDist);
		endSim = 1;
		dsStop();
		return;
	}
	//get body's orientation
	if (abs(getBodyPitch(body[0]))>1.2)
	{
		cost = 1000;
		travelledDistance = abs(tpos[0]-startDist);
		endSim = 1;
		dsStop();
		return;
	}
	//canculating cost fuction as COT from t=3 in 5 steps
	if(t>2){
		if((FRAngleRatePre<0) && (FRAngleRate>0) && (jaFR[0]<0))
		{
			if (first == 0)
			{
				first = 1;
				t1 = t;
				startE = E;
				startDist = tpos[0];
				startTime = t;
			}
		}
		if((FRAngleRatePre<0) && (FRAngleRate>0) && (jaFR[0]<0) && ((t-t1)>0.1))
		{
			if (second == 0)
			{
				second = 1;
				t2 = t;
			}
		}
		if((FRAngleRatePre<0) && (FRAngleRate>0) && (jaFR[0]<0) && ((t-t2)>0.1))
		{
			if (third == 0)
			{
				third = 1;
				t3 = t;
			}
		}
		if((FRAngleRatePre<0) && (FRAngleRate>0) && (jaFR[0]<0) && ((t-t3)>0.1))
		{
			if (forth == 0)
			{
				forth = 1;
				t4 = t;
			}
		}
		if((FRAngleRatePre<0) && (FRAngleRate>0) && (jaFR[0]<0) && ((t-t4)>0.1))
		{
			if (fifth == 0)
			{
				fifth = 1;
				t5 = t;
				stopTime = t;
			}
		}
	}
	if((first+second+third+forth+fifth) == 5)
	{
		//computing cost of transportation
		cost = abs(((E-startE))/(totalMass()*((tpos[0]-startDist)/totalBodyLength())));
		speed = abs((tpos[0]-startDist)/(startTime-stopTime));
		if (tpos[0] < minTravelledDistance)
			cost = 1000;
		travelledDistance = abs(tpos[0]-startDist);
		endSim = 1;
		dsStop();
		return;
	}
	if(t>10)
	{
		cost = 1000;
		travelledDistance = abs(tpos[0]-startDist);
		endSim = 1;
		dsStop();
		return;
	}
}

//----------------------------------------------------
// Main Loop
//----------------------------------------------------
int main (int argc, char **argv)
{
	time_t tstart, tend; 
	tstart = time(0);
  	// random parameter
	srand((unsigned)time(NULL));

	// setup pointers to drawstuff callback functions
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = &command;
	fn.stop = 0;
	fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

	//window position
	double xPos = 200;
	double yPos = 30;

	//get inputs from console
	if (INFILE == 0)
	{
		if (argc == 0) return -1;
		int index = 1;
		bodySegNum = (int)atof(argv[index]);
		index++;
		bodySegParams = new double[4*bodySegNum];
		bodyJointsParams = new double[2*bodySegNum];
		for (int i=0; i<bodySegNum; i++)
		{
			bodySegParams[4*i] = atof(argv[index]);
			index++;
			bodySegParams[4*i+3] = atof(argv[index]);
			index++;
			bodyJointsParams[2*i] = atof(argv[index]);
			index++;
			bodyJointsParams[2*i+1] = atof(argv[index]);
			index++;
			bodySegParams[4*i+1] = 0.2;
			bodySegParams[4*i+2] = 0.1;
		}
		frontLegsSegNum = (int) atof(argv[index]);
		index++;
		frontLegsSegParams = new double[4*frontLegsSegNum];
		frontLegsJointsParams = new double[2*frontLegsSegNum];
		for (int i=0; i<frontLegsSegNum; i++)
		{
			frontLegsSegParams[4*i+1] = atof(argv[index]);
			index++;
			frontLegsSegParams[4*i+2] = atof(argv[index]);
			index++;
			frontLegsSegParams[4*i+3] = atof(argv[index]); //r, l, m
			index++;
			frontLegsSegParams[4*i] = 0.05;
			frontLegsJointsParams[2*i] = atof(argv[index]);
			index++;
			frontLegsJointsParams[2*i+1] = atof(argv[index]); //k, theta0
			index++;
		}
		rearLegsSegNum = (int) atof(argv[index]);
		index++;
		rearLegsSegParams = new double[4*rearLegsSegNum];
		rearLegsJointsParams = new double[2*rearLegsSegNum];
		for (int i=0; i<rearLegsSegNum; i++)
		{
			rearLegsSegParams[4*i+1] = atof(argv[index]);
			index++;
			rearLegsSegParams[4*i+2] = atof(argv[index]);
			index++;
			rearLegsSegParams[4*i+3] = atof(argv[index]); //r, l, m
			index++;
			rearLegsSegParams[4*i] = 0.05;
			rearLegsJointsParams[2*i] = atof(argv[index]);
			index++;
			rearLegsJointsParams[2*i+1] = atof(argv[index]);
			index++;
		}
		mue = atof(argv[index]);
		index++;
		wSwing = atof(argv[index]);
		index++;
		double input;
		input = atof(argv[index]);
		index++;
		wStance = input*wSwing;
		DRAW = atof(argv[index]);
		index++;
		xPos = atof(argv[index]);
		index++;
		yPos = atof(argv[index]);
		index++;
	}
	reset();
	if (validateInputParameters() == 1)
	{
		if (DRAW == 1)
			dsSimulationLoop (argc,argv,800,800,&fn);//,xPos,yPos, 1);
		else
			while(!endSim)
				simLoop(0);
	}
	//putOutputsForMatlab();
	tend = time(0); 
	//cout << "It took "<< difftime(tend, tstart) <<" second(s)."<< endl;
	//cout << "cost:" <<cost<<endl;
	//cout << "speed:"<<speed<<endl;
	if (LOG == 1)
	{
		//outputParams<<"cost: "<<cost<<", speed:"<<speed<<", simulationTime: "<<difftime(tend, tstart)<<endl;
		outputParams<<cost<<endl;
		outputParams<<travelledDistance<<endl;
	}
	destroy();
	//cin >> tend;
	return 0;
}