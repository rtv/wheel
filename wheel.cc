/////////////////////////////////
// File: wheel.cc
// Desc: experimental
// Created: 2010.6.6
// Author: Richard Vaughan <vaughan@sfu.ca>
// License: GPL
/////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stage.hh"
using namespace Stg;

static const double VSPEED = 0.4; // meters per second
static const double EXPAND_WGAIN = 1.0;//0.3; // turn speed gain
static const double FLOCK_WGAIN = 0.3; // turn speed gain
static const double SAFE_DIST = 1.0; // meters
static const double SAFE_ANGLE = 0.5; // radians

class Robot
{
public:
	// callback wrappers
	static int RangerUpdateCb( ModelRanger* mod, Robot* robot ){ return robot->RangerUpdate(); }
	static int FiducialUpdateCb( ModelFiducial* fid, Robot* robot ){ return robot->FiducialUpdate(); }

	int RangerUpdate();
	int FiducialUpdate();
	
  ModelPosition* position;
  ModelRanger* ranger;
  ModelFiducial* fiducial;
	
	radians_t best_bearing;
	meters_t best_range;
	radians_t best_heading_error; 	
	ModelFiducial::Fiducial* bestp;

	double resultant_angle;
	
	Robot( ModelPosition* p, ModelRanger* r, ModelFiducial* f) 
		: position(p), 
			ranger(r), 
			fiducial(f), 
			best_bearing(0.0),
			best_range(1.0),
			best_heading_error(0.0),
			bestp(NULL),
			resultant_angle(0.0),
			vis(this)
	{
		assert(position);
		assert(ranger);
		assert(fiducial);
			
		ranger->AddCallback( Model::CB_UPDATE, (model_callback_t)RangerUpdateCb, this ); 
		fiducial->AddCallback( Model::CB_UPDATE, (model_callback_t)FiducialUpdateCb, this );
		
		fiducial->AddVisualizer( &vis, true );

		fiducial->Subscribe();
		ranger->Subscribe();
		position->Subscribe();
	}

	class RobotVis : public Visualizer
	{
	public:
		Robot* robot;
		
		RobotVis( Robot* robot ) 
			: Visualizer( "robot", "vis_robot" ), robot(robot) {}
		virtual ~RobotVis(){}
		
	virtual void Visualize( Model* mod, Camera* cam )
		{		
			double x = robot->best_range * cos( robot->best_bearing );
			double y = robot->best_range * sin( robot->best_bearing );		

			double a = 0.2 * cos( robot->best_bearing - 4.0*M_PI/5.0 );
			double b = 0.2 * sin( robot->best_bearing - 4.0*M_PI/5.0 );		

			double c = 0.2 * cos( robot->best_bearing + 4.0*M_PI/5.0 );
			double d = 0.2 * sin( robot->best_bearing + 4.0*M_PI/5.0 );		
			
			mod->PushColor( Color(0,0,0,1.0) );

			glBegin( GL_LINES );
			glVertex2f( 0,0 );
			glVertex2f( x, y );
			glEnd();
			
			
			glBegin( GL_POLYGON );
			glVertex2f( x+c, y+d );
			glVertex2f( x+a, y+b );
			glVertex2f( x, y );
			glEnd();
			
			
			mod->PushColor( Color::green );
			glBegin( GL_LINES );
			glVertex2f( 0,0 );
			glVertex2f( cos(robot->resultant_angle), sin(robot->resultant_angle) );
			glEnd();
			mod->PopColor();
					
		// a sphere over the robot
	// 	GLUquadric* quadric = gluNewQuadric();		
// 		gluQuadricDrawStyle( quadric, GLU_FILL );
// 		gluSphere( quadric, robot->position->GetGeom().size.x*4.0, 16, 8  );
// 		gluDeleteQuadric( quadric );
 

		mod->PopColor();

		//		glPopMatrix();

		}
	} vis; // instance;
};





// Stage calls this when the model starts up
extern "C" int Init( Model* mod )
{
	new Robot( (ModelPosition*)mod,
						 (ModelRanger*)mod->GetUnusedModelOfType( "ranger" ),
						 (ModelFiducial*)mod->GetUnusedModelOfType( "fiducial"  ) );
  return 0; //ok
}

double anglediff( double a, double b )
{
	return( fmod( a + M_PI -  b, M_PI*2.0) - M_PI);
}


int Robot::RangerUpdate()
{  	
  // compute the vector sum of the sonar ranges	      
  double dx=0, dy=0;
  
	const std::vector<ModelRanger::Sensor>& sensors = ranger->GetSensors();

   // use the front-facing sensors only
   for( unsigned int i=0; i < 8; i++ )
 	 {
		 dx += ( sensors[i].ranges[0]) * cos( sensors[i].pose.a );
		 dy += ( sensors[i].ranges[0]) * sin( sensors[i].pose.a );
 	 }
	
	resultant_angle = atan2( dy, dx );
	double forward_speed = 0.0;
	double side_speed = 0.0;	   
	double turn_speed = EXPAND_WGAIN * resultant_angle;
	  
  // if the front is clear, drive forwards
	if( (sensors[3].ranges[0] > SAFE_DIST) && // forwards
	    (sensors[4].ranges[0] > SAFE_DIST) &&
	    (sensors[5].ranges[0] > SAFE_DIST ) && //
	    (sensors[6].ranges[0] > SAFE_DIST/2.0) && 
	    (sensors[2].ranges[0] > SAFE_DIST ) && 
	    (sensors[1].ranges[0] > SAFE_DIST/2.0) && 
	    (fabs( resultant_angle ) < SAFE_ANGLE) )
	  {
			if( bestp )
				forward_speed = VSPEED;
			
			//		if( best_range < 4.0 )
			//forward_speed *= 0.8;

			//forward_speed = 0.2 * (best_range - 2.0);
			

			// and steer to match the heading of the nearest robot
			//			if( best )
			//turn_speed += FLOCK_WGAIN * best_heading_error;
				//else
				//turn_speed = 0.0;
			
			// drift up
  		//if( fabs( anglediff( position->GetPose().a, 0.0 )) < 1.0 )
				//{	
 				//position->SetColor(Color::blue);
 				//turn_speed += 0.03;
				//}
			//else
			//	position->SetColor(Color::red);			

			turn_speed += FLOCK_WGAIN * best_heading_error;
			//turn_speed += FLOCK_WGAIN * best_bearing;

		}
  else
		{
			// front not clear. we might be stuck, so wiggle a bit
			if( fabs(turn_speed) < 0.1 )
				turn_speed = drand48();
		}
  

  position->SetSpeed( forward_speed, side_speed, turn_speed );
  
  return 0;
}

int Robot::FiducialUpdate()
{ 
	double best = 1e9; // big - low scores are good
	
	bestp = NULL;
	
	FOR_EACH( it, fiducial->GetFiducials() )
		{
			ModelFiducial::Fiducial* other = &(*it);
			
			// ignore things going the other way
			if( fabs(other->geom.a) > (1.0 * (M_PI/2.0)) )
				continue;

			// find the squared error difference from the last favoured fiducial
			double kd = 0.5;
			double kb = 1.0;
			
			double dist_score = other->range - best_range;
			double bear_score = anglediff( other->bearing, best_bearing );
			//double orient_score = anglediff( M_PI, other->bearing );
			
			double score = pow( kd*dist_score + kb*bear_score, 
													2.0 );
			
			if( score < best ) // best score so far seen
				{
					best = score; 
					bestp = other;
				}				
		}
		
	if( bestp )
		{
			best_bearing = bestp->bearing;
			best_range = bestp->range;
			best_heading_error = bestp->bearing;						
		}
	else
		{
			//			printf( "model %s sees noone\n", position->Token() );

			best_bearing *= 0.97;

			//if( best_range > 1.0 )
				//best_range *= 1.0 / best_range;
			best_heading_error *= 0.97;
		}
  
  return 0;
}
