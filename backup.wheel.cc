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

typedef struct
{
  ModelPosition* position;
  ModelRanger* ranger;
  ModelFiducial* fiducial;
	
	ModelFiducial::Fiducial* closest;  
	ModelFiducial::Fiducial* closest_red;  

	stg_radians_t closest_bearing;
	stg_meters_t closest_range;
	stg_radians_t closest_heading_error; 
	
} robot_t;

// forward declare
int RangerUpdate( ModelRanger* mod, robot_t* robot );
int FiducialUpdate( ModelFiducial* fid, robot_t* robot );

const double VSPEED = 0.4; // meters per second
const double EXPAND_WGAIN = 1.0;//0.3; // turn speed gain
const double FLOCK_WGAIN = 0.3; // turn speed gain
const double SAFE_DIST = 1.0; // meters
const double SAFE_ANGLE = 0.5; // radians


// Stage calls this when the model starts up
extern "C" int Init( Model* mod )
{
  robot_t* robot = new robot_t;
  robot->position = (ModelPosition*)mod;

  // subscribe to the ranger, which we use for navigating
  robot->ranger = (ModelRanger*)mod->GetUnusedModelOfType( "ranger" );
  assert( robot->ranger );
  
  // ask Stage to call into our ranger update function
  robot->ranger->AddCallback( Model::CB_UPDATE, (stg_model_callback_t)RangerUpdate, robot );
 
  robot->fiducial = (ModelFiducial*)mod->GetUnusedModelOfType( "fiducial" ) ;
  assert( robot->fiducial );
  robot->fiducial->AddCallback( Model::CB_UPDATE, (stg_model_callback_t)FiducialUpdate, robot );

  robot->fiducial->Subscribe();
  robot->ranger->Subscribe();
  robot->position->Subscribe();

  return 0; //ok
}

int RangerUpdate( ModelRanger* rgr, robot_t* robot )
{  	
  // compute the vector sum of the sonar ranges	      
  double dx=0, dy=0;
  
	const std::vector<ModelRanger::Sensor>& sensors = rgr->GetSensors();

  // use the front-facing sensors only
  for( unsigned int i=0; i < 8; i++ )
	 {
		dx += sensors[i].range * cos( sensors[i].pose.a );
		dy += sensors[i].range * sin( sensors[i].pose.a );
	 }
  
  if( (dx == 0) || (dy == 0) )
	 return 0;
    
  double resultant_angle = atan2( dy, dx );
  double forward_speed = 0.0;
  double side_speed = 0.0;	   
  double turn_speed = EXPAND_WGAIN * resultant_angle;
  
  // if the front is clear, drive forwards
  if( (sensors[3].range > SAFE_DIST) && // forwards
	  (sensors[4].range > SAFE_DIST) &&
	  (sensors[5].range > SAFE_DIST ) && //
	  (sensors[6].range > SAFE_DIST/2.0) && 
	  (sensors[2].range > SAFE_DIST ) && 
	  (sensors[1].range > SAFE_DIST/2.0) && 
	  (fabs( resultant_angle ) < SAFE_ANGLE) )
	{
	  forward_speed = VSPEED;
	  
	  // and steer to match the heading of the nearest robot
	  if( robot->closest )
		turn_speed += FLOCK_WGAIN * robot->closest_heading_error;
	}
  else
	{
	  // front not clear. we might be stuck, so wiggle a bit
	  if( fabs(turn_speed) < 0.1 )
		turn_speed = drand48();
	}
  
  robot->position->SetSpeed( forward_speed, side_speed, turn_speed );
  
  return 0;
}


int FiducialUpdate( ModelFiducial* fid, robot_t* robot )
{  	
  // find the closest teammate
  
  double dist = 1e6; // big
  double dist_red = 1e6; // big
  
  robot->closest = NULL;
  robot->closest_red = NULL;
  
  FOR_EACH( it, fid->GetFiducials() )
	{
	  ModelFiducial::Fiducial* other = &(*it);
	  
	  if( other->range < dist )
		{
		  dist = other->range;
		  robot->closest = other;
		}				
	  
		Color red( "red" );

	  if( other->mod->GetColor() == red && other->range < dist_red )
		{
		  dist_red = other->range;
		  robot->closest_red = other;
		}				
	}
  
  if( robot->closest_red  ) // if we saw someone red
	{
		robot->closest_bearing = robot->closest_red->bearing;
		robot->closest_range = robot->closest_red->range;
		robot->closest_heading_error = robot->closest_red->bearing;

		
		printf( "bearing %.2f\n", robot->closest_red->bearing );
	}
//   else if( robot->closest ) // if we saw someone else
// 	{
// 		robot->closest_bearing = robot->closest->bearing;
// 		robot->closest_range = robot->closest->range;
// 		robot->closest_heading_error = robot->closest->geom.a;
// 	}


  
  return 0;
}
