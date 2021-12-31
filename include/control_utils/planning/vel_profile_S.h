/**
 Copyright (c) 2021, Zhangjie Tu.
 All rights reserved.
  @author zhanjet
 */

#ifndef VEL_PROFILE_S_HPP
#define VEL_PROFILE_S_HPP

#include <vector>
#include <stdio.h>
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

using namespace std;

class VelProfileS
{
public:
  /**
   * @brief Construct a new Vel Profile S object
   * 
   * @param ndof degree of motion
   * @param dt   cycling time
   * @param constraints motion limits of vel, acc and jerk(defined in the inner loop 
   * of the vector variable)
   */
  VelProfileS(unsigned int ndof, double dt, vector<vector<double> > constraints);
  ~VelProfileS();

  int update(vector<double> &vel_target, vector<double> &vel_planned);

private:
  int                           res_;
  ReflexxesAPI                  *RML;
  RMLPositionInputParameters    *IP;
  RMLPositionOutputParameters   *OP;
  RMLPositionFlags              Flags;

  unsigned int                  ndof_;
  double                        dt_;
  vector<vector<double> >       constraints_;
};

VelProfileS::VelProfileS(unsigned int ndof, double dt, vector<vector<double> > constraints)
  : ndof_(ndof), dt_(dt), constraints_(constraints)
{
  try
  {
    if(constraints.size() != ndof_){
      throw invalid_argument("ndof not equals the dimention of constraints!");
    }
  }
  catch(const exception& e)
  {
    cerr << e.what() << '\n';
  }

  RML = new ReflexxesAPI(ndof_, dt_);
  IP  = new RMLPositionInputParameters(ndof_);
  OP  = new RMLPositionOutputParameters(ndof_);

  for (unsigned int i = 0; i < ndof_; i++)
  {
    IP->CurrentPositionVector->VecData[i] = 0.0;
    IP->CurrentVelocityVector->VecData[i] = 0.0;
    IP->CurrentAccelerationVector->VecData[i] = 0.0;

    // Conresponding to Acceleration actually
    IP->MaxVelocityVector->VecData[i] = constraints_[i][1];
    // Conresponding to Jerk actually
    IP->MaxAccelerationVector->VecData[i] = constraints_[i][2];
    IP->MaxJerkVector->VecData[i] = 0.0;
    
    IP->SelectionVector->VecData[i] = true;
  }

  Flags.SynchronizationBehavior = RMLFlags::NO_SYNCHRONIZATION;
}

VelProfileS::~VelProfileS()
{
  delete RML;
  delete IP;
  delete OP;
}

int VelProfileS::update(vector<double> &vel_target, vector<double> &vel_planned)
{
  if(vel_target.size() != ndof_ || vel_planned.size() != ndof_){
    return -1;
  }

  // vector<double> vel_trgt(vel_target.begin(), vel_target.end());
  // vector<double> vel_trgt(ndof_);
  for (unsigned int i = 0; i < ndof_; i++){
    IP->TargetPositionVector->VecData[i] = max(-constraints_[i][0], min(vel_target[i], constraints_[i][0]));
    IP->TargetVelocityVector->VecData[i] = 0.0;
  }

  res_ = RML->RMLPosition(*IP, OP, Flags);

  if (res_ < 0)
  {
    printf("An error occurred (%d).\n", res_ );
    return -2;
  }
  
  *IP->CurrentPositionVector      =   *OP->NewPositionVector      ;
  *IP->CurrentVelocityVector      =   *OP->NewVelocityVector      ;
  *IP->CurrentAccelerationVector  =   *OP->NewAccelerationVector  ;
  
  for (unsigned int i = 0; i < ndof_; i++)
  {
    vel_planned[i] = OP->NewPositionVector->VecData[i];
  }

  return 0;
}

#endif // !VEL_PROFILE_S_HPP