#include <stdio.h>
#include <ros/ros.h>
#include <ros/time.h>
// #include <cmath>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#include <planning/vel_profile_S.h>

#define     sign(x)     (((x) < 0) ? -1 : 1)
#define     nDOF        3
#define     dT          0.01

double MAX_VEL[nDOF]    = {1.0, 1.0, 0.5} ;
double target_vel[nDOF] = {.0, .0, .0}    ;

double gen_rand_angle(int l, int u, double precision=1e-6){
    return double(rand()%(u-l) + l + precision*(rand() % int(1 / precision)));
}

void targetCB(const std_msgs::Float64MultiArrayConstPtr& msg){
  int size = sizeof(msg->data)/sizeof(msg->data[0]);
  ROS_ERROR_COND(size != nDOF, "Wrong size of target velocities, which should be %d!", nDOF);
  for(int i=0; i < size; i++){
    target_vel[i] = std::max(-MAX_VEL[i], std::min(msg->data[i], MAX_VEL[i]));
  }  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "otg_jl_test");
  ros::NodeHandle nh;

  ros::Subscriber sub_targetVel  = nh.subscribe("target_vel", 5, targetCB);
  ros::Publisher pub_plannedVel  = nh.advertise<std_msgs::Float64MultiArray>("planned_vel", 5);
  ros::Publisher pub_plannedAcc  = nh.advertise<std_msgs::Float64MultiArray>("planned_acc", 5);
  ros::Publisher pub_plannedJerk = nh.advertise<std_msgs::Float64MultiArray>("planned_jerk", 5);
  ros::Publisher pub_refVel = nh.advertise<std_msgs::Float64>("ref_vel", 1);
  ros::Publisher pub_plannedPos = nh.advertise<std_msgs::Float64>("planned_pos", 1);

  std_msgs::Float64MultiArray planned_vel, planned_acc, planned_jerk;
  planned_vel.data.assign(nDOF, .0);
  planned_acc.data.assign(nDOF, .0);
  planned_jerk.data.assign(nDOF, .0);

  // ********************************************************************
  // Creating all relevant objects of the Type II Reflexxes Motion Library

  int                  ResultValue  =   0;

  ReflexxesAPI                *RML  =   new ReflexxesAPI( nDOF,  dT );

  RMLPositionInputParameters  *IP   =   new RMLPositionInputParameters( nDOF );

  RMLPositionOutputParameters *OP   =   new RMLPositionOutputParameters( nDOF );

  RMLPositionFlags            Flags;

  // ********************************************************************
  // Set-up the input parameters

  // Conresponding to Velocity actually
  IP->CurrentPositionVector->VecData      [0] =    0.0      ;
  IP->CurrentPositionVector->VecData      [1] =    0.0      ;
  IP->CurrentPositionVector->VecData      [2] =    0.0      ;

  // Conresponding to Acceleration actually
  IP->CurrentVelocityVector->VecData      [0] =    0.0      ;
  IP->CurrentVelocityVector->VecData      [1] =    0.0      ;
  IP->CurrentVelocityVector->VecData      [2] =    0.0      ;

  // Conresponding to Jerk actually
  IP->CurrentAccelerationVector->VecData  [0] =    0.0      ;
  IP->CurrentAccelerationVector->VecData  [1] =    0.0      ;
  IP->CurrentAccelerationVector->VecData  [2] =    0.0      ;

  // Conresponding to Acceleration actually
  IP->MaxVelocityVector->VecData          [0] =    5.0*MAX_VEL[0] ;
  IP->MaxVelocityVector->VecData          [1] =    5.0*MAX_VEL[1] ;
  IP->MaxVelocityVector->VecData          [2] =    5.0*MAX_VEL[2] ;

  // Conresponding to Jerk actually
  IP->MaxAccelerationVector->VecData      [0] =    25.0*MAX_VEL[0] ;
  IP->MaxAccelerationVector->VecData      [1] =    25.0*MAX_VEL[1] ;
  IP->MaxAccelerationVector->VecData      [2] =    25.0*MAX_VEL[2] ;

  IP->MaxJerkVector->VecData              [0] =    0.0*MAX_VEL[0];
  IP->MaxJerkVector->VecData              [1] =    0.0*MAX_VEL[1];
  IP->MaxJerkVector->VecData              [2] =    0.0*MAX_VEL[2];

  IP->SelectionVector->VecData            [0] =    true       ;
  IP->SelectionVector->VecData            [1] =    true       ;
  IP->SelectionVector->VecData            [2] =    true       ;

  Flags.SynchronizationBehavior   =   RMLFlags::NO_SYNCHRONIZATION;

  // for testing VelProfileS
  std::vector<std::vector<double> > constraints{{MAX_VEL[0], 5.0*MAX_VEL[0], 10.0*MAX_VEL[0]}};
  VelProfileS vel_planner(1, dT, constraints);

  // ********************************************************************
  // Starting the control loop

  ros::Rate loop_rate(1.0/dT);
  ros::Duration sim_time;
  ros::Time start_time = ros::Time::now();
  std_msgs::Float64 ref_vel, planned_pos;
  planned_pos.data = -3.0;
  while(ros::ok())
  {
    // Auto-generate velocity command for test
    sim_time = ros::Time::now() - start_time;
    ref_vel.data = sign(std::sin(0.5*sim_time.toSec()))*1.0 + 0.0*std::sin(0.5*sim_time.toSec()) + 0.00*gen_rand_angle(-1, 1);
    // ref_vel.data = gen_rand_angle(-2, 2);

    IP->TargetPositionVector->VecData[0] = ref_vel.data ;
    // IP->TargetPositionVector->VecData[0] = target_vel[0] ;
    IP->TargetPositionVector->VecData[1] = target_vel[1] ;
    IP->TargetPositionVector->VecData[2] = target_vel[2] ;

    IP->TargetVelocityVector->VecData[0] = 0.0 ;
    IP->TargetVelocityVector->VecData[1] = 0.0 ;
    IP->TargetVelocityVector->VecData[2] = 0.0 ;

    // Calling the Reflexxes OTG algorithm
    ResultValue =   RML->RMLPosition(       *IP
                                        ,   OP
                                        ,   Flags       );

    if (ResultValue < 0)
    {
        printf("An error occurred (%d).\n", ResultValue );
        break;
    }

    *IP->CurrentPositionVector      =   *OP->NewPositionVector      ;
    *IP->CurrentVelocityVector      =   *OP->NewVelocityVector      ;
    *IP->CurrentAccelerationVector  =   *OP->NewAccelerationVector  ;

    // for testing VelProfileS
    std::vector<double> vel_trgt{ref_vel.data};
    std::vector<double> vel_planned(1);
    vel_planner.update(vel_trgt, vel_planned);

    planned_vel.data[0] = OP->NewPositionVector->VecData[0];
    planned_vel.data[1] = OP->NewPositionVector->VecData[1];
    planned_vel.data[2] = OP->NewPositionVector->VecData[2];

    planned_pos.data += planned_vel.data[0] * dT;

    planned_acc.data[0] = OP->NewVelocityVector->VecData[0];
    planned_acc.data[1] = OP->NewVelocityVector->VecData[1];
    planned_acc.data[2] = OP->NewVelocityVector->VecData[2];

    planned_jerk.data[0] = vel_planned[0];  // for testing VelProfileS
    // planned_jerk.data[0] = OP->NewAccelerationVector->VecData[0];
    planned_jerk.data[1] = OP->NewAccelerationVector->VecData[1];
    planned_jerk.data[2] = OP->NewAccelerationVector->VecData[2];

    pub_plannedVel.publish(planned_vel);
    pub_plannedAcc.publish(planned_acc);
    pub_plannedJerk.publish(planned_jerk);
    pub_refVel.publish(ref_vel);
    pub_plannedPos.publish(planned_pos);
    // ROS_INFO_THROTTLE(1, "Planned Velocity = %5.2f, %5.2f, %5.2f",
    //                       planned_vel.data[0],
    //                       planned_vel.data[1],
    //                       planned_vel.data[2]);

    ros::spinOnce();
    loop_rate.sleep();

  }
  // ********************************************************************
  // Deleting the objects of the Reflexxes Motion Library and terminating
  // the process

  delete  RML         ;
  delete  IP          ;
  delete  OP          ;

  exit(EXIT_SUCCESS)  ;

}
