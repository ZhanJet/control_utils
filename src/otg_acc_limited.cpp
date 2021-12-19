
#include <stdio.h>
#include <ros/ros.h>
#include <ros/time.h>
// #include <cmath>
#include <std_msgs/Float64MultiArray.h>

#include <ReflexxesAPI.h>
#include <RMLVelocityFlags.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>

#define     sign(x)     (((x) < 0) ? -1 : 1)
#define     nDOF        3
#define     dT          0.01

double MAX_VEL[nDOF]    = {1.0, 1.0, 0.5} ;
double target_vel[nDOF] = {.0, .0, .0}    ;

void targetCB(const std_msgs::Float64MultiArrayConstPtr& msg){
  int size = sizeof(msg->data)/sizeof(msg->data[0]);
  ROS_ERROR_COND(size != nDOF, "Wrong size of target velocities, which should be %d!", nDOF);
  for(int i=0; i < size; i++){
    target_vel[i] = std::max(-MAX_VEL[i], std::min(msg->data[i], MAX_VEL[i]));
  }  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "otg_al_test");
  ros::NodeHandle nh;

  ros::Subscriber sub_targetVel  = nh.subscribe("target_vel", 5, targetCB);
  ros::Publisher pub_plannedVel  = nh.advertise<std_msgs::Float64MultiArray>("planned_vel", 5);
  ros::Publisher pub_plannedAcc  = nh.advertise<std_msgs::Float64MultiArray>("planned_acc", 5);

  std_msgs::Float64MultiArray planned_vel, planned_acc;
  planned_vel.data.assign(nDOF, .0);
  planned_acc.data.assign(nDOF, .0);

  // ********************************************************************
  // Creating all relevant objects of the Type II Reflexxes Motion Library

  int                  ResultValue  =   0;

  ReflexxesAPI                *RML  =   new ReflexxesAPI( nDOF,  dT );

  RMLVelocityInputParameters  *IP   =   new RMLVelocityInputParameters( nDOF );

  RMLVelocityOutputParameters *OP   =   new RMLVelocityOutputParameters( nDOF );

  RMLVelocityFlags            Flags;

  // ********************************************************************
  // Set-up the input parameters

  IP->CurrentPositionVector->VecData      [0] =    0.0      ;
  IP->CurrentPositionVector->VecData      [1] =    0.0      ;
  IP->CurrentPositionVector->VecData      [2] =    0.0      ;

  IP->CurrentVelocityVector->VecData      [0] =    0.0      ;
  IP->CurrentVelocityVector->VecData      [1] =    0.0      ;
  IP->CurrentVelocityVector->VecData      [2] =    0.0      ;

  IP->CurrentAccelerationVector->VecData  [0] =    0.0      ;
  IP->CurrentAccelerationVector->VecData  [1] =    0.0      ;
  IP->CurrentAccelerationVector->VecData  [2] =    0.0      ;

  IP->MaxAccelerationVector->VecData      [0] =    1.0*MAX_VEL[0] ;
  IP->MaxAccelerationVector->VecData      [1] =    1.0*MAX_VEL[1] ;
  IP->MaxAccelerationVector->VecData      [2] =    1.0*MAX_VEL[2] ;

  IP->MaxJerkVector->VecData              [0] =    0.0*MAX_VEL[0];
  IP->MaxJerkVector->VecData              [1] =    0.0*MAX_VEL[1];
  IP->MaxJerkVector->VecData              [2] =    0.0*MAX_VEL[2];

  IP->SelectionVector->VecData            [0] =    true       ;
  IP->SelectionVector->VecData            [1] =    true       ;
  IP->SelectionVector->VecData            [2] =    true       ;

  Flags.SynchronizationBehavior   =   RMLFlags::NO_SYNCHRONIZATION;

  // ********************************************************************
  // Starting the control loop

  ros::Rate loop_rate(1.0/dT);
  while(ros::ok())
  {

    IP->TargetVelocityVector->VecData[0] = target_vel[0] ;
    IP->TargetVelocityVector->VecData[1] = target_vel[1] ;
    IP->TargetVelocityVector->VecData[2] = target_vel[2] ;

    // Calling the Reflexxes OTG algorithm
    ResultValue =   RML->RMLVelocity(       *IP
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

    planned_vel.data[0] = OP->NewVelocityVector->VecData[0];
    planned_vel.data[1] = OP->NewVelocityVector->VecData[1];
    planned_vel.data[2] = OP->NewVelocityVector->VecData[2];

    planned_acc.data[0] = OP->NewAccelerationVector->VecData[0];
    planned_acc.data[1] = OP->NewAccelerationVector->VecData[1];
    planned_acc.data[2] = OP->NewAccelerationVector->VecData[2];

    pub_plannedVel.publish(planned_vel);
    pub_plannedAcc.publish(planned_acc);
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
