#include <ros/ros.h>
#include <ros/time.h>
#include <cmath>
#include <std_msgs/Float64.h>
#include "filters/iir_filter.h"

double input;

void input_cb(const std_msgs::Float64::ConstPtr& msg) {
    input = msg->data;
}

double sign(double arg) {
    return (arg<0)?(-1):(1);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "filter_test");
    ros::NodeHandle nh;

    ros::Subscriber input_sub = nh.subscribe("filter_input", 1, input_cb);
    ros::Publisher filter_x_pub = nh.advertise<std_msgs::Float64>("filter_x", 1);
    ros::Publisher filter_dx_pub = nh.advertise<std_msgs::Float64>("filter_dx", 1);
    ros::Publisher deltat_pub = nh.advertise<std_msgs::Float64>("delta_t", 1);
    ros::Publisher setpoint_pub = nh.advertise<std_msgs::Float64>("setpoint", 1);

    ros::Rate rate(100.0);
    double cutoff_freq = 10.0;
    // double zeta = 1.0;
    // IirFilterBw2rd* filter = new IirFilterBw2rd(cutoff_freq, zeta);
    IirFilterBw* filter = new IirFilterBw(2, 100, cutoff_freq);
    ros::Time prev_time, start_time;
    ros::Duration delta_t, sim_time;
    std_msgs::Float64 x, dx, delta_t_topic;
    std_msgs::Float64 setpoint;
    // setpoint.data = 1.0;
    start_time = ros::Time::now();
    while (ros::ok())
    {
        ros::spinOnce();

        // calculate delta_t
        if(!prev_time.isZero()){
            delta_t = ros::Time::now() - prev_time;
            prev_time = ros::Time::now();
            if(0 == delta_t.toSec()){
                ROS_ERROR("delta_t is 0, skipping this loop.");
                continue;
            }
        } else {
            ROS_INFO("prev_time is 0, doing nothing");
            prev_time = ros::Time::now();
            continue;
        }

        // x.data = filter->update(delta_t.toSec(), input);
        // dx.data = filter->update_dx(true, delta_t.toSec(), input);
        sim_time = ros::Time::now() - start_time;
        setpoint.data = sign(std::sin(sim_time.toSec()))*1.0 + 0*std::sin(sim_time.toSec());
        // if(sim_time.toSec() / 2 < 1e-2) setpoint.data = 0.0 - setpoint.data;
        x.data = filter->update(delta_t.toSec(), setpoint.data);
        dx.data = filter->update_dx(true, delta_t.toSec(), setpoint.data);

        filter_x_pub.publish(x);
        filter_dx_pub.publish(dx);
        delta_t_topic.data = delta_t.toSec();
        deltat_pub.publish(delta_t_topic);
        setpoint_pub.publish(setpoint);

        rate.sleep();

    }
}