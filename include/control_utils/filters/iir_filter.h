/**
 Copyright (c) 2021, Zhangjie Tu.
 All rights reserved.
  @author zhanjet
 */

#ifndef IIR_FILTER_HPP
#define IIR_FILTER_HPP

#include <math.h>
#include <vector>

class IirFilterBw
{
private:
    int order_;
    // cutoff_freq: default -1 corresponds to a cutoff frequency at 1/4 sampling rate
    double cutoff_freq_, zeta_;
    double c_, tan_flt_, sampling_rate_;
    std::vector<double> xk_, xk_filtered_, xk_diff_, xk_diff_filtered_;

public:
    // IirFilterBw2rd():cutoff_freq_(-1), xk_(3,0), xk_filtered_(3,0), xk_diff_(3,0), xk_diff_filtered_(3,0){}
    /**
     * @brief Construct a new IirFilterBw object 
     * @param order: order of iirfilter, value in 1 or 2
     * @param cutoff_freq: in Hz
     */
    IirFilterBw(int order, double sampling_rate, double cutoff_freq): 
                order_(order), sampling_rate_(sampling_rate), cutoff_freq_(cutoff_freq), zeta_(1.),
                xk_(order+1,0.), xk_filtered_(order+1,0.), xk_diff_(order+1,0.), xk_diff_filtered_(order+1,0.){}
    ~IirFilterBw(){}

    void setSamplingRate(double sampling_rate){
        sampling_rate_ = sampling_rate;
    }

    void setCutoffFreq(double cutoff_freq){
        cutoff_freq_ = cutoff_freq;
        if(cutoff_freq_ < 0.0) cutoff_freq_ = -1;
    }

    void setZeta(double zeta){
        zeta_ = zeta;
        if(zeta_ < 0.0) zeta_ = 1;
    }

    double update(const double delta_t, const double xk){
        if(cutoff_freq_ > 0.0){
            // tan_flt_ = tan((cutoff_freq_ * 2.0 * M_PI) * delta_t / 2.0);
            // tan_flt_ = tan(cutoff_freq_ * M_PI * delta_t);
            tan_flt_ = tan(cutoff_freq_ * M_PI / sampling_rate_);
            // check if tan(0)
            if(tan_flt_ < 0.01)
                tan_flt_ = 0.01;
            
            c_ = 1.0 / tan_flt_;
        } else {
            c_ = 1.0;
        }

        if(1 == order_) {
            xk_.at(1) = xk_.at(0);
            xk_.at(0) = xk;

            xk_filtered_.at(1) = xk_filtered_.at(0);
            xk_filtered_.at(0) = (1 / (1 + c_)) * (xk_.at(0) + xk_.at(1) - (1 - c_) * xk_filtered_.at(1));
        }

        if(2 == order_) {
            xk_.at(2) = xk_.at(1);
            xk_.at(1) = xk_.at(0);
            xk_.at(0) = xk;

            xk_filtered_.at(2) = xk_filtered_.at(1);
            xk_filtered_.at(1) = xk_filtered_.at(0);
            // xk_filtered_.at(0) = 
            //     (1 / (1 + c_ * c_ + 1.414 * c_)) * (xk_.at(2) + 2 * xk_.at(1) + xk_.at(0) -
            //                             (c_ * c_ - 1.414 * c_ + 1) * xk_filtered_.at(2) -
            //                             (-2 * c_ * c_ + 2) * xk_filtered_.at(1));
            xk_filtered_.at(0) = 
                (1 / (1 + c_ * c_ + 2*zeta_ * c_)) * (xk_.at(2) + 2 * xk_.at(1) + xk_.at(0) -
                                        (c_ * c_ - 2*zeta_ * c_ + 1) * xk_filtered_.at(2) -
                                        (-2 * c_ * c_ + 2) * xk_filtered_.at(1));
        }

        return xk_filtered_.at(0);
    }

    double update_dx(const bool updated, const double delta_t, const double xk){

        if(!updated){
            update(delta_t, xk);
        }

        if(1 == order_) {
            xk_diff_filtered_.at(0) = (xk_filtered_.at(0) - xk_filtered_.at(1)) / delta_t;
        }

        if(2 == order_) {
            xk_diff_filtered_.at(0) = (xk_filtered_.at(0) - xk_filtered_.at(1)) / delta_t;
            
            // xk_diff_.at(2) = xk_diff_.at(1);
            // xk_diff_.at(1) = xk_diff_.at(0);
            // xk_diff_.at(0) = (xk_.at(0) - xk_.at(1)) / delta_t;

            // xk_diff_filtered_.at(2) = xk_diff_filtered_.at(1);
            // xk_diff_filtered_.at(1) = xk_diff_filtered_.at(0);
            // // xk_diff_filtered_.at(0) = (1 / (1 + c_ * c_ + 1.414 * c_)) *
            // //                             (xk_diff_.at(2) + 2 * xk_diff_.at(1) + xk_diff_.at(0) -
            // //                             (c_ * c_ - 1.414 * c_ + 1) * xk_diff_filtered_.at(2) - (-2 * c_ * c_ + 2) * xk_diff_filtered_.at(1));
            
            // xk_diff_filtered_.at(0) = (1 / (1 + c_ * c_ + 2*zeta_ * c_)) *
            //                             (xk_diff_.at(2) + 2 * xk_diff_.at(1) + xk_diff_.at(0)
            //                             - (c_ * c_ - 2*zeta_ * c_ + 1) * xk_diff_filtered_.at(2)
            //                             - (-2 * c_ * c_ + 2) * xk_diff_filtered_.at(1));

        }
        
        return xk_diff_filtered_.at(0);
    }
};


// class IirFilterDynCutoff
// {
// private:

// public:
//     IirFilterDynCutoff(const int filter_order, const double sample_freq);
//     ~IirFilterDynCutoff();


// }

#endif
