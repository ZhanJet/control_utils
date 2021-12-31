/**
 Copyright (c) 2021, Zhangjie Tu.
 All rights reserved.
  @author zhanjet
 */

#ifndef EXECUTION_TIMER_H
#define EXECUTION_TIMER_H

#include <iostream>
#include <sstream>
#include <chrono>
// #include <type_traits>
#include <typeinfo>
#include <string>

template<class Resolution = std::chrono::milliseconds>
class ExecutionTimer {
public:
    // using Clock = std::conditional<std::chrono::high_resolution_clock::is_steady,
    //                                  std::chrono::high_resolution_clock,
    //                                  std::chrono::steady_clock>;
    using Clock = std::conditional_t<std::chrono::high_resolution_clock::is_steady,
                                     std::chrono::high_resolution_clock,
                                     std::chrono::steady_clock>;
private:
    Clock::time_point T_start = Clock::now();
    std::string res_name;

public:
    // ExecutionTimer() = default;
    ExecutionTimer(){
        if(typeid(Resolution).name()==typeid(std::chrono::seconds).name()){
            res_name = " seconds";
        }
        if(typeid(Resolution).name()==typeid(std::chrono::milliseconds).name()){
            res_name = " milliseconds";
        }
        if(typeid(Resolution).name()==typeid(std::chrono::microseconds).name()){
            res_name = " microseconds";
        }
        if(typeid(Resolution).name()==typeid(std::chrono::nanoseconds).name()){
            res_name = " nanoseconds";
        }
    }

    ~ExecutionTimer() {
        const auto end = Clock::now();
        std::ostringstream strStream;
        strStream << "Destructor Elapsed: "
                  << std::chrono::duration_cast<Resolution>( end - T_start ).count()
                  << res_name
                  << std::endl;
        std::cout << strStream.str() << std::endl;
    }

    inline void start() {
        std::cout << "Start Timing..." << std::endl;
        T_start = Clock::now();
    }

    inline void stop() {
        const auto end = Clock::now();
        std::ostringstream strStream;
        strStream << "Stop Elapsed: "
                  << std::chrono::duration_cast<Resolution>(end - T_start).count()
                //   << typeid(Resolution).name()
                  << res_name
                  << std::endl;
        std::cout << strStream.str() << std::endl;
    }

}; // ExecutionTimer

#endif // EXECUTION_TIMER_H
