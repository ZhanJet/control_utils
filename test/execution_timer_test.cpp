#include "utils/execution_timer.h"

int main() {
    { // empty scope to display ExecutionTimer's destructor's message
         // displayed in milliseconds
         ExecutionTimer<std::chrono::milliseconds> timer;
        //  timer.start();

         // function or code block here

         timer.stop();
    }

    { // same as above
        ExecutionTimer<std::chrono::microseconds> timer;
        // timer.start();

        // code block here...

        timer.stop();
    }

    {  // same as above
       ExecutionTimer<std::chrono::nanoseconds> timer;
    //    timer.start();

       // code block here...

       timer.stop();
    }

    {  // same as above
       ExecutionTimer<std::chrono::seconds> timer;
    //    timer.start();

       // code block here...

       timer.stop();
    }

    return 0;
}