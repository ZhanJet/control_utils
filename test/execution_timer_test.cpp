#include "utils/execution_timer.h"
#include "unistd.h"
#include <thread>

using std::this_thread::sleep_for;

int main() {
    { // empty scope to display ExecutionTimer's destructor's message
         // displayed in milliseconds
         ExecutionTimer<std::chrono::milliseconds> timer;
         timer.start();

         // function or code block here
         sleep_for(std::chrono::milliseconds(3000));
         

         timer.stop();
    }

    { // same as above
        ExecutionTimer<std::chrono::microseconds> timer;
        // timer.start();

        // code block here...
        usleep(1000);


        timer.stop();
    }

    {  // same as above
       ExecutionTimer<std::chrono::nanoseconds> timer;
    //    timer.start();

       // code block here...
       sleep_for(std::chrono::nanoseconds(3000000));

       timer.stop();
    }

    {  // same as above
       ExecutionTimer<std::chrono::seconds> timer;
    //    timer.start();

       // code block here...
       sleep(3);

       timer.stop();
    }

    return 0;
}