#include "linux_serial.hpp"
#include <time.h>
#include <stdint.h>
  
void delay(int number_of_seconds)
{
    // Converting time into milli_seconds
    int milli_seconds = 1000 * number_of_seconds;
  
    // Storing start time
    clock_t start_time = clock();
  
    // looping till required time is not achieved
    while (clock() < start_time + milli_seconds)
        ;
}

int main() {
    serial.init("/dev/ttyS12", B115200);
    while (1)
    {
        serial.send((uint8_t*)"HALO!/n", 6);
        delay(1);
    }
    
}

// hallo update

