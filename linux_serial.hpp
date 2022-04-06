#ifndef _LINUX_SERIAL_INCLUDED_
#define _LINUX_SERIAL_INCLUDED_

#include <stdint.h>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

class linux_serial{
    private:
        typedef enum {
            ERROR = 0,
            OK
        } status_Typedef;
        int serial_port;
    public:
        uint8_t buff[256];
        status_Typedef init(char* dir, speed_t speed);
        status_Typedef send(uint8_t* data, size_t data_length);
        status_Typedef receive(uint8_t* data, size_t max_lenght);

} serial;

#endif