#include <stdio.h>
#include <string.h>
#include <iostream>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

using namespace std;

int serial_port;

void close_port_handler(int signum) {
    cout << "Exiting serial port test..." << endl;
    exit(signum);
}

int main(int argc, char const *argv[])
{
    signal(SIGINT, close_port_handler);
    cout << "Running serial port tester..." << endl;
    
    serial_port = open("/dev/ttyS0", O_RDWR);
    if (serial_port < 0) {
        cout << "Error " << errno << " from open: " << strerror(errno);
    };

    struct termios tty;

    if(tcgetattr(serial_port, &tty) != 0) {
        cout << "Error " << errno << " from tcgetattr: " << strerror(errno);
    }

    struct termios {
        tcflag_t c_iflag;
        tcflag_t c_oflag;
        tcflag_t c_cflag;
        tcflag_t c_lflag;
        cc_t c_line;
        cc_t c_cc[NCCS];
    };

    // 8N1
    tty.c_cflag &= ~PARENB; // No parity bit
    tty.c_cflag &= ~CSTOPB; // One stop bit
    tty.c_cflag |= CS8; // 8 bits per byte

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 50;
    tty.c_cc[VMIN] = 1980;

    cfsetspeed(&tty, B115200);

    if(tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        cout << "Error " << errno << " from tcsetattr: " << strerror(errno);
    }
    uint8_t temp_char;
    uint8_t start_count = 0;
    bool got_scan = false;

    uint8_t read_buf [1980];
    uint8_t good_sets = 0;
    uint32_t motor_speed = 0;
    int index;

    while(true) {
        int n = read(serial_port, &read_buf[start_count], 1);

        // if(n < 0) {
        //     printf("Error reading: %s", strerror(errno));
        // }
        // cout << "Read " << n << " bytes. Recieved message: " << read_buf;

        if(start_count == 0) {
            if(read_buf[start_count] == 0xFA) {
                start_count = 1;
            }
        } else if(start_count == 1) {
            if(read_buf[start_count] == 0xA0) {
                start_count = 0;

                // Now that entire start sequence has been found, read in the rest of the message
                got_scan = true;

                read(serial_port, &read_buf[2], 1978);
                // cout << "Read " << n << " bytes. Recieved message: " << read_buf;

                // scan->angle_min = 0.0;
                // scan->angle_max = 2.0*M_PI;
                // scan->angle_increment = (2.0*M_PI/360.0);
                // scan->range_min = 0.06;
                // scan->range_max = 5.0;
                // scan->ranges.resize(360);
                // scan->intensities.resize(360);

                //read data in sets of 4
                for(uint16_t i = 0; i < sizeof(read_buf); i=i+22) {
                    if(read_buf[i] == 0xFA && read_buf[i+1] == (0xA0+i/22)) {//&& CRC check
                        good_sets++;
                        motor_speed += (read_buf[i+3] << 8) + read_buf[i+2]; //accumulate count for avg. time increment
                            // rpms=(read_buf[i+3]<<8|read_buf[i+2])/64; 
                        cout << (read_buf[i+3]<<8|read_buf[i+2])/64 << " rpms" << endl;

                        for(uint16_t j = i+4; j < i+20; j=j+4) {
                            index = (4*i)/22 + (j-4-i)/4;
                            // Four bytes per reading
                            uint8_t byte0 = read_buf[j];
                            uint8_t byte1 = read_buf[j+1];
                            uint8_t byte2 = read_buf[j+2];
                            uint8_t byte3 = read_buf[j+3];
                            // First two bits of byte1 are status flags
                            // uint8_t flag1 = (byte1 & 0x80) >> 7;  // No return/max range/too low of reflectivity
                            // uint8_t flag2 = (byte1 & 0x40) >> 6;  // Object too close, possible poor reading due to proximity kicks in at < 0.6m
                            // Remaining bits are the range in mm
                            uint16_t range = ((byte1 & 0x3F)<< 8) + byte0;
                            // Last two bytes represent the uncertanty or intensity, might also be pixel area of target...
                            uint16_t intensity = (byte3 << 8) + byte2;

                            //   scan  ->ranges[index] = range / 1000.0;
                            //   scan->intensities[index] = intensity;
                            cout << "Bit " << index << " intensity: " << intensity << endl;
                        }
                    }
                }

            // scan->time_increment = motor_speed/good_sets/1e8;
            }   
        }
    }

    close(serial_port);

    return 0;
}
