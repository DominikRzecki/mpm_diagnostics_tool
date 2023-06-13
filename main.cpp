#include <iostream>
#include <filesystem>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdio_ext.h>

struct __attribute__((packed)) timestep_t {
    int32_t missed;
    float voltage;
    float current;
    float phase;
};

int main(int argc, const char* argv[]) {
    std::filesystem::path path{argv[1]};

    std::ios::sync_with_stdio(false);

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    auto serial_port = open(path.c_str(), O_RDWR /* O_NONBLOCK | O_NDELAY*/ ); //
    auto fp = fdopen(serial_port, "rw");
    if (serial_port == -1) {
        std::cerr<<"File does not exist, or inadequate permissions\n";
        close(serial_port);
        return 1;
    }

    if(tcgetattr(serial_port, &tty) != 0) {
        std::cerr<<"Error " << errno <<" from tcgetattr: "<<strerror(errno) << '\n';
        close(serial_port);
        return 1;
    }

    //cfmakeraw(&tty);

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 0;
    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B921600);
    cfsetospeed(&tty, B921600);


    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr<<"Error " << errno <<" from tcgetattr: "<<strerror(errno) << '\n';
        close(serial_port);
        return 1;
    }

    char c;
    timestep_t tmp {0};
    char ack_byte {'#'};
    size_t read_cnt = 0;

    while(true) {

        fread(&c, 1, 1, fp);
        //read(serial_port, &c, 1); // -- Erzeugt manchmal störungen, würde gerne wissen wieso?
        if(c == '#') {
            //read(serial_port, (void *)&tmp, 13);
            fread((void*)&tmp, 1, 16, fp);
            std::cout<<"missed: "<<tmp.missed<<"\nvoltage: "<<tmp.voltage<<"\ncurrent: "<<tmp.current<<"\nphi: "<<tmp.phase<<"\nPtot: "<<tmp.voltage/sqrt(2) * tmp.current/sqrt(2)<<"\nPw: "<<abs(tmp.voltage / sqrt(2.00) * tmp.current / sqrt(2.00) * cos(tmp.phase))<<'\n';
            std::cout.flush();
        } else if(c == '@'){
            do {
                fread(&c, 1, 1, fp);
                std::cout<<c;
            } while(false);
            std::cout<<'\n';
        }
    }
}