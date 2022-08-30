#include <unistd.h> 
#include <fcntl.h> 
#include <termios.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

void serial_setup(int _sfd);

int main(void)
{
    int sfd = open("/dev/serial0", O_RDWR | O_NOCTTY ); 
    if (sfd == -1){
        printf ("Error no is : %d\n", errno);
        printf("Error description is : %s\n",strerror(errno));
        return(-1);
    };

    serial_setup(sfd);

    char buf[]="hello world";
    char buf2[11];
    int count = write(sfd, buf, 11);
    count=read(sfd,buf2,11);buf2[11]=0;
    printf("%s",buf2);
    close(sfd);

    puts("Hello, World");
    
    return(0);
}

void serial_setup(int _sfd)
{
    puts("Serial Setup");
    // cfsetospeed(&options,B57600); //for output speed    
    // cfsetispeed(&options,B57600);  //for input speed
    // cfsetspeed(&options,B57600); //for both?

    struct termios options;
    tcgetattr(_sfd, &options);
    cfsetspeed(&options, B57600);
    options.c_cflag &= ~CSTOPB;
    options.c_cflag |= CLOCAL;
    options.c_cflag |= CREAD;
    cfmakeraw(&options);
    tcsetattr(_sfd, TCSANOW, &options);
}
