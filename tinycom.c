// tinycom
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/select.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/types.h>
#include <pwd.h>

#define VERSION 1.0
#define MAXNAME 256

// constants and global variables
enum FILEOPERATIONTYPES { READ=0, WRITE };
const char *switchpositions[]={ "off", "on" };
const char *defaultserialportsettings[]={ "/dev/tnt1 serial device", "57600 baud rate", "8N1 read/stop bits, parity", "on hardware flow control", "off software flow control" }; // device, speed, readstopparity, hardware flow control, software flow control
char serialportsettings[5][MAXNAME];
struct termios options;

// function definitions
void setserialflags(struct termios *options);
int readwritefile(const char *file, int mode);
char *filterconfigline(char *line);

int main(int argc, char *argv[])
{
  char buffer[256], *bufptr, c, *myname=argv[0], configfile[MAXNAME];
  fd_set set, read_set;
  ssize_t nread, nwrite;
  int fd, fd_hwm = STDIN_FILENO, serial_port, operation=1;
  struct timeval timeout;
  // initialize the timeout data structure
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
  
   // read configuration file
   struct passwd *pw = getpwuid(getuid()); // read password entry
   sprintf(configfile, "%s/.tinycom", pw->pw_dir);
   if (argc>1) {
    if (argc==2)
     strcpy(configfile, argv[1]);
    else
   printf("usage: %s <config file> ..using %s\n", myname, configfile); }
   if ((readwritefile(configfile, READ))==0) {
    printf("unable to read/write configuration file\n");
   exit(EXIT_FAILURE); }
  
   // standard input handlers
   fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK); // unblock stdin
   tcgetattr( STDIN_FILENO, &options); // read current options
   options.c_lflag &= ~ICANON; // disable canonical line oriented input
   options.c_lflag &= ~ECHOCTL; // do not echo control chars as ^char, delete as ~?
   tcsetattr( STDIN_FILENO, TCSANOW, &options); // write back to stdin
   // serial port handlers
   if ((serial_port=open(serialportsettings[0], O_RDWR | O_NOCTTY | O_NDELAY))==-1) {
     printf("unable to open serial port %s\n", serialportsettings[0]);
   exit(EXIT_FAILURE); }
   fcntl (serial_port, F_SETFL, FNDELAY); // unblock serial port
   // get the current options for the port
   tcgetattr(serial_port, &options);
   // set serial flags and options according to default/file entries
   setserialflags(&options);
   // set the new options for the port
   tcsetattr(serial_port, TCSANOW, &options);
  
   // arrange file descriptors set
   FD_ZERO(&set); // zero the set
   FD_SET(STDIN_FILENO, &set); // add stdin
   FD_SET(serial_port, &set); // add serial port
   fd_hwm = serial_port; // raise highest fd value
   
   printf("tinycom v%.1f, port settings --> ", VERSION);
   for (int i=0;i<5;i++)
    printf("%s ", serialportsettings[i]);
   printf(" <--\n");
        
    while (operation) {
        
     read_set=set; // renew set for select 
     select(fd_hwm + 1, &read_set, NULL, NULL, &timeout); // see if fds are set
     for (fd = 0; fd <= fd_hwm; fd++)
      if (FD_ISSET(fd, &read_set)) {
       if (fd==STDIN_FILENO) { // read from keyboard
        nread=read(STDIN_FILENO, &c, 1);
        if (c==127) // remap DEL to BACKSPACE
         c=8;
        if (c==17) { // ctrl+q to hangup
         write(serial_port, "+++", 3);
        continue; }
        if (c=='\n')
         write(serial_port, "\r", 1); // add carriage return before newline
        write(serial_port, &c, 1);
       }
       if (fd==serial_port) { // read from serial port
        memset(buffer, 0, sizeof(buffer));
        bufptr = buffer;
        while ((nread = read(serial_port, bufptr, buffer + sizeof(buffer) - bufptr - 1)) > 0) {
         bufptr += nread;
         if (bufptr[-1] == '\n' || bufptr[-1] == '\r')
        break; }
        *bufptr = '\0';
        if (!strcmp(buffer, "\r\nOK\r\n") && c==17) // finish hang up
         write(serial_port, "ath0\r\n", 6);
        printf("%s", buffer); 
       }
      }
      usleep(150); // do not overtire the CPU
      
     }

  close(serial_port);

 return EXIT_SUCCESS;
}

// set flags for serial port
void setserialflags(struct termios *options)
{
   // set the baud rate
   int tbaudrate=atoi(serialportsettings[1]), baudrate;
   switch (tbaudrate) {
    case 0:
     baudrate=B0;
    break;
    case 50:
     baudrate=B50;
    break;
    case 75:
     baudrate=B75;
    break;
    case 110:
     baudrate=B110;
    break;
    case 134:
     baudrate=B134;
    break;
	case 150:
     baudrate=B150;
    break;
	case 200:
     baudrate=B200;
    break;
	case 300:
     baudrate=B300;
    break;
	case 600:
     baudrate=B600;
    break;
	case 1200:
     baudrate=B1200;
    break;
	case 2400:
     baudrate=B2400;
    break;
	case 4800:
     baudrate=B4800;
    break;
	case 9600:
     baudrate=B9600;
    break;
	case 38400:
     baudrate=B38400;
    break;
	case 57600:
     baudrate=B57600;
    break;
	case 115200:
     baudrate=B115200;
    break;
	case 230400:
     baudrate=B230400;
    break;
	case 460800:
     baudrate=B460800;
    break;
	case 500000:
     baudrate=B500000;
    break;
	case 576000:
     baudrate=B576000;
    break;
	case 921600:
     baudrate=B921600;
    break;
	case 1000000:
     baudrate=B1000000;
    break;
	case 1152000:
     baudrate=B1152000;
    break;
	case 1500000:
     baudrate=B1500000;
    break;
	case 2000000:
     baudrate=B2000000;
    break;
	case 2500000:
     baudrate=B2500000;
    break;
	case 3000000:
     baudrate=B3000000;
    break;
	case 3500000:
     baudrate=B3500000;
    break;
	case 4000000:
     baudrate=B4000000;
    break;
    default:
     baudrate=B57600;
     strcpy(serialportsettings[1], "57600");
   break; }
   cfsetispeed(options, baudrate);
   cfsetospeed(options, baudrate);
   // enable the receiver and set local mode
   options->c_cflag |= (CLOCAL | CREAD); // enable receiver, set local mode
   options->c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // diselect canonical, raw
   options->c_oflag &= ~OPOST; // diselect processed output
   // no parity, 8 read, 1 stop bits - most common
   if (strcmp(serialportsettings[2], "7E1") && strcmp(serialportsettings[2], "7O1") && strcmp(serialportsettings[2], "7S1")) {
    strcpy(serialportsettings[2], "8N1");
    options->c_cflag &= ~PARENB;
    options->c_cflag &= ~CSTOPB;
    options->c_cflag &= ~CSIZE;
   options->c_cflag |= CS8; }
   if (!strcmp(serialportsettings[2], "7E1")) {
    options->c_cflag |= PARENB;
    options->c_cflag &= ~PARODD;
    options->c_cflag &= ~CSTOPB;
   options->c_cflag |= CS7; }
   if (!strcmp(serialportsettings[2], "7O1")) {
    options->c_cflag |= PARENB;
    options->c_cflag |= PARODD;
    options->c_cflag &= ~CSTOPB;
   options->c_cflag |= CS7; }
   if (!strcmp(serialportsettings[2], "7S1")) {
    options->c_cflag &= ~PARENB;
    options->c_cflag &= ~CSTOPB;
   options->c_cflag |= CS7; }

   if (strcmp(serialportsettings[3], "off")) {
    strcpy(serialportsettings[3], "on");
   options->c_cflag |= CRTSCTS; } // hardware flow control on
   if (!strcmp(serialportsettings[3], "off"))
    options->c_cflag &= ~CRTSCTS;
   if (strcmp(serialportsettings[4], "on")) {
    strcpy(serialportsettings[4], "off");
   options->c_iflag &= ~(IXON | IXOFF | IXANY);  }// disable software flow control
   if (!strcmp(serialportsettings[4], "on"))
    options->c_iflag |= (IXON | IXOFF | IXANY);
}

// read/write config file
int readwritefile(const char *file, int mode)
{
  FILE *f;
  int i;
  size_t len=0;
  char *line=NULL;
  
   switch (mode) {
    case READ:
     if (!(f=fopen(file, "r"))) {
      for (i=0;i<5;i++)
       strcpy(serialportsettings[i], defaultserialportsettings[i]);
      i=readwritefile(file, WRITE);
     return i; }
     for (i=0;i<5;i++) {
      if ((getline(&line, &len, f))==-1)
       return 0;
      line[strlen(line)-1]='\0'; // remove \n
     strcpy(serialportsettings[i], filterconfigline(line)); }
    break;
    case WRITE:
     if (!(f=fopen(file, "w")))
      return 0;
     for (i=0;i<5;i++) {
      fprintf(f, "%s\n", serialportsettings[i]);
     strcpy(serialportsettings[i], filterconfigline(serialportsettings[i])); }
   break; }

  fclose(f);

 return 1;
}

// filter configuration line
char *filterconfigline(char *line)
{
  int i;
  
   for (i=0;i<strlen(line);i++)
    if (line[i]==';' || line[i]==' ')
     break;
   line[i]='\0';

 return &line[0];
}
