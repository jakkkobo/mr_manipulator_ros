/************************************************************************
 *                                                                      *
 * Author: Matias   Project: Turttle    Date:23-04-2015     Version:1.0 *
 * Lab: Autonomous Systems Laboratory                  CROB-INESC PORTO *
 *                                                                      *
 ************************************************************************/
#include "../include/mr_communication/mts_serial.h"
#include "../include/mr_communication/mts_out.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/signal.h>
#include <sys/select.h>
//low latency
#include <sys/ioctl.h>
#include <linux/serial.h>

static void setSignalHandlerIO(void *signal_handler_IO);
static int iOpenDev(Serial_t *serial_t);
static bool bSetupDev(Serial_t *serial_t, int fd);

Out OUT;

bool bInitDev(Serial_t *serial_t)
{
    memcpy(serial_t->dev,MTS_SERIAL_DEV,MTS_SERIAL_SIZE_BUFFER_DEV);
    serial_t->baudrate=MTS_SERIAL_BAUD_RATE;
    serial_t->data_bits=MTS_SERIAL_DATA_BITS;
    serial_t->stop_bits=MTS_SERIAL_STOP_BITS;
    serial_t->parity=MTS_SERIAL_PARITY;
    serial_t->hard_flow_control=MTS_SERIAL_HFLOW_CONTROL;
    serial_t->signal_handler_IO=NULL;
    serial_t->is_close=true;
    return true;
}

bool bConnect(Serial_t *serial_t)
{
    if((serial_t->fd=iOpenDev(serial_t))<0) return false;
    serial_t->is_close=false;
    if(!bSetupDev(serial_t,serial_t->fd)) return false;
    return true;
}

bool bTryReconnet(Serial_t *serial_t, int max_ntimes)
{
    int i=0;
    for(i=0;i<max_ntimes;i++)
    {
        vCloseDev(serial_t);
        sleep(MTS_SERIAL_TIME_TRY_RECONNECT_SEC);
        if(bConnect(serial_t)){
          serial_t->is_close=false;
          return true;
        }
    }
    return false;
}

bool bSetDev(Serial_t *serial_t, const char* dev)
{
    size_t size=strlen(dev);
    if(size<1 || size>255)
    {
        // OUT(OUT_WARNING,"%s The name of device is to big or too small",MTS_SERIAL_MSG);
        return false;
    }

    memcpy(serial_t->dev,dev,size);
    serial_t->dev[size]='\0';
    return true;
}

inline ssize_t sReadDev(Serial_t *serial_t, void *buffer, size_t size)
{
   fd_set rfds;
   int retval;
   struct timeval time_out;

   FD_ZERO(&rfds);
   FD_SET(serial_t->fd, &rfds);

   time_out.tv_sec = MTS_SERIAL_READ_TIME_OUT_SEC;
   time_out.tv_usec = 0;
   retval = select(serial_t->fd + 1, &rfds, NULL, NULL, &time_out);
   if (retval>0)
   {
       if(FD_ISSET(serial_t->fd, &rfds))
       {
           retval=read(serial_t->fd,buffer,size);
           if(retval==0)
           {
               OUT(OUT_ERROR,"%s Connection losted", MTS_SERIAL_MSG);
               if(!bTryReconnet(serial_t, MTS_SERIAL_NTIMES_RECONNECT)) OUT(OUT_FATAL,"%s Impossible to reconnect", MTS_SERIAL_MSG);
           }
           else if(retval<0)  OUT(OUT_FATAL,"%s Fail to read, error unknown", MTS_SERIAL_MSG);
       }
   }
   else
   {
       if (retval == -1 && serial_t->is_close==false)    OUT(OUT_FATAL,"%s Lost file descriptor",MTS_SERIAL_MSG);
       //else if (retval == 0)    OUT(OUT_WARNING,"%s Timeout...",MTS_SERIAL_MSG);
   }
   return retval;
}

inline ssize_t sWriteDev(Serial_t *serial_t, void *buffer, size_t size){
  return write(serial_t->fd,buffer,size);
}
void vSetBaudRate(Serial_t *serial_t, int32_t baud_rate)
{
    serial_t->baudrate=baud_rate;
}

void vSetDataBits(Serial_t *serial_t, int8_t data_bits)
{
    serial_t->data_bits=data_bits;
}

void vSetStopBits(Serial_t *serial_t, int8_t stop_bits)
{
    serial_t->stop_bits=stop_bits;
}

void vSetParity(Serial_t *serial_t, int8_t patity)
{
    serial_t->parity=patity;
}

void vSetHardFlowControl(Serial_t *serial_t, int8_t hard_flow_control)
{
    serial_t->hard_flow_control=hard_flow_control;
}

void vSetSignalHandler(Serial_t *serial_t, void *signal_handler_IO)
{
    serial_t->signal_handler_IO=signal_handler_IO;
}

static int iOpenDev(Serial_t *serial_t)
{
  int fd; /* File descriptor for the port */

  struct serial_struct serial;
  fd = open(serial_t->dev, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);    // O_RDWR - Read and write    // O_NOCTTY - Ignore special chars like CTRL-C
  if (fd == -1)
  {
    /* Could not open the port. */
    OUT(OUT_ERROR, "%s Could not open device %s",MTS_SERIAL_MSG, serial_t->dev);
    return (-1);
  }
  //Low latency
  ioctl(fd, TIOCGSERIAL, &serial);
  serial.flags |= ASYNC_LOW_LATENCY; // (0x2000)
  ioctl(fd, TIOCSSERIAL, &serial);

  if(serial_t->signal_handler_IO!=NULL)
  {
      setSignalHandlerIO(serial_t->signal_handler_IO);
      /* allow the process to receive SIGIO (This step is necessary for the kernel to know just whom to notify)*/
      fcntl(fd, F_SETOWN, getpid());
      /* Make the file descriptor asynchronous (the manual page says only
      O_APPEND and O_NONBLOCK, will work with F_SETFL...) */
      fcntl(fd, F_SETFL, FASYNC);
  }
  else  fcntl(fd, F_SETFL, 0);

  return (fd);
}

static bool bSetupDev(Serial_t *serial_t, int fd)
{
    struct termios new_port_settings;

    memset(&new_port_settings, 0, sizeof(new_port_settings));

    if (!isatty(fd))
    {

        OUT(OUT_FATAL, "%s File descriptor %d is NOT a serial port",MTS_SERIAL_MSG, fd);
        return false;
    }
    if (tcgetattr(fd, &(serial_t->old_port_settings)) < 0)
    {
        OUT(OUT_ERROR, "%s Could not read configuration of fd %d",MTS_SERIAL_MSG, fd);
        return false;
    }
    //
    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    //
    new_port_settings.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    //
    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    //
    new_port_settings.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

    //Disavle Map lowercase characters to uppercase on output
    #ifdef OLCUC
        new_port_settings.c_oflag &= ~OLCUC;
    #endif

    #ifdef ONOEOT
        new_port_settings.c_oflag &= ~ONOEOT;
    #endif


    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    //
    new_port_settings.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    //
    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    //
    //Enable Receiver READ
    new_port_settings.c_cflag |= (CLOCAL | CREAD); //These will ensure that your program does not become the 'owner' of the port subject to sporatic job control and hangup signals, and also that the serial interface driver will read incoming data bytes.
    //Not bit mask for data bits
    new_port_settings.c_cflag &= ~CSIZE;
    //
    // One input byte is enough to return from read()
    // Inter-character timer off
    //
    new_port_settings.c_cc[VMIN] = 1;
    new_port_settings.c_cc[VTIME] = 0; // was 0

    switch(serial_t->parity)
    {
        case MTS_SERIAL_NONE:
            new_port_settings.c_cflag &= ~PARENB;
            new_port_settings.c_cflag &= ~CSTOPB;
        break;
        case MTS_SERIAL_EVEN:
            new_port_settings.c_cflag |= PARENB;
            new_port_settings.c_cflag &= ~PARODD;
        break;
        case MTS_SERIAL_ODD:
            new_port_settings.c_cflag &=PARENB;
            new_port_settings.c_cflag |= PARODD;
        break;
        default:
            OUT(OUT_ERROR, "%s Desired parity %d could not be set, aborting.",MTS_SERIAL_MSG, serial_t->parity);
            return false;
        break;
   }

    switch(serial_t->data_bits)
    {
        case 5:
            new_port_settings.c_cflag |= CS5;
            break;
        case 6:
            new_port_settings.c_cflag |= CS6;
            break;
        case 7:
            new_port_settings.c_cflag |= CS7;
            break;
        case 8:
            new_port_settings.c_cflag |= CS8;
            break;

        default:
            OUT(OUT_ERROR, "%s Desired data bits %d could not be set, aborting.",MTS_SERIAL_MSG, serial_t->data_bits);
            return false;
            break;
    }

    switch(serial_t->stop_bits)
    {
        case 1:
            new_port_settings.c_cflag &= ~CSTOPB;
            break;
        case 2:
            new_port_settings.c_cflag &= CSTOPB;
            break;
        default:
            OUT(OUT_ERROR, "%s Desired stop bits %d could not be set, aborting.",MTS_SERIAL_MSG, serial_t->stop_bits);
            return false;
            break;
    }

    switch(serial_t->hard_flow_control)
    {
        case MTS_SERIAL_OFF:
            new_port_settings.c_cflag &= ~CRTSCTS;
            break;
        case MTS_SERIAL_ON:
            new_port_settings.c_cflag |= CRTSCTS;
            break;
        default:
            OUT(OUT_ERROR, "%s Desired hardware flow control %d could not be set, aborting.",MTS_SERIAL_MSG, serial_t->hard_flow_control);
            return false;
        break;
    }

    switch (serial_t->baudrate)
    {
    case 1200:
        if (cfsetispeed(&new_port_settings, B1200) < 0 || cfsetospeed(&new_port_settings, B1200) < 0)
        {
            OUT(OUT_ERROR, "%s Could not set desired baud rate of %d Baud",MTS_SERIAL_MSG,  serial_t->baudrate);
            return false;
        }
        break;
    case 1800:
        cfsetispeed(&new_port_settings, B1800);
        cfsetospeed(&new_port_settings, B1800);
        break;
    case 9600:
        cfsetispeed(&new_port_settings, B9600);
        cfsetospeed(&new_port_settings, B9600);
        break;
    case 19200:
        cfsetispeed(&new_port_settings, B19200);
        cfsetospeed(&new_port_settings, B19200);
        break;
    case 38400:
        if (cfsetispeed(&new_port_settings, B38400) < 0 || cfsetospeed(&new_port_settings, B38400) < 0)
        {
            OUT(OUT_ERROR, "%s Could not set desired baud rate of %d Baud", MTS_SERIAL_MSG, serial_t->baudrate);
            return false;
        }
        break;
    case 57600:
        if (cfsetispeed(&new_port_settings, B57600) < 0 || cfsetospeed(&new_port_settings, B57600) < 0)
        {
            OUT(OUT_ERROR, "%s Could not set desired baud rate of %d Baud",MTS_SERIAL_MSG, serial_t->baudrate);
            return false;
        }
        break;
    case 115200:
        if (cfsetispeed(&new_port_settings, B115200) < 0 || cfsetospeed(&new_port_settings, B115200) < 0)
        {
            OUT(OUT_ERROR, "%s Could not set desired baud rate of %d Baud",MTS_SERIAL_MSG, serial_t->baudrate);
            return false;
        }
        break;
    case 230400:
        if (cfsetispeed(&new_port_settings, B230400) < 0 || cfsetospeed(&new_port_settings, B230400) < 0)
        {
            OUT(OUT_ERROR, "%s Could not set desired baud rate of %d Baud",MTS_SERIAL_MSG, serial_t->baudrate);
            return false;
        }
        break;

        // These two non-standard (by the 70'ties ) rates are fully supported on
        // current Debian and Mac OS versions (tested since 2010).
    case 460800:
        if (cfsetispeed(&new_port_settings, B460800) < 0 || cfsetospeed(&new_port_settings, B460800) < 0)
        {
            OUT(OUT_ERROR, "%s Could not set desired baud rate of %d Baud",MTS_SERIAL_MSG, serial_t->baudrate);
            return false;
        }
        break;
    case 921600:
        if (cfsetispeed(&new_port_settings, B921600) < 0 || cfsetospeed(&new_port_settings, B921600) < 0)
        {
            OUT(OUT_ERROR, "%s Could not set desired baud rate of %d Baud",MTS_SERIAL_MSG, serial_t->baudrate);
            return false;
        }
        break;
    case 1000000:
        if (cfsetispeed(&new_port_settings, B1000000) < 0 || cfsetospeed(&new_port_settings, B1000000) < 0)
        {
            OUT(OUT_ERROR, "%s Could not set desired baud rate of %d Baud",MTS_SERIAL_MSG, serial_t->baudrate);
            return false;
        }
        break;
    case 2000000:
        if (cfsetispeed(&new_port_settings, B2000000) < 0 || cfsetospeed(&new_port_settings, B2000000) < 0)
        {
            OUT(OUT_ERROR, "%s Could not set desired baud rate of %d Baud",MTS_SERIAL_MSG, serial_t->baudrate);
            return false;
        }
        break;
    case 3000000:
        if (cfsetispeed(&new_port_settings, B3000000) < 0 || cfsetospeed(&new_port_settings, B3000000) < 0)
        {
            OUT(OUT_ERROR, "%s Could not set desired baud rate of %d Baud",MTS_SERIAL_MSG, serial_t->baudrate);
            return false;
        }
        break;
    case 4000000:
        if (cfsetispeed(&new_port_settings, B4000000) < 0 || cfsetospeed(&new_port_settings, B4000000) < 0)
        {
            OUT(OUT_ERROR, "%s Could not set desired baud rate of %d Baud",MTS_SERIAL_MSG, serial_t->baudrate);
            return false;
        }
        break;
    default:
        OUT(OUT_ERROR, "%s Desired baud rate %d could not be set, aborting.",MTS_SERIAL_MSG, serial_t->baudrate);
        return false;

        break;
    }
    if (tcsetattr(fd, TCSAFLUSH, &new_port_settings) < 0)
    {
        OUT(OUT_ERROR, "%s could not set configuration of fd %d",MTS_SERIAL_MSG, fd);
        return false;
    }
    return true;
}

void vCloseDev(Serial_t *serial_t)
{
  if(serial_t->fd>0)
  {
    close(serial_t->fd);
    serial_t->is_close=true;
    tcsetattr(serial_t->fd, TCSANOW, &(serial_t->old_port_settings));
    OUT(OUT_INFO,"%s The device %s is closed",MTS_SERIAL_MSG, serial_t->dev);

  }
  else
  {
      tcsetattr(serial_t->fd, TCSANOW, &(serial_t->old_port_settings));
      OUT(OUT_ERROR,"%s The device %s cant't be closed",MTS_SERIAL_MSG, serial_t->dev);
  }
}

static void setSignalHandlerIO(void *signal_handler_IO)
{
    struct sigaction saio; /* definition of signal action */
    /* install the signal handler before making the device asynchronous */
    saio.sa_handler = (void*)signal_handler_IO;
    sigemptyset(&saio.sa_mask);
    saio.sa_flags = 0;
    saio.sa_restorer = NULL;
    sigaction(SIGIO,&saio,NULL);
}
