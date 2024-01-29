/************************************************************************
 *                                                                      *
 * Author: Matias   Project: Turttle    Date:23-04-2015     Version:1.0 *
 * Lab: Autonomous Systems Laboratory                  CROB-INESC PORTO *
 *                                                                      *
 ************************************************************************/

#ifndef MTS_SERIAL_H
#define MTS_SERIAL_H


#include <stdbool.h>
#include <stdint.h>
#include <termios.h>
#include <sys/types.h>


#define MTS_SERIAL_NONE                     0
#define MTS_SERIAL_ODD                      1
#define MTS_SERIAL_EVEN                     2
#define MTS_SERIAL_ON                       1
#define MTS_SERIAL_OFF                      0
#define MTS_SERIAL_DEV                      "/dev/ttyUSB0"
#define MTS_SERIAL_BAUD_RATE                115200
#define MTS_SERIAL_DATA_BITS                8
#define MTS_SERIAL_STOP_BITS                1
#define MTS_SERIAL_PARITY                   MTS_SERIAL_NONE
#define MTS_SERIAL_HFLOW_CONTROL            MTS_SERIAL_OFF
#define MTS_SERIAL_TIME_TRY_RECONNECT_SEC   2
#define MTS_SERIAL_READ_TIME_OUT_SEC        1
#define MTS_SERIAL_NTIMES_RECONNECT         5
#define MTS_SERIAL_SIZE_BUFFER_DEV          256
#define MTS_SERIAL_MSG                      "SERIAL:"
/******************/
typedef struct
{
    int fd;
    char dev[MTS_SERIAL_SIZE_BUFFER_DEV];
    int32_t baudrate;
    int8_t data_bits;
    int8_t stop_bits;
    int8_t parity;
    int8_t hard_flow_control;
    void *signal_handler_IO;
    struct termios old_port_settings;
    bool is_close;
}Serial_t;


bool bInitDev(Serial_t *serial_t);
void vCloseDev(Serial_t *serial_t);
bool  bConnect(Serial_t *serial_t);
bool bTryReconnet(Serial_t *serial_t, int max_ntimes);
ssize_t sReadDev(Serial_t *serial_t, void *buffer, size_t size);
ssize_t sWriteDev(Serial_t *serial_t, void *buffer, size_t size);
bool bSetDev(Serial_t *serial_t, const char* dev);
void vSetBaudRate(Serial_t *serial_t, int32_t baud_rate);
void vSetDataBits(Serial_t *serial_t, int8_t data_bits);
void vSetStopBits(Serial_t *serial_t, int8_t stop_bits);
void vSetParity(Serial_t *serial_t, int8_t patity);
void vSetHardFlowControl(Serial_t *serial_t, int8_t hard_flow_control);
void vSetSignalHandler(Serial_t *serial_t, void *signal_handler_IO);

#endif // MTS_SERIAL_H
