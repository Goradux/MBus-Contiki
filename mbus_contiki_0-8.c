#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include <stdio.h>
#include <math.h>

#include <time.h>
#include <stdlib.h>

#include <termios.h>





#include <unistd.h>
#include <fcntl.h>

#include <sys/types.h>

#include <stdio.h>
#include <strings.h>

#include <termios.h>
#include <errno.h>
#include <string.h>



#define MBUS_FRAME_TYPE_ANY     0x00
#define MBUS_FRAME_TYPE_ACK     0x01
#define MBUS_FRAME_TYPE_SHORT   0x02
#define MBUS_FRAME_TYPE_CONTROL 0x03
#define MBUS_FRAME_TYPE_LONG    0x04


#define MBUS_FRAME_ACK_START     0xE5
#define MBUS_FRAME_SHORT_START   0x10
#define MBUS_FRAME_CONTROL_START 0x68
#define MBUS_FRAME_LONG_START    0x68
#define MBUS_FRAME_STOP          0x16



#define MBUS_FRAME_ACK_BASE_SIZE        1
#define MBUS_FRAME_SHORT_BASE_SIZE      5
#define MBUS_FRAME_CONTROL_BASE_SIZE    9
#define MBUS_FRAME_LONG_BASE_SIZE       9



#define MBUS_MAX_PRIMARY_SLAVES 256

//
// Control field
//
#define MBUS_CONTROL_FIELD_DIRECTION    0x07
#define MBUS_CONTROL_FIELD_FCB          0x06
#define MBUS_CONTROL_FIELD_ACD          0x06
#define MBUS_CONTROL_FIELD_FCV          0x05
#define MBUS_CONTROL_FIELD_DFC          0x05
#define MBUS_CONTROL_FIELD_F3           0x04
#define MBUS_CONTROL_FIELD_F2           0x03
#define MBUS_CONTROL_FIELD_F1           0x02
#define MBUS_CONTROL_FIELD_F0           0x01

#define MBUS_CONTROL_MASK_SND_NKE       0x40
#define MBUS_CONTROL_MASK_SND_UD        0x53
#define MBUS_CONTROL_MASK_REQ_UD2       0x5B
#define MBUS_CONTROL_MASK_REQ_UD1       0x5A
#define MBUS_CONTROL_MASK_RSP_UD        0x08

#define MBUS_CONTROL_MASK_FCB           0x20
#define MBUS_CONTROL_MASK_FCV           0x10

#define MBUS_CONTROL_MASK_ACD           0x20
#define MBUS_CONTROL_MASK_DFC           0x10

#define MBUS_CONTROL_MASK_DIR           0x40
#define MBUS_CONTROL_MASK_DIR_M2S       0x40
#define MBUS_CONTROL_MASK_DIR_S2M       0x00



#define PACKET_BUFF_SIZE 2048



#define MBUS_FRAME_BASE_SIZE_ACK       1
#define MBUS_FRAME_BASE_SIZE_SHORT     5
#define MBUS_FRAME_BASE_SIZE_CONTROL   9
#define MBUS_FRAME_BASE_SIZE_LONG      9



#define MBUS_HANDLE_TYPE_SERIAL 1



typedef struct _mbus_frame {

    u_char start1;
    u_char length1;
    u_char length2;
    u_char start2;
    u_char control;
    u_char address;
    u_char control_information;
    // variable data field
    u_char checksum;
    u_char stop;

    u_char   data[252];
    size_t data_size;

    int type;
    time_t timestamp;

    void *next;

} mbus_frame;



typedef struct _mbus_serial_handle {

    char *device;
    //int *device;
    // ^^^ RS232_PORT_1

    int fd;
    // ^^^ comment out later

    struct termios t;
    // ^^^ baudrate

} mbus_serial_handle;



typedef struct _mbus_handle {
    char is_serial;                           /**< _handle type (non zero for serial) */
    mbus_serial_handle * m_serial_handle; /**< Serial gateway handle */
} mbus_handle;



mbus_serial_handle *
mbus_serial_connect(char *device)
{
    mbus_serial_handle *handle;

    if (device == NULL)
    {
        return NULL;
    }

    if ((handle = (mbus_serial_handle *)malloc(sizeof(mbus_serial_handle))) == NULL)
    {
        printf("failed to allocate memory for handle\n");
        return NULL;
    }

    handle->device = device;

    // Use blocking read and handle it by serial port VMIN/VTIME setting
    if ((handle->fd = open(handle->device, O_RDWR | O_NOCTTY)) < 0)
    {
        printf("failed to open tty.\n");
        return NULL;
    }

    memset(&(handle->t), 0, sizeof(handle->t));
    handle->t.c_cflag |= (CS8|CREAD|CLOCAL);
    handle->t.c_cflag |= PARENB;

    // No received data still OK
    handle->t.c_cc[VMIN]  = 0;

    handle->t.c_cc[VTIME] = 2; // Timeout in 1/10 sec

    cfsetispeed(&(handle->t), B9600);
    cfsetospeed(&(handle->t), B9600);

    tcsetattr(handle->fd, TCSANOW, &(handle->t));

    return handle;
}



mbus_handle *
mbus_connect_serial(const char * device)
{
    mbus_serial_handle * serial_handle;
    if ((serial_handle = mbus_serial_connect((char*)device)) == NULL)
    {
        printf("Failed to setup serial connection to M-bus gateway on ???.\n");
        return NULL;
    }

    mbus_handle * handle;
    if ((handle = (mbus_handle * ) malloc(sizeof(mbus_handle))) == NULL)
    {
        printf("Failed to allocate handle.\n");
        return NULL;
    }
    handle->is_serial = 1;
    handle->m_serial_handle = serial_handle;
    return handle;
}












mbus_frame *
mbus_frame_new(int frame_type)
{
    mbus_frame *frame = NULL;

    if ((frame = malloc(sizeof(mbus_frame))) != NULL)
    {
        memset((void *)frame, 0, sizeof(mbus_frame));

        frame->type = frame_type;
        switch (frame->type)
        {
            case MBUS_FRAME_TYPE_ACK:

                frame->start1 = MBUS_FRAME_ACK_START;

                break;

            case MBUS_FRAME_TYPE_SHORT:

                frame->start1 = MBUS_FRAME_SHORT_START;
                frame->stop   = MBUS_FRAME_STOP;

                break;
        }
    }

    return frame;
}



u_char
calc_checksum(mbus_frame *frame)
{
    size_t i;
    u_char cksum;

    switch(frame->type)
    {
        case MBUS_FRAME_TYPE_SHORT:

            cksum = frame->control;
            cksum += frame->address;

            break;

        case MBUS_FRAME_TYPE_ACK:
        default:
            cksum = 0;
    }

    return cksum;
}



int
mbus_frame_calc_checksum(mbus_frame *frame)
{
    if (frame)
    {
        switch (frame->type)
        {
            case MBUS_FRAME_TYPE_ACK:
            case MBUS_FRAME_TYPE_SHORT:
                frame->checksum = calc_checksum(frame);

                break;

            default:
                return -1;
        }
    }

    return 0;
}



int
mbus_frame_pack(mbus_frame *frame, u_char *data, size_t data_size)
{
    size_t i, offset = 0;

    if (frame && data)
    {


        if (mbus_frame_calc_checksum(frame) == -1)
        {
            return -3;
        }

        switch (frame->type)
        {
            case MBUS_FRAME_TYPE_ACK:

                if (data_size < MBUS_FRAME_ACK_BASE_SIZE)
                {
                    return -4;
                }

                data[offset++] = frame->start1;

                return offset;

            case MBUS_FRAME_TYPE_SHORT:

                if (data_size < MBUS_FRAME_SHORT_BASE_SIZE)
                {
                    return -4;
                }

                data[offset++] = frame->start1;
                data[offset++] = frame->control;
                data[offset++] = frame->address;
                data[offset++] = frame->checksum;
                data[offset++] = frame->stop;

                return offset;

            default:
                return -5;
        }
    }

    return -1;
}



void (*_mbus_send_event)(u_char src_type, const char *buff, size_t len) = NULL;
void (*_mbus_recv_event)(u_char src_type, const char *buff, size_t len) = NULL;


int
mbus_serial_send_frame(mbus_serial_handle *handle, mbus_frame *frame)
{
    u_char buff[PACKET_BUFF_SIZE];
    int len, ret;

    if (handle == NULL || frame == NULL)
    {
        return -1;
    }

    if ((len = mbus_frame_pack(frame, buff, sizeof(buff))) == -1)
    {
        printf("mbus_frame_pack failed\n");
        return -1;
    }


    if ((ret = write(handle->fd, buff, len)) == len)
    // ^^^ change
    {
        //
        // call the send event function, if the callback function is registered
        //
        if (_mbus_send_event)
                _mbus_send_event(MBUS_HANDLE_TYPE_SERIAL, buff, len);
    }
    else
    {
        printf("Failed to write frame to socket.\n");
        return -1;
    }

    //
    // wait until complete frame has been transmitted
    //
    tcdrain(handle->fd);
    // ^^^ add sleep???

    return 0;
}



int
mbus_send_frame(mbus_handle * handle, mbus_frame *frame)
{
    if (handle == NULL)
    {
        printf("Invalid M-Bus handle for send.\n");
        return 0;
    }

    if (handle->is_serial)
    {
        return mbus_serial_send_frame(handle->m_serial_handle, frame);
    }
    return 0;
}



//------------------------------------------------------------------------------
/// Free the memory resources allocated for the M-Bus frame data structure.
//------------------------------------------------------------------------------
int
mbus_frame_free(mbus_frame *frame)
{
    if (frame)
    {
        if (frame->next != NULL)
            mbus_frame_free(frame->next);

        free(frame);
        return 0;
    }
    return -1;
}



//------------------------------------------------------------------------------
// send a data request packet to from master to slave
//------------------------------------------------------------------------------
int
mbus_send_ping_frame(mbus_handle *handle, int address)
{
    int retval = 0;
    mbus_frame *frame;

    frame = mbus_frame_new(MBUS_FRAME_TYPE_SHORT);

    if (frame == NULL)
    {
        printf("failed to allocate mbus frame.\n");
        return -1;
    }

    frame->control  = MBUS_CONTROL_MASK_SND_NKE | MBUS_CONTROL_MASK_DIR_M2S;
    frame->address  = address;

    if (mbus_send_frame(handle, frame) == -1)
    {
        printf("failed to send mbus frame.\n");
        retval = -1;
    }

    mbus_frame_free(frame);
    return retval;
}

































//------------------------------------------------------------------------------
/// Verify that parsed frame is a valid M-bus frame.
//
// Possible checks:
//
// 1) frame type
// 2) Start/stop bytes
// 3) control field
// 4) length field and actual data size
// 5) checksum
//
//------------------------------------------------------------------------------
int
mbus_frame_verify(mbus_frame *frame)
{
    u_char checksum;

    if (frame)
    {
        switch (frame->type)
        {
            case MBUS_FRAME_TYPE_ACK:
                return frame->start1 == MBUS_FRAME_ACK_START;

            case MBUS_FRAME_TYPE_SHORT:
                if(frame->start1 != MBUS_FRAME_SHORT_START)
                {
                    printf("No frame start\n");

                    return -1;
                }

                if ((frame->control !=  MBUS_CONTROL_MASK_SND_NKE)                          &&
                    (frame->control !=  MBUS_CONTROL_MASK_REQ_UD1)                          &&
                    (frame->control != (MBUS_CONTROL_MASK_REQ_UD1 | MBUS_CONTROL_MASK_FCB)) &&
                    (frame->control !=  MBUS_CONTROL_MASK_REQ_UD2)                          &&
                    (frame->control != (MBUS_CONTROL_MASK_REQ_UD2 | MBUS_CONTROL_MASK_FCB)))
                {
                    printf("Unknown Control Code.\n");

                    return -1;
                }

                break;

            default:
                printf("Unknown frame type\n");

                return -1;
        }

        if(frame->stop != MBUS_FRAME_STOP)
        {
            printf("No frame stop.\n");

            return -1;
        }

        checksum = calc_checksum(frame);

        if(frame->checksum != checksum)
        {
            printf("Invalid checksum.\n");

            return -1;
        }

        return 0;
    }

    printf("Got null pointer to frame.\n");

    return -1;
}



//------------------------------------------------------------------------------
/// PARSE M-BUS frame data structures from binary data.
//------------------------------------------------------------------------------
int
mbus_parse(mbus_frame *frame, u_char *data, size_t data_size)
{
    size_t i, len;

    if (frame && data && data_size > 0)
    {

        switch (data[0])
        {
            case MBUS_FRAME_ACK_START:

                // OK, got a valid ack frame, require no more data
                frame->start1   = data[0];
                frame->type = MBUS_FRAME_TYPE_ACK;
                return 0;
                //return MBUS_FRAME_BASE_SIZE_ACK - 1; // == 0

            case MBUS_FRAME_SHORT_START:

                if (data_size < MBUS_FRAME_BASE_SIZE_SHORT)
                {
                    // OK, got a valid short packet start, but we need more data
                    return MBUS_FRAME_BASE_SIZE_SHORT - data_size;
                }

                if (data_size != MBUS_FRAME_BASE_SIZE_SHORT)
                {
                    printf("Too much data in frame.\n");
                    // too much data... ?
                    return -2;
                }

                // init frame data structure
                frame->start1   = data[0];
                frame->control  = data[1];
                frame->address  = data[2];
                frame->checksum = data[3];
                frame->stop     = data[4];

                frame->type = MBUS_FRAME_TYPE_SHORT;

                // verify the frame
                if (mbus_frame_verify(frame) != 0)
                {
                    return -3;
                }

                // successfully parsed data
                return 0;

            default:
                printf("Invalid M-Bus frame start.\n");

                // not a valid M-Bus frame header (start byte)
                return -4;
        }

    }


    printf("Got null pointer to frame, data or zero data_size.\n");

    return -1;
}



//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
int
mbus_serial_recv_frame(mbus_serial_handle *handle, mbus_frame *frame)
{
    char buff[PACKET_BUFF_SIZE];
    int len, remaining, nread, timeouts;

    if (handle == NULL || frame == NULL)
    {
        printf("Invalid parameter.\n");
        return -1;
    }

    memset((void *)buff, 0, sizeof(buff));

    //
    // read data until a packet is received
    //
    remaining = 1; // start by reading 1 byte
    len = 0;
    timeouts = 0;

    do {

        if ((nread = read(handle->fd, &buff[len], remaining)) == -1)
        // ^^^ use our read
        {
            printf("-1!\n");
            return -1;
        }

        if (nread == 0)
        {
            timeouts++;

            if (timeouts >= 3)
            {
                // abort to avoid endless loop
                printf("Timeout!\n");
                break;
            }
        }
        len += nread;

    } while ((remaining = mbus_parse(frame, buff, len)) > 0);

    if (len == 0)
    {
        // No data received
        return -1;
    }

    //
    // call the receive event function, if the callback function is registered
    //
    if (_mbus_recv_event)
        _mbus_recv_event(MBUS_HANDLE_TYPE_SERIAL, buff, len);

    if (remaining != 0)
    {
        // Would be OK when e.g. scanning the bus, otherwise it is a failure.
        // printf("%s: M-Bus layer failed to receive complete data.\n", __PRETTY_FUNCTION__);
        return -2;
    }

    if (len == -1)
    {
        printf("M-Bus layer failed to parse data.\n");
        return -1;
    }

    return 0;
}



int
mbus_recv_frame(mbus_handle * handle, mbus_frame *frame)
{
    int result = 0;

    if (handle == NULL)
    {
        printf("Invalid M-Bus handle for receive.\n");
        return 0;
    }

    if (handle->is_serial)
    {
        result = mbus_serial_recv_frame(handle->m_serial_handle, frame);
    }

    if (frame != NULL)
    {
        /* set timestamp to receive time */
        time(&(frame->timestamp));
        // ^^^ ?????
    }

    return result;
}








int
mbus_serial_disconnect(mbus_serial_handle *handle)
{
    if (handle == NULL)
    {
        return -1;
    }

    close(handle->fd);
    // ^^^ ???

    free(handle);

    return 0;
}

int
mbus_disconnect(mbus_handle * handle)
{
    if (handle == NULL)
    {
        printf("Invalid M-Bus handle for disconnect.\n");
        return 0;
    }

    if (handle->is_serial)
    {
        mbus_serial_disconnect(handle->m_serial_handle);
        handle->m_serial_handle = NULL;
    }

    free(handle);
    return 0;
}










int
mbus_serial_set_baudrate(mbus_serial_handle *handle, int baudrate)
{
    speed_t speed;

    if (handle == NULL)
        return -1;

    switch (baudrate)
    {
        case 300:
            speed = B300;
            handle->t.c_cc[VTIME] = 12; // Timeout in 1/10 sec
            break;

        case 1200:
            speed = B1200;
            handle->t.c_cc[VTIME] = 4;  // Timeout in 1/10 sec
            break;

        case 2400:
            speed = B2400;
            handle->t.c_cc[VTIME] = 2;  // Timeout in 1/10 sec
            break;

        case 9600:
            speed = B9600;
            handle->t.c_cc[VTIME] = 1;  // Timeout in 1/10 sec
            break;

       default:
            return -1; // unsupported baudrate
    }

    // Set input baud rate
    if (cfsetispeed(&(handle->t), speed) != 0)
    {
        return -1;
    }

    // Set output baud rate
    if (cfsetospeed(&(handle->t), speed) != 0)
    {
        return -1;
    }

    // Change baud rate immediately
    if (tcsetattr(handle->fd, TCSANOW, &(handle->t)) != 0)
    {
        return -1;
    }

    return 0;
}




















//------------------------------------------------------------------------------
/// Return the M-Bus frame type
//------------------------------------------------------------------------------
int
mbus_frame_type(mbus_frame *frame)
{
    if (frame)
    {
        return frame->type;
    }
    return -1;
}



int contiki_mbus_serial_scan(char * input) {
  mbus_handle *handle;
  char *device = input;
  // int *device;
  // ^^^ RS232_PORT_1
  int address = 67;
  int baudrate = 9600;
  int ret;

  if ((handle = mbus_connect_serial(device)) == NULL)
  {
      printf("Failed to setup connection to M-bus gateway\n");
      return 1;
  }

  if (mbus_serial_set_baudrate(handle->m_serial_handle, baudrate) == -1)
    {
        printf("Failed to set baud rate.\n");
        return 1;
    }


  mbus_frame reply;

  memset((void *)&reply, 0, sizeof(mbus_frame));


  if (mbus_send_ping_frame(handle, address) == -1)
  {
      printf("Scan failed. Could not send ping frame.\n");
      return 1;
  }

  ret = mbus_recv_frame(handle, &reply);


  if (ret == -2)
  {
      /* check for more data (collision) */
      while (mbus_recv_frame(handle, &reply) != -1);

      printf("Collision at address.\n");

  }

  if (mbus_frame_type(&reply) == MBUS_FRAME_TYPE_ACK)
  {
      /* check for more data (collision) */
      while (mbus_recv_frame(handle, &reply) != -1)
      {
          ret = -2;
      }

      if (ret == -2)
      {
          printf("Collision at address.\n");

      }

      printf("Found a M-Bus device at address.\n");
  }


  return 0;
}

int main(int argc, char **argv) {
  char *input = argv[1];
  contiki_mbus_serial_scan(input);
  return 0;
}
