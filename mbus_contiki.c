#include <string.h>
#include <stdio.h>
#include <unistd.h>

#define MBUS_FRAME_PURGE_S2M  2
#define MBUS_FRAME_PURGE_M2S  1
#define MBUS_FRAME_PURGE_NONE 0




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

#define MBUS_RECV_RESULT_OK        0
#define MBUS_RECV_RESULT_ERROR     -1
#define MBUS_RECV_RESULT_INVALID   -2
#define MBUS_RECV_RESULT_TIMEOUT   -3
#define MBUS_RECV_RESULT_RESET     -4

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


typedef struct _mbus_handle {
  int fd;
  int max_data_retry;
  int max_search_retry;
  char purge_first_frame;
  char is_serial;
  int (*open) (struct _mbus_handle *handle);
  int (*close) (struct _mbus_handle *handle);
  int (*send) (struct _mbus_handle *handle, mbus_frame *frame);
  int (*recv) (struct _mbus_handle *handle, mbus_frame *frame);
  void (*free_auxdata) (struct _mbus_handle *handle);
  void (*recv_event) (unsigned char src_type, const char *buff, size_t len);
  void (*send_event) (unsigned char src_type, const char *buff, size_t len);
  void (*scan_progress) (struct _mbus_handle *handle, const char *mask);
  void (*found_event) (struct _mbus_handle *handle, mbus_frame *frame);
  void *auxdata;
  } mbus_handle;




typedef struct _mbus_serial_data
{
    char *device;
    //int *device;
    //change ^^^ this to RS232_PORT_1
    struct termios t; //remove this
    // add some baud rate stuff here
} mbus_serial_data;




//------------------------------------------------------------------------------
/// Pack the M-bus frame into a binary string representation that can be sent
/// on the bus. The binary packet format is different for the different types
/// of M-bus frames.
//------------------------------------------------------------------------------
int mbus_frame_pack(mbus_frame *frame, unsigned char *data, size_t data_size) {
    size_t i, offset = 0;

    if (frame && data)
    {
        if (mbus_frame_calc_length(frame) == -1)
        {
            return -2;
        }

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

            case MBUS_FRAME_TYPE_CONTROL:

                if (data_size < MBUS_FRAME_CONTROL_BASE_SIZE)
                {
                    return -4;
                }

                data[offset++] = frame->start1;
                data[offset++] = frame->length1;
                data[offset++] = frame->length2;
                data[offset++] = frame->start2;

                data[offset++] = frame->control;
                data[offset++] = frame->address;
                data[offset++] = frame->control_information;

                data[offset++] = frame->checksum;
                data[offset++] = frame->stop;

                return offset;

            case MBUS_FRAME_TYPE_LONG:

                if (data_size < frame->data_size + MBUS_FRAME_LONG_BASE_SIZE)
                {
                    return -4;
                }

                data[offset++] = frame->start1;
                data[offset++] = frame->length1;
                data[offset++] = frame->length2;
                data[offset++] = frame->start2;

                data[offset++] = frame->control;
                data[offset++] = frame->address;
                data[offset++] = frame->control_information;

                for (i = 0; i < frame->data_size; i++)
                {
                    data[offset++] = frame->data[i];
                }

                data[offset++] = frame->checksum;
                data[offset++] = frame->stop;

                return offset;

            default:
                return -5;
        }
    }

    return -1;
}




int
mbus_serial_connect(mbus_handle *handle)
{
    mbus_serial_data *serial_data;
    const char *device;
    struct termios *term;

    if (handle == NULL)
        return -1;

    serial_data = (mbus_serial_data *) handle->auxdata;
    if (serial_data == NULL || serial_data->device == NULL)
        return -1;

    device = serial_data->device;
    term = &(serial_data->t);
    //
    // create the SERIAL connection
    //

    // Use blocking read and handle it by serial port VMIN/VTIME setting
    if ((handle->fd = open(device, O_RDWR | O_NOCTTY)) < 0)
    // ^^^^^ RS232_PORT_1
    {
        fprintf(stderr, "%s: failed to open tty.", __PRETTY_FUNCTION__);
        return -1;
    }

    //memset(term, 0, sizeof(*term));
    // term->c_cflag |= (CS8|CREAD|CLOCAL);
    // term->c_cflag |= PARENB;
    //
    // // No received data still OK
    // term->c_cc[VMIN] = (cc_t) 0;
    // term->c_cc[VTIME] = (cc_t) 3; // Timeout in 1/10 sec

    //cfsetispeed(term, B2400);
    //cfsetospeed(term, B2400);
    // ^^^^^ Baudrate stuff

    //tcsetattr(handle->fd, TCSANOW, term);

    return 0;
}



int mbus_serial_send_frame(mbus_handle *handle, mbus_frame *frame) {
    unsigned char buff[PACKET_BUFF_SIZE];
    int len, ret;

    if (handle == NULL || frame == NULL) {
        return -1;
    }

    // Make sure serial connection is open
    if (isatty(handle->fd) == 0) {
        return -1;
    }
    // ^^^^^^ instead of this use some sort of RS232_PORT_1 check
    // isatty() is a standard function btw



    if ((len = mbus_frame_pack(frame, buff, sizeof(buff))) == -1)
    {
        printf("mbus_frame_pack failed\n");
        return -1;
    }

    if ((ret = write(handle->fd, buff, len)) == len)
    // write is unistd.h stuff
    {
        //
        // call the send event function, if the callback function is registered
        //
        if (handle->send_event)
                handle->send_event(MBUS_HANDLE_TYPE_SERIAL, buff, len);
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
    // ^^^^^ termios.h stuff

    return 0;
}
//^^^^^^^^^^^^^^^^^^^ change this function
//check mbus_frame_pack


mbus_handle * mbus_context_serial(const char *device) {
  //mbus_handle * mbus_context_serial(const int *device) {
  mbus_handle *handle;
  mbus_serial_data *serial_data;
  char error_str[128];

  if ((handle = (mbus_handle *) malloc(sizeof(mbus_handle))) == NULL)
      {
          printf("Failed to allocate mbus_handle.\n");
          return NULL;
      }

  if ((serial_data = (mbus_serial_data *)malloc(sizeof(mbus_serial_data))) == NULL)
      {
          printf("Failed to allocate memory for serial data.\n");
          //mbus_error_str_set(error_str);
          free(handle);
          return NULL;
      }

  handle->max_data_retry = 3;
  handle->max_search_retry = 1;
  handle->is_serial = 1;
  handle->purge_first_frame = MBUS_FRAME_PURGE_M2S;       //find later
  handle->auxdata = serial_data;
  handle->open = mbus_serial_connect;                     //????
  handle->close = mbus_serial_disconnect;                 //????
  handle->recv = mbus_serial_recv_frame;
  handle->send = mbus_serial_send_frame;
  handle->free_auxdata = mbus_serial_data_free;
  handle->recv_event = NULL;
  handle->send_event = NULL;
  handle->scan_progress = NULL;
  handle->found_event = NULL;

  serial_data->device = device;
  //serial_data->device = RS232_PORT_1;

  return handle;
}




int mbus_connect(mbus_handle *handle) {
  if (handle == NULL)
    {
        printf("Invalid M-Bus handle.\n");
        return -1;
    }

    return handle->open(handle);
    //wrong ^^^
    // need to use RS232_PORT_1
}




int mbus_disconnect(mbus_handle *handle) {
    if (handle == NULL)
    {
      printf("Invalid M-Bus handle.\n");
      return -1;
    }

    return handle->close(handle);
    //wrong ^^^
    // RS232_PORT_1 something
}




int mbus_serial_set_baudrate(mbus_handle *handle, long baudrate) {
  mbus_serial_data *serial_data;

  if (handle == NULL)
    return -1;

  serial_data = (mbus_serial_data *) handle->auxdata;
  // ^^^ What is this??
  if (serial_data == NULL)
      return -1;

  switch (baudrate) {
    case 300:
      //fill later

    case 2400:
      //fill later

    case 9600:
      //fill later

    default:
      return -1;
  }

  //fill later
  //set baudrates
  //ask Robert for his SETBAUD() function

  return 0;
}




void mbus_serial_data_free(mbus_handle *handle) {
    mbus_serial_data *serial_data;

    if (handle) {
        serial_data = (mbus_serial_data *) handle->auxdata;

        if (serial_data == NULL)
          return;

        free(serial_data->device);
        //^^^ might not need to do that because device is fixed to RS232_PORT_1
        free(serial_data);
        //^^^ might not need to do that because serial data is just device and no termios
        handle->auxdata = NULL;
    }
}
void mbus_context_free(mbus_handle * handle) {
    if (handle) {
        handle->free_auxdata(handle);
        free(handle);
    }
}




#define MBUS_FRAME_DATA_LENGTH 252

typedef struct _mbus_frame {

    unsigned char start1;
    unsigned char length1;
    unsigned char length2;
    unsigned char start2;
    unsigned char control;
    unsigned char address;
    unsigned char control_information;
    // variable data field
    unsigned char checksum;
    unsigned char stop;

    unsigned char   data[MBUS_FRAME_DATA_LENGTH];
    size_t data_size;

    int type;
    time_t timestamp;

    //mbus_frame_data frame_data;

    void *next; // pointer to next mbus_frame for multi-telegram replies

} mbus_frame;

mbus_frame *mbus_frame_new(int frame_type);
int         mbus_frame_free(mbus_frame *frame);

mbus_frame *mbus_frame_new(int frame_type) {
    mbus_frame *frame = NULL;

    if ((frame = malloc(sizeof(mbus_frame))) != NULL)
    {
        memset((void *)frame, 0, sizeof(mbus_frame));
        //no idea if memset() works on Contiki

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

            case MBUS_FRAME_TYPE_CONTROL:
                frame->start1 = MBUS_FRAME_CONTROL_START;
                frame->start2 = MBUS_FRAME_CONTROL_START;
                frame->length1 = 3;
                frame->length2 = 3;
                frame->stop   = MBUS_FRAME_STOP;
                break;

            case MBUS_FRAME_TYPE_LONG:
                frame->start1 = MBUS_FRAME_LONG_START;
                frame->start2 = MBUS_FRAME_LONG_START;
                frame->stop   = MBUS_FRAME_STOP;
                break;
        }
    }
    return frame;
}
int mbus_frame_free(mbus_frame *frame) {
    if (frame)
    {
        if (frame->next != NULL)
            mbus_frame_free(frame->next);
        free(frame);
        return 0;
    }
    return -1;
}



//---------------------------------------------------------
// Checks if an integer is a valid primary address.
//---------------------------------------------------------
int mbus_is_primary_address(int value) {
    return ((value >= 0x00) && (value <= 0xFF));
}

int mbus_send_frame(mbus_handle * handle, mbus_frame *frame) {
    if (handle == NULL)
    {
        printf("Invalid M-Bus handle for send.\n");
        return 0;
    }
    return handle->send(handle, frame);
    // ^^^ find out this send()
}
//------------------------------------------------------------------------------
// send a data request packet to from master to slave and optional purge response
//------------------------------------------------------------------------------
int mbus_send_ping_frame(mbus_handle *handle, int address, char purge_response) {
    int retval = 0;
    mbus_frame *frame;

    if (mbus_is_primary_address(address) == 0)
    {
        printf("Invalid address: %d\n", address);
        return 1;
    }

    frame = mbus_frame_new(MBUS_FRAME_TYPE_SHORT);

    if (frame == NULL)
    {
        printf("Failed to allocate M-Bus frame.\n");
        return -1;
    }

    frame->control  = MBUS_CONTROL_MASK_SND_NKE | MBUS_CONTROL_MASK_DIR_M2S;
    frame->address  = address;

    if (mbus_send_frame(handle, frame) == -1)
    //^^^ check this
    {
        printf("Failed to send M-Bus frame.\n");
        mbus_frame_free(frame);
        return -1;
    }

    if (purge_response)
    {
        //mbus_purge_frames(handle);
        //purge frames here
    }

    mbus_frame_free(frame);
    return retval;
}

int mbus_recv_frame(mbus_handle * handle, mbus_frame *frame) {
    int result = 0;

    if (handle == NULL)
    {
        printf("Invalid M-Bus handle for receive.\n");
        return MBUS_RECV_RESULT_ERROR;
    }

    if (frame == NULL)
    {
        printf("Invalid frame.\n");
        return MBUS_RECV_RESULT_ERROR;
    }

    result = handle->recv(handle, frame);

    switch (mbus_frame_direction(frame))
    {
        case MBUS_CONTROL_MASK_DIR_M2S:
            if (handle->purge_first_frame == MBUS_FRAME_PURGE_M2S)
                result = handle->recv(handle, frame);  // purge echo and retry
            break;
        case MBUS_CONTROL_MASK_DIR_S2M:
            if (handle->purge_first_frame == MBUS_FRAME_PURGE_S2M)
                result = handle->recv(handle, frame);  // purge echo and retry
            break;
    }

    if (frame != NULL)
    {
        /* set timestamp to receive time */
        time(&(frame->timestamp));
    }

    return result;
}

int ping_address(mbus_handle *handle, mbus_frame *reply, int address) {
    int i, ret = MBUS_RECV_RESULT_ERROR;

    memset((void *)reply, 0, sizeof(mbus_frame));
    //^^^ no idea if Contiki supports memset()

    for (i = 0; i <= handle->max_search_retry; i++)
    {
        if (mbus_send_ping_frame(handle, address, 0) == -1)
        {
            fprintf(stderr,"Scan failed. Could not send ping frame: %s\n", mbus_error_str());
            printf("Scan failed. Could not send ping frame.\n");
            return MBUS_RECV_RESULT_ERROR;
        }

        ret = mbus_recv_frame(handle, reply);

        if (ret != MBUS_RECV_RESULT_TIMEOUT)
        {
            return ret;
        }
    }

    return ret;
}





/*

finish up mbus_recv_frame() and mbus_send_ping_frame()

*/














//main function
int mbus_contiki_scan(int baudrate) {
  mbus_handle *handle;
  char *device;
  //int *device;
  //device = RS232_PORT_1;
  int address, retries = 0;
  //int baudrate = 9600;
  int ret;

  if ((handle = mbus_context_serial(device)) == NULL)
    {
        printf("Scan failed: Could not initialize M-Bus context.\n");
        return -1;
    }

  if (mbus_connect(handle) == -1)
    {
      printf("Failed to connect M-Bus\n");
      return -1;
    }

  if (mbus_serial_set_baudrate(handle, baudrate) == -1)
    {
      printf("Failed to set baudrate.\n");
      return -1;
    }

    /* Main scanning part*/
    for (address = 0; address <= 250; address++) {
      mbus_frame reply;

      ret = ping_address(handle, &reply, address);

      if (ret == MBUS_RECV_RESULT_TIMEOUT)
            continue;

      if (ret == MBUS_RECV_RESULT_INVALID)
        {
            //purge frames here
            printf("Collision at address %d\n", address);
            continue;
        }

      if (mbus_frame_type(&reply) == MBUS_FRAME_TYPE_ACK)
        {
            /* check for more data (collision) */
            if (0 /*purge frames here*/)
            {
                printf("Collision at address %d\n", address);
                continue;
            }
            printf("Found a M-Bus device at address %d\n", address);
        }
    }

    mbus_disconnect(handle);
    mbus_context_free(handle);
    return 0;
}


int main(int argc, char **argv) {
  mbus_contiki_scan(argv[1]);
  return 0;
}
