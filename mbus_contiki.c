#include <string.h>
#include <stdio.h>

#define MBUS_FRAME_PURGE_S2M  2
#define MBUS_FRAME_PURGE_M2S  1
#define MBUS_FRAME_PURGE_NONE 0

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






int mbus_contiki_scan(int baudrate) {
  mbus_handle *handle;
  char *device;
  //int *device;
  //device = RS232_PORT_1;
  int address, retries = 0;
  int baudrate = 9600;

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
    }






    mbus_disconnect(handle);
    mbus_context_free(handle);
    return 0;
}
