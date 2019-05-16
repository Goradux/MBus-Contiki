#include <string.h>
#include <stdio.h>

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

mbus_handle * mbus_context_serial(const char *device) {
  
}
