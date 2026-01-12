#ifndef __HIFI4_USB_BULK_H
#define __HIFI4_USB_BULK_H

#include "sharespace.h"
#include "command.h" // MESSAGE_MAX

struct receive_msg_block{
    int msg_len;
    char buf[MESSAGE_MAX];
    struct receive_msg_block *next;
};

struct receive_msg_queue {
    struct receive_msg_block *front, *rear;
};

void share_space_find_data_pack(void);
void receive_msg_queue_pop(struct receive_msg_queue* receive_msg, char* buf, int* len);
void receive_msg_queue_push(struct receive_msg_queue* receive_msg, char* buf, int* len);
int receive_msg_queue_isEmpty(struct receive_msg_queue* receive_msg);

#endif // usb_bulk.h
