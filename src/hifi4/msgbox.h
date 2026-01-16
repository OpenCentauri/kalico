#ifndef __HIFI4_MSGBOX_H
#define __HIFI4_MSGBOX_H

#include <stdint.h>

extern uint16_t msgbox_new_msg[2];

int msgbox_hw_init(void);
void msgbox_send_signal(uint32_t data);

#endif // msgbox.h
