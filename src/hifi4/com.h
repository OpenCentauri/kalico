#ifndef __HIFI4_COM_H
#define __HIFI4_COM_H

#include <stdint.h> // uint32_t

void rpmsg_notify_rx(const void *data, uint32_t len);

#endif // com.h
