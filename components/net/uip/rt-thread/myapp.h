
#ifndef __MY_APP_H__
#define __MY_APP_H__

#include <rtdef.h>

struct app_state 
{
    enum {CONNECTED, SEND_ACKED,CONNECTION_CLOSED} state;
    rt_uint8_t flag;
	rt_uint8_t *ptr;
	rt_uint8_t data_left;
};

typedef struct app_state uip_tcp_appstate_t; 
 
extern void myapp_call(void);
//#define UIP_APPCALL myapp_call

#endif
