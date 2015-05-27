#ifndef    __UIP_RTT_H__
#define    __UIP_RTT_h__

#include <rtdef.h>





//#define HELLO_WORLD   1
//#define TELNETD       1
#define WEBSERVER     1
//#define WEBCLIENT     1

#if    HELLO_WORLD 
#include "hello-world.h"
#elif  TELNETD
#include "telnetd.h"
#elif  WEBSERVER   
#include "webserver.h"
#elif  WEBCLIENT
#include "resolv.h"
#include "webclient.h"
#else
#include "myapp.h"
#endif


#endif	/* __UIP_RTT_h__ */ 
