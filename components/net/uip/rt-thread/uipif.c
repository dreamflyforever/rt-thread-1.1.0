/*
 * Copyright (c) 2001, Adam Dunkels.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by Adam Dunkels.
 * 4. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the uIP TCP/IP stack.
 *
 * $Id: main.c,v 1.16 2006/06/11 21:55:03 adam Exp $
 *
 */

#include <rtthread.h>
#include <rtdef.h>
#include "uip_rtt.h"
#include "uip-conf.h"
#include "uip.h"
#include "uip_arp.h"
#include "uipif.h"

#pragma pack(1)
typedef  struct arp_hdr {
    struct uip_eth_hdr ethhdr;
    u16_t hwtype;
    u16_t protocol;
    u8_t hwlen;
    u8_t protolen;
    u16_t opcode;
    struct uip_eth_addr shwaddr;
    u16_t sipaddr[2];
    struct uip_eth_addr dhwaddr;
    u16_t dipaddr[2];
}arp_hdr;
#pragma pack()

#define BUF_IF ((struct uip_eth_hdr *)uip_buf)

#define BUF   ((struct arp_hdr *)&uip_buf[0])
#define ARP_REPLY   2
#define ARP_REQUEST 1
//#define offsect(book, member)  (int)(&((book *)0->member))
#define offsect(BOOK, author) (int)(&((BOOK *)0)->author)

#ifndef NULL
#define NULL (void *)0
#endif /* NULL */

/*---------------------------------------------------------------------------*/
rt_err_t eth_device_init(struct eth_device* dev, const char* name)
{
    uip_ipaddr_t ipaddr;

    rt_device_register(&(dev->fops), name, RT_DEVICE_FLAG_RDWR);
    dev->fops.type = RT_Device_Class_NetIf;

    /*
       uip_ipaddr(ipaddr, RT_UIP_IPADDR0,RT_UIP_IPADDR1,RT_UIP_IPADDR2,RT_UIP_IPADDR3);
       uip_sethostaddr(ipaddr);
       uip_ipaddr(ipaddr, RT_UIP_GWADDR0,RT_UIP_GWADDR1,RT_UIP_GWADDR2,RT_UIP_GWADDR3);
       uip_setdraddr(ipaddr);
       uip_ipaddr(ipaddr, RT_UIP_MSKADDR0,RT_UIP_MSKADDR1,RT_UIP_MSKADDR2,RT_UIP_MSKADDR3);
       uip_setnetmask(ipaddr);
     */
    uip_ipaddr(ipaddr, 192,168,0,2);
    uip_sethostaddr(ipaddr);
    uip_ipaddr(ipaddr, 192,168,0,1);
    uip_setdraddr(ipaddr);
    uip_ipaddr(ipaddr, 255,255,255,0);
    uip_setnetmask(ipaddr);

    return RT_EOK;
}


/* eth rx/tx thread */
static struct rt_thread eth_thread;
#ifndef RT_UIP_ETHTHREAD_PRIORITY
#define RT_ETHERNETIF_THREAD_PREORITY	30//0x90
static char eth_thread_stack[1024];
#endif

void eth_thread_entry(void* parameter);
rt_err_t eth_system_device_init(void)
{
    unsigned int res;

    res = rt_thread_init(&eth_thread, "ethernet", eth_thread_entry, RT_NULL,
            &eth_thread_stack[0], sizeof(eth_thread_stack),
            RT_ETHERNETIF_THREAD_PREORITY, 16);
    RT_ASSERT(res == RT_EOK);

    res = rt_thread_startup(&eth_thread);
    RT_ASSERT(res == RT_EOK);
}

void uip_sys_init(void)
{    
    struct rt_device *eth_dev;
    uip_ipaddr_t ipaddr;

    uip_init();   

    httpd_init();  
    /*#if   HELLO_WORLD
      hello_world_init();
#elif TELNETD
telnetd_init();
#elif WEBSERVER
httpd_init();
printf("httpd_init\n\n");
#elif WEBCLIENT
webclient_init();
resolv_init();
uip_ipaddr(ipaddr, 202,96,128,166);  //set DNS server 
resolv_conf(ipaddr);
resolv_query("www.rt-thread.org");
#else
uip_listen(HTONS(1234));
uip_ipaddr(ipaddr, 192,168,2,244);
uip_connect(&ipaddr, HTONS(5678)); 
#endif
     */
    eth_dev = rt_device_find("e0");
    RT_ASSERT(eth_dev != RT_NULL);

    return;
}

#define CLOCK_SECOND 32
struct timer{
    unsigned int start;
    unsigned int interval;
};

struct timer periodic_timer, arp_timer;


void eth_thread_entry(void* parameter)
    {
#if 0
    struct pbuf *tmp;

    timer_set(&periodic_timer, CLOCK_SECOND / 2);
    timer_set(&arp_timer, CLOCK_SECOND * 10);
    while(1) {
        tmp = current_dev->eth_rx(current_dev);
        if(tmp == NULL)
        {
            continue;
        }

        rt_free(tmp);
        rt_free(tmp->payload);
    }
#endif
 
    timer_set(&periodic_timer, CLOCK_SECOND / 2);
    timer_set(&arp_timer, CLOCK_SECOND * 10);

    while(1)
    {
        current_dev->eth_rx(current_dev);   
        //rt_thread_sleep(10);
    }

}
void deal(){
    int i;
    if(uip_len > 0) {
        if(BUF_IF->type == htons(UIP_ETHTYPE_IP)) {
            uip_arp_ipin();
            uip_input();
            /* If the above function invocation resulted in data that
               should be sent out on the network, the global variable
               uip_len is set to a value > 0. */
            if(uip_len > 0) {
                uip_arp_out();
                current_dev->eth_tx(current_dev, uip_buf, uip_len);
            }
        } else if(BUF_IF->type == htons(UIP_ETHTYPE_ARP)) {
            uip_arp_arpin();
            /* If the above function invocation resulted in data that
               should be sent out on the network, the global variable
               uip_len is set to a value > 0. */
            if(uip_len > 0) {
                current_dev->eth_tx(current_dev, uip_buf, uip_len);
            }
        }

    } else if(timer_expired(&periodic_timer)) {
        timer_reset(&periodic_timer);
        for(i = 0; i < UIP_CONNS; i++) {
            uip_periodic(i);
            /* If the above function invocation resulted in data that
               should be sent out on the network, the global variable
               uip_len is set to a value > 0. */
            if(uip_len > 0) {
                uip_arp_out();
                current_dev->eth_tx(current_dev, uip_buf, uip_len);	
            }
        }

#if UIP_UDP
        for(i = 0; i < UIP_UDP_CONNS; i++) {
            uip_udp_periodic(i);
            /* If the above function invocation resulted in data that
               should be sent out on the network, the global variable
               uip_len is set to a value > 0. */
            if(uip_len > 0) {
                uip_arp_out();
                current_dev->eth_tx(current_dev, uip_buf, uip_len);	
            }
        }
#endif /* UIP_UDP */

        /* Call the ARP timer function every 10 seconds. */
        if(timer_expired(&arp_timer)) {
            timer_reset(&arp_timer);
            uip_arp_timer();
        }
    }
    //rt_exit_critical();

    return 0;
}
/*---------------------------------------------------------------------------*/
    void
uip_log(char *m)
{
    printf("uIP log message: %s\n", m);
}


    void
resolv_found(char *name, u16_t *ipaddr)
{
    u16_t *ipaddr2;

    if(ipaddr == NULL) {
        printf("Host '%s' not found.\n", name);
    } else {
        printf("Found name '%s' = %d.%d.%d.%d\n", name,
                htons(ipaddr[0]) >> 8,
                htons(ipaddr[0]) & 0xff,
                htons(ipaddr[1]) >> 8,
                htons(ipaddr[1]) & 0xff);
        /*    webclient_get("www.sics.se", 80, "/~adam/uip");*/
    }
}
/*---------------------------------------------------------------------------*/

