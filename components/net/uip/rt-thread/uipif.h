#include <rtthread.h>
//#define RT_UIP_ETHTHREAD_MBOX_SIZE

struct pbuf 
{
   rt_uint8_t *payload;
   rt_uint16_t len;  
};

struct eth_tx_msg
{
 	struct rt_semaphore tx_ack;
    rt_uint8_t  *buf;
	rt_uint16_t  len;
	
};
#if 0
struct eth_device
{
	/* inherit from rt_device */
	struct rt_device fops;
	struct eth_tx_msg tx_msg;
	/* eth device interface */
	struct pbuf* (*eth_rx)(void);
	rt_err_t (*eth_tx)(rt_uint8_t *buf,rt_uint16_t len);
	/* interface address info. */
	
};
#endif

struct ethoc {
	u32 num_tx;
	u32 cur_tx;
	u32 dty_tx;
	u32 num_rx;
	u32 cur_rx;
};

typedef struct eth_device{
    struct rt_device fops;
    unsigned int  iobase;
    struct ethoc *priv;
    unsigned char enetaddr[6];
    struct pbuf* (*eth_rx)(struct eth_device *dev);
	rt_err_t (*eth_tx)(struct eth_device *dev, volatile void *packet/*rt_uint8_t *buf*/, rt_uint16_t len);
   
    struct eth_tx_msg tx_msg;

    void (*halt)(struct eth_device *dev);
    int (*write_hwaddr)(struct eth_device *dev);
}eth_device;

eth_device *current_dev;
