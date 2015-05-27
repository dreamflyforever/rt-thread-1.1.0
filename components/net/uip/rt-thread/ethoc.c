/*
 * Opencore 10/100 ethernet mac driver
 *
 * Copyright (C) 2007-2008 Avionic Design Development GmbH
 * Copyright (C) 2008-2009 Avionic Design GmbH
 *   Thierry Reding <thierry.reding@avionic-design.de>
 * Copyright (C) 2010 Thomas Chou <thomas@wytron.com.tw>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Modify from linux driver,first use in openrisc by Shanjin Yang
 */

//#include <malloc.h>
//#include <net.h>
//#include <miiphy.h>
//#include <asm/io.h>
//#include <asm/cache.h>
#include "uip.h"
#include "uipif.h"
#include <rtthread.h>
#include"io.h"
#include"rthw.h"
#include "spr-defs.h"
#include"support.h"

/* register offsets */
#define	MODER		0x00
#define	INT_SOURCE	0x04
#define	INT_MASK	0x08
#define	IPGT		0x0c
#define	IPGR1		0x10
#define	IPGR2		0x14
#define	PACKETLEN	0x18
#define	COLLCONF	0x1c
#define	TX_BD_NUM	0x20
#define	CTRLMODER	0x24
#define	MIIMODER	0x28
#define	MIICOMMAND	0x2c
#define	MIIADDRESS	0x30
#define	MIITX_DATA	0x34
#define	MIIRX_DATA	0x38
#define	MIISTATUS	0x3c
#define	MAC_ADDR0	0x40
#define	MAC_ADDR1	0x44
#define	ETH_HASH0	0x48
#define	ETH_HASH1	0x4c
#define	ETH_TXCTRL	0x50

/* mode register */
#define	MODER_RXEN	(1 <<  0)	/* receive enable */
#define	MODER_TXEN	(1 <<  1)	/* transmit enable */
#define	MODER_NOPRE	(1 <<  2)	/* no preamble */
#define	MODER_BRO	(1 <<  3)	/* broadcast address */
#define	MODER_IAM	(1 <<  4)	/* individual address mode */
#define	MODER_PRO	(1 <<  5)	/* promiscuous mode */
#define	MODER_IFG	(1 <<  6)	/* interframe gap for incoming frames */
#define	MODER_LOOP	(1 <<  7)	/* loopback */
#define	MODER_NBO	(1 <<  8)	/* no back-off */
#define	MODER_EDE	(1 <<  9)	/* excess defer enable */
#define	MODER_FULLD	(1 << 10)	/* full duplex */
#define	MODER_RESET	(1 << 11)	/* FIXME: reset (undocumented) */
#define	MODER_DCRC	(1 << 12)	/* delayed CRC enable */
#define	MODER_CRC	(1 << 13)	/* CRC enable */
#define	MODER_HUGE	(1 << 14)	/* huge packets enable */
#define	MODER_PAD	(1 << 15)	/* padding enabled */
#define	MODER_RSM	(1 << 16)	/* receive small packets */

/* interrupt source and mask registers */
#define	INT_MASK_TXF	(1 << 0)	/* transmit frame */
#define	INT_MASK_TXE	(1 << 1)	/* transmit error */
#define	INT_MASK_RXF	(1 << 2)	/* receive frame */
#define	INT_MASK_RXE	(1 << 3)	/* receive error */
#define	INT_MASK_BUSY	(1 << 4)
#define	INT_MASK_TXC	(1 << 5)	/* transmit control frame */
#define	INT_MASK_RXC	(1 << 6)	/* receive control frame */

#define	INT_MASK_TX	(INT_MASK_TXF | INT_MASK_TXE)
#define	INT_MASK_RX	(INT_MASK_RXF | INT_MASK_RXE)

#define	INT_MASK_ALL ( \
		INT_MASK_TXF | INT_MASK_TXE | \
		INT_MASK_RXF | INT_MASK_RXE | \
		INT_MASK_TXC | INT_MASK_RXC | \
		INT_MASK_BUSY \
	)

/* packet length register */
#define	packetlen_min(min)		(((min) & 0xffff) << 16)
#define	packetlen_max(max)		(((max) & 0xffff) <<  0)
#define	packetlen_min_max(min, max)	(packetlen_min(min) | \
					packetlen_max(max))

/* transmit buffer number register */
#define	tx_bd_num_val(x)	(((x) <= 0x80) ? (x) : 0x80)

/* control module mode register */
#define	ctrlMODER_passall	(1 << 0)	/* pass all receive frames */
#define	ctrlMODER_rxflow	(1 << 1)	/* receive control flow */
#define	ctrlMODER_txflow	(1 << 2)	/* transmit control flow */

/* mii mode register */
#define	miiMODER_clkdiv(x)	((x) & 0xfe)	/* needs to be an even number */
#define	miiMODER_nopre		(1 << 8)	/* no preamble */

/* mii command register */
#define	miicommand_scan		(1 << 0)	/* scan status */
#define	miicommand_read		(1 << 1)	/* read status */
#define	miicommand_write	(1 << 2)	/* write control data */

/* mii address register */
#define	miiaddress_fiad(x)		(((x) & 0x1f) << 0)
#define	miiaddress_rgad(x)		(((x) & 0x1f) << 8)
#define	miiaddress_addr(phy, reg)	(miiaddress_fiad(phy) | \
					miiaddress_rgad(reg))

/* mii transmit data register */
#define	miitx_data_val(x)	((x) & 0xffff)

/* mii receive data register */
#define	miirx_data_val(x)	((x) & 0xffff)

/* mii status register */
#define	miistatus_linkfail	(1 << 0)
#define	miistatus_busy		(1 << 1)
#define	miistatus_invalid	(1 << 2)

/* tx buffer descriptor */
#define	tx_bd_cs		(1 <<  0)	/* carrier sense lost */
#define	tx_bd_df		(1 <<  1)	/* defer indication */
#define	tx_bd_lc		(1 <<  2)	/* late collision */
#define	tx_bd_rl		(1 <<  3)	/* retransmission limit */
#define	tx_bd_retry_mask	(0x00f0)
#define	tx_bd_retry(x)		(((x) & 0x00f0) >>  4)
#define	tx_bd_ur		(1 <<  8)	/* transmitter underrun */
#define	tx_bd_CRC		(1 << 11)	/* tx CRC enable */
#define	tx_bd_pad		(1 << 12)	/* pad enable */
#define	tx_bd_wrap		(1 << 13)
#define	tx_bd_irq		(1 << 14)	/* interrupt request enable */
#define	tx_bd_ready		(1 << 15)	/* tx buffer ready */
#define	tx_bd_len(x)		(((x) & 0xffff) << 16)
#define	tx_bd_len_mask		(0xffff << 16)

#define	tx_bd_stats		(tx_bd_cs | tx_bd_df | tx_bd_lc | \
				tx_bd_rl | tx_bd_retry_mask | tx_bd_ur)

/* rx buffer descriptor */
#define	rx_bd_lc	(1 <<  0)	/* late collision */
#define	rx_bd_CRC	(1 <<  1)	/* rx CRC error */
#define	rx_bd_sf	(1 <<  2)	/* short frame */
#define	rx_bd_tl	(1 <<  3)	/* too long */
#define	rx_bd_dn	(1 <<  4)	/* dribble nibble */
#define	rx_bd_is	(1 <<  5)	/* invalid symbol */
#define	rx_bd_or	(1 <<  6)	/* receiver overrun */
#define	rx_bd_miss	(1 <<  7)
#define	rx_bd_cf	(1 <<  8)	/* control frame */
#define	rx_bd_wrap	(1 << 13)
#define	rx_bd_irq	(1 << 14)	/* interrupt request enable */
#define	rx_bd_empty	(1 << 15)
#define	rx_bd_len(x)	(((x) & 0xffff) << 16)

#define	rx_bd_stats	(rx_bd_lc | rx_bd_CRC | rx_bd_sf | rx_bd_tl | \
			rx_bd_dn | rx_bd_is | rx_bd_or | rx_bd_miss)

#define	ethoc_bufsiz		1536
#define	ethoc_zlen		64
#define	ethoc_bd_base		0x400
#define	ethoc_timeout		(hz / 2)
#define	ethoc_mii_timeout	(1 + (hz / 5))

/**
 * struct ethoc - driver-private device structure
 * @num_tx:	number of send buffers
 * @cur_tx:	last send buffer written
 * @dty_tx:	last buffer actually sent
 * @num_rx:	number of receive buffers
 * @cur_rx:	current receive buffcurrent_dev 
struct ethoc {
	u32 num_tx;
	u32 cur_tx;
	u32 dty_tx;
	u32 num_rx;
	u32 cur_rx;
};*/

/**
 * struct ethoc_bd - buffer descriptor
 * @stat:	buffer statistics
 * @addr:	physical memory address
 */
struct ethoc_bd {
	u32 stat;
	u32 addr;
};


#define loff_t long
//#define readl(reg)            *((volatile unsigned int *)reg)
//#define writel(data,reg)      *((volatile unsigned int *)reg) = data;
#define  pktbufsrx 4 /*is system value? 4 or 16*/
#define ETH_ZLEN        60              /* Min. octets in frame sans FCS */
//#define TX_BD_PAD               (1 << 12) /* pad enable for short packets */
/* TX buffer descriptor */
 #define TX_BD_CS                (1 <<  0) /* carrier sense lost */
 #define TX_BD_DF                (1 <<  1) /* defer indication */
 #define TX_BD_LC                (1 <<  2) /* late collision */
#define TX_BD_RL                (1 <<  3) /* retransmission limit */
 #define TX_BD_RETRY_MASK        (0x00f0)
 #define TX_BD_RETRY(x)          (((x) & 0x00f0) >>  4)
 #define TX_BD_UR                (1 <<  8) /* transmitter underrun */
 #define TX_BD_CRC               (1 << 11) /* TX CRC enable */
 #define TX_BD_PAD               (1 << 12) /* pad enable for short packets */
 #define TX_BD_WRAP              (1 << 13)
 #define TX_BD_IRQ               (1 << 14) /* interrupt request enable */
 #define TX_BD_READY             (1 << 15) /* TX buffer ready */
 #define TX_BD_LEN(x)            (((x) & 0xffff) << 16)
 #define TX_BD_LEN_MASK          (0xffff << 16)
 
 #define TX_BD_STATS             (TX_BD_CS | TX_BD_DF | TX_BD_LC | \
                                 TX_BD_RL | TX_BD_RETRY_MASK | TX_BD_UR)
 
 /* RX buffer descriptor */
 #define RX_BD_LC        (1 <<  0) /* late collision */
 #define RX_BD_CRC       (1 <<  1) /* RX CRC error */
 #define RX_BD_SF        (1 <<  2) /* short frame */
 #define RX_BD_TL        (1 <<  3) /* too long */
 #define RX_BD_DN        (1 <<  4) /* dribble nibble */
 #define RX_BD_IS        (1 <<  5) /* invalid symbol */
 #define RX_BD_OR        (1 <<  6) /* receiver overrun */
 #define RX_BD_MISS      (1 <<  7)
 #define RX_BD_CF        (1 <<  8) /* control frame */
 #define RX_BD_WRAP      (1 << 13)
 #define RX_BD_IRQ       (1 << 14) /* interrupt request enable */
 #define RX_BD_EMPTY     (1 << 15)
 #define RX_BD_LEN(x)    (((x) & 0xffff) << 16)
 
 #define RX_BD_STATS     (RX_BD_LC | RX_BD_CRC | RX_BD_SF | RX_BD_TL | \
                         RX_BD_DN | RX_BD_IS | RX_BD_OR | RX_BD_MISS)
 #define ETHOC_ZLEN              64 
#define uchar unsigned char 
# define PKTBUFSRX	pktbufsrx
#define PKTALIGN	32
#define PKTSIZE_ALIGN		1536
volatile unsigned char	PktBuf[(PKTBUFSRX+1) * PKTSIZE_ALIGN + PKTALIGN];
/* THE transmit packet */
volatile uchar *NetTxPacket;
/* Receive packet */
volatile uchar *NetRxPackets[PKTBUFSRX];
#define ulong unsigned long
struct rt_semaphore lock;
struct rt_semaphore eth_rx_sem;
static void delay_ms(rt_uint32_t ms)
{
    rt_uint32_t len;
    for (; ms > 0; ms --)
        for (len = 0; len < 100; len++ );
}

static inline u32 ethoc_read(struct eth_device *dev, loff_t offset)
{
    return readl(dev->iobase + offset);
}

static inline void ethoc_write(struct eth_device *dev, loff_t offset, u32 data)
{

    writel(data, dev->iobase + offset);
}

static inline void ethoc_read_bd(struct eth_device *dev, int index,
				 struct ethoc_bd *bd)
{
	loff_t offset = ethoc_bd_base + (index * sizeof(struct ethoc_bd));
	bd->stat = ethoc_read(dev, offset + 0);
	bd->addr = ethoc_read(dev, offset + 4);
}

static inline void ethoc_write_bd(struct eth_device *dev, int index,
				  const struct ethoc_bd *bd)
{
	loff_t offset = ethoc_bd_base + (index * sizeof(struct ethoc_bd));
	ethoc_write(dev, offset + 0, bd->stat);
	ethoc_write(dev, offset + 4, bd->addr);
}

static int ethoc_set_mac_address(struct eth_device *dev)
{
	u8 *mac = dev->enetaddr;

	ethoc_write(dev, MAC_ADDR0, (mac[2] << 24) | (mac[3] << 16) |
		    (mac[4] << 8) | (mac[5] << 0));
	ethoc_write(dev, MAC_ADDR1, (mac[0] << 8) | (mac[1] << 0));
	return 0;
}

static inline void ethoc_ack_irq(struct eth_device *dev, u32 mask)
{
	//ethoc_write(dev, INT_SOURCE, 0x7f );
    ethoc_write(dev, INT_SOURCE, mask/*1<<6*/);
}

static inline void ethoc_enable_rx_and_tx(struct eth_device *dev)
{
	u32 mode = ethoc_read(dev, MODER);
	mode |= MODER_RXEN | MODER_TXEN;
	ethoc_write(dev, MODER, mode);
}

static inline void ethoc_disable_rx_and_tx(struct eth_device *dev)
{
	u32 mode = ethoc_read(dev, MODER);
	mode &= ~(MODER_RXEN | MODER_TXEN);
	ethoc_write(dev, MODER, mode);
}
extern void flush_dcache_range(unsigned long addr, unsigned long stop);
static int ethoc_init_ring(struct eth_device *dev)
{
	struct ethoc *priv = (struct ethoc *)dev->priv;
	struct ethoc_bd bd;
	int i;

	priv->cur_tx = 0;
	priv->dty_tx = 0;
	priv->cur_rx = 0;


	/* setup transmission buffers */
	bd.stat = tx_bd_irq | tx_bd_CRC;
	for (i = 0; i < priv->num_tx; i++) {
		if (i == priv->num_tx - 1)
			bd.stat |= tx_bd_wrap;
		ethoc_write_bd(dev, i, &bd);
	}
    //copy from uboot
	NetTxPacket = RT_NULL;    
	if (!NetTxPacket) {
		int	i;
		/*
		 *	Setup packet buffers, aligned correctly.
		 */
		NetTxPacket = &PktBuf[0] + (PKTALIGN - 1);
		NetTxPacket -= (ulong)NetTxPacket % PKTALIGN;
		for (i = 0; i < PKTBUFSRX; i++)
			NetRxPackets[i] = NetTxPacket + (i+1)*PKTSIZE_ALIGN;
	}

	bd.stat = rx_bd_empty | rx_bd_irq;

	for (i = 0; i < priv->num_rx; i++) {
		bd.addr = (u32)NetRxPackets[i];
		if (i == priv->num_rx - 1)
			bd.stat |= rx_bd_wrap;

		flush_dcache_range(bd.addr, bd.addr + PKTSIZE_ALIGN);//attention
		ethoc_write_bd(dev, priv->num_tx + i, &bd);
	}

	return 0;
}

static int ethoc_reset(struct eth_device *dev)
{
	u32 mode;

	/* todo: reset controller? */

	ethoc_disable_rx_and_tx(dev);

	/* todo: setup registers */

	/* enable fcs generation and automatic padding */
	mode = ethoc_read(dev, MODER);
	mode |= MODER_CRC | MODER_PAD;
	ethoc_write(dev, MODER, mode);

	/* set full-duplex mode */
	mode = ethoc_read(dev, MODER);
	mode |= MODER_FULLD;
	ethoc_write(dev, MODER, mode);
	ethoc_write(dev, IPGT, 0x15);
    
   	mode = ethoc_read(dev, CTRLMODER);
	mode |= (1<<2) | 1;
    ethoc_write(dev, CTRLMODER, mode); // text interrupt enable
   // ethoc_write(dev, CTRLMODER, 2); // recv interrupt enable  
	//ethoc_ack_irq(dev, int_mask_all);
	ethoc_ack_irq(dev, INT_MASK_ALL);
    ethoc_enable_rx_and_tx(dev);

    return 0;
}

static struct rt_semaphore sem_ack, sem_lock;

static int ethoc_init(struct eth_device *dev)//, bd_t * bd)
{
	struct ethoc *priv = (struct ethoc *)dev->priv;
    //rt_sem_init(&sem_ack, "tx_ack", 1, RT_IPC_FLAG_FIFO);
    //rt_sem_init(&sem_lock, "eth_lock", 1, RT_IPC_FLAG_FIFO);

	priv->num_tx = 1;
	priv->num_rx = pktbufsrx;
	ethoc_write(dev, TX_BD_NUM/*tx_bd_num*/, priv->num_tx);
	ethoc_init_ring(dev);
	ethoc_reset(dev);
	return 0;
}

static int ethoc_update_rx_stats(struct ethoc_bd *bd)
{
	int ret = 0;

	if (bd->stat & rx_bd_tl) {
		debug("ethoc: " "rx: frame too long\n");
		ret++;
	}

	if (bd->stat & rx_bd_sf) {
		debug("ethoc: " "rx: frame too short\n");
		ret++;
	}

	if (bd->stat & rx_bd_dn)
		debug("ethoc: " "rx: dribble nibble\n");

	if (bd->stat & rx_bd_CRC) {
		debug("ethoc: " "rx: wrong CRC\n");
		ret++;
	}

	if (bd->stat & rx_bd_or) {
		debug("ethoc: " "rx: overrun\n");
		ret++;
	}

	if (bd->stat & rx_bd_lc) {
		debug("ethoc: " "rx: late collision\n");
		ret++;
	}
	return ret;
}
extern void deal();

struct pbuf *ethoc_rx(struct eth_device *dev, int limit)
{
	struct ethoc *priv = (struct ethoc *)dev->priv;
	int count;
    struct pbuf *p;
 	p = RT_NULL;

	for (count = 0; count < limit; ++count) {
		u32 entry;
		struct ethoc_bd bd;

		entry = priv->num_tx + (priv->cur_rx % priv->num_rx);
		ethoc_read_bd(dev, entry, &bd);
		if (bd.stat & rx_bd_empty)
			break;
#if 0
        debug("%s(): rx buffer %d, %x received\n",
		      __func__, priv->cur_rx, bd.stat);
#endif	
        if (ethoc_update_rx_stats(&bd) == 0) {
			int size = bd.stat >> 16;
			size -= 4;	/* strip the CRC */
			//netreceive((void *)bd.addr, size);

            rt_memcpy(uip_buf, (char *)bd.addr,size);
            uip_len = size;

        }

		/* clear the buffer descriptor so it can be reused */
		flush_dcache_range(bd.addr, bd.addr + PKTSIZE_ALIGN );
		bd.stat &= ~rx_bd_stats;
		bd.stat |= rx_bd_empty;
		ethoc_write_bd(dev, entry, &bd);
		priv->cur_rx++;

    }

    return p;
}

static int ethoc_update_tx_stats(struct ethoc_bd *bd)
{
	if (bd->stat & tx_bd_lc)
		debug("ethoc: " "tx: late collision\n");

	if (bd->stat & tx_bd_rl)
		debug("ethoc: " "tx: retransmit limit\n");

	if (bd->stat & tx_bd_ur)
		debug("ethoc: " "tx: underrun\n");

	if (bd->stat & tx_bd_cs)
		debug("ethoc: " "tx: carrier sense lost\n");

	return 0;
}

static void ethoc_tx(struct eth_device *dev)
{
	struct ethoc *priv = (struct ethoc *)dev->priv;
	u32 entry = priv->dty_tx % priv->num_tx;
	struct ethoc_bd bd;

	ethoc_read_bd(dev, entry, &bd);
	if ((bd.stat & tx_bd_ready) == 0)
		(void)ethoc_update_tx_stats(&bd);
}

static int ethoc_send(struct eth_device *dev, volatile void *packet, int length)
{
	struct ethoc *priv = (struct ethoc *)dev->priv;
	struct ethoc_bd bd;
	u32 entry;
	u32 pending;
	int tmo;

    //rt_sem_take(&lock, 100);

	entry = priv->cur_tx % priv->num_tx;
	ethoc_read_bd(dev, entry, &bd);
	if (length < ETHOC_ZLEN)
		bd.stat |= TX_BD_PAD;
	else
		bd.stat &= ~TX_BD_PAD;
	bd.addr = (u32)packet;

	flush_dcache_range(bd.addr, bd.addr + length);
	bd.stat &= ~(TX_BD_STATS | TX_BD_LEN_MASK);
	bd.stat |= TX_BD_LEN(length);
	ethoc_write_bd(dev, entry, &bd);

	/* start transmit */
	bd.stat |= TX_BD_READY;
	ethoc_write_bd(dev, entry, &bd);
#if 0
    int i;
    for (i = 0; i < length; i++) {
    if((i & 0xf) == 0) {
        rt_kprintf("\n%04x:", i);
    }
    if((i & 0xf) == 8) {
        rt_kprintf(" |");
    }
        rt_kprintf(" %02x",uip_buf[i]);
    }
    rt_kprintf("\npack send\n");
#endif

	/* wait for transfer to succeed */
	//tmo = get_timer(0) + 5 * CONFIG_SYS_HZ;
    tmo = rt_tick_get() + 1000*5; 
	while (1) {
		pending = ethoc_read(dev, INT_SOURCE);
		ethoc_ack_irq(dev, pending & ~INT_MASK_RX);
		if (pending & INT_MASK_BUSY)
			debug("%s(): packet dropped\n", __func__);

	//	if (pending & INT_MASK_TX) {
           		ethoc_tx(dev);
			break;
	//	}
        if (/*get_timer(0)*/rt_tick_get >= tmo) {
			debug("%s(): timed out\n", __func__);
			return -1;
		}
	}

    //rt_sem_release(&lock);

	//debug("%s(): packet sent\n", __func__);
	return 0;
}

static void ethoc_halt(struct eth_device *dev)
{
	ethoc_disable_rx_and_tx(dev);
}

//extern rt_err_t eth_device_ready(struct eth_device* dev);

struct pbuf *ethoc_recv(struct eth_device *dev)
{
	u32 pending;
    struct pbuf* p;
    p = RT_NULL;
  
    //rt_sem_take(&lock, 100);

    pending = ethoc_read(dev, INT_SOURCE);
	ethoc_ack_irq(dev, pending);
	if (pending & INT_MASK_BUSY)
		debug("%s(): packet dropped\n", __func__);
	if (pending & INT_MASK_RX) {
	//	debug("%s(): rx irq\n", __func__);

        p = ethoc_rx(dev, pktbufsrx/*PKTBUFSRX*/);
	}
   
    deal();
    //rt_sem_release(&lock);
    
    return p;
}

extern rt_err_t eth_device_init(struct eth_device* dev, const char* name);

void rt_hw_ethoc_isr(int arg);
int ethoc_initialize(u8 dev_num, int base_addr)
{
	struct ethoc *priv;
	struct eth_device *dev;

	//rt_sem_init(&lock, "eth", 1, RT_IPC_FLAG_FIFO);


	priv = malloc(sizeof(*priv));
	if (!priv)
		return 0;
	dev = malloc(sizeof(*dev));
	if (!dev) {
		free(priv);
		return 0;
	}

	memset(dev, 0, sizeof(*dev));
	dev->priv = priv;
	dev->iobase = base_addr;
	dev->halt = ethoc_halt;

	dev->write_hwaddr = ethoc_set_mac_address; 
    dev->fops.init = ethoc_init;
    dev->eth_tx    = ethoc_send;
    dev->eth_rx      = ethoc_recv;
    eth_device_init(dev, "e0");
    //rt_hw_interrupt_install(4, rt_hw_ethoc_isr,RT_NULL);
    ethoc_write(dev, INT_MASK, 0xffffffff);
    
    current_dev = dev;
extern struct uip_eth_addr uip_ethaddr;
    dev->enetaddr[0] = uip_ethaddr.addr[0];
    dev->enetaddr[1] = uip_ethaddr.addr[1]; 
    dev->enetaddr[2] = uip_ethaddr.addr[2]; 
    dev->enetaddr[3] = uip_ethaddr.addr[3]; 
    dev->enetaddr[4] = uip_ethaddr.addr[4]; 
    dev->enetaddr[5] = uip_ethaddr.addr[5]; 

    //eth_device_ready(current_dev);

    ethoc_set_mac_address(dev);	
    return 1;
}

void rt_hw_ethoc_isr(int arg)
{
    mtspr(SPR_PICSR, mfspr(SPR_PICSR) & (~(1 << 4)));
   	u32 pending;
        
    struct pbuf* p;
    p = RT_NULL;
    ethoc_disable_rx_and_tx( current_dev );
    
	pending = ethoc_read(current_dev, INT_SOURCE);
	if (pending & INT_MASK_BUSY)
		debug("%s(): packet dropped\n", __func__);
	if (pending & INT_MASK_RX) {
		//debug("%s(): rx irq\n", __func__);
        //p = ethoc_rx(current_dev, pktbufsrx/*PKTBUFSRX*/);
        //eth_device_ready(current_dev);

    }
    if(pending & INT_MASK_TX)
    {
        //rt_kprintf("tx iqr\n");
    }
    ethoc_enable_rx_and_tx( current_dev );    
}
/* Mask is mean no interrupt*/
