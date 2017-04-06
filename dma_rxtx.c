#include <stdio.h>
#include <stdlib.h>
#include "xdma-core.h"

#include<string.h>
#include<fcntl.h>
#include<sys/mman.h>
#include<stdint.h>

#include"virtio.h"

#define PAGE_SIZE 4096
#define DESC_CNT 1
#define LENGTH 4*1024
#define NUM_BUFFERS 64
#define LOGGING 0
#define TIMEOUT 3
extern volatile connected_to_guest;
int channel;
volatile struct mydata *data;
volatile unsigned int * bar_base_user;
volatile unsigned int  *rd_data[64], *wr_data[64];
struct mydata
{
        //struct device dev;
        void *tx_queue;
        uint64_t tx_queue_dma_addr;
        void *rx_queue;
        uint64_t rx_queue_dma_addr;
        void *rx_result;
	uint64_t rx_result_dma_addr;
	void *bar_virt;
	uint64_t bar_base_addr_phy;
        void *coherent_mem_tx[NUM_BUFFERS];
        uint64_t coherent_mem_tx_dma_addr[NUM_BUFFERS];
        void *coherent_mem_rx[NUM_BUFFERS];
        uint64_t coherent_mem_rx_dma_addr[NUM_BUFFERS];
} __attribute__((packed)) ;

int simple_read(/*struct packet *pkt*/);
int simple_write(/*struct packet *pkt*/);
volatile struct xdma_desc *tx_desc_virt;
volatile struct xdma_desc *rx_desc_virt;
volatile struct xdma_result *rx_result_virt;
extern unsigned char **tx_packet_buff;
extern unsigned char **rx_packet_buff;
extern uint64_t *coherent_tx_hw_addresses;
extern uint64_t *coherent_rx_hw_addresses;

/*
 * xdma_engine_stop() - stop an SG DMA engine
 */

void xdma_engine_stop(volatile struct xdma_engine *engine)
{
	unsigned int w;

	w = 0;
	w |= (unsigned int)XDMA_CTRL_IE_DESC_ALIGN_MISMATCH;
	w |= (unsigned int)XDMA_CTRL_IE_MAGIC_STOPPED;
	w |= (unsigned int)XDMA_CTRL_IE_READ_ERROR;
	w |= (unsigned int)XDMA_CTRL_IE_DESC_ERROR;
	w |= (unsigned int) XDMA_CTRL_POLL_MODE_WB;

	engine->regs->control = w;
}

static inline __attribute__((always_inline)) uint64_t current_clock_cycles()

{

	unsigned int eax_reg_rdtsc, edx_reg_rdtsc;

	unsigned long long int uptime;

	asm volatile  ("rdtsc\n\t" 
			"mov %%eax, %0\n\t"
			"mov %%edx, %1\n\t"

			:       "=r" (eax_reg_rdtsc) , "=r" (edx_reg_rdtsc)

			:

			: "eax" , "edx"

			);



	uptime =  ((unsigned long long int)edx_reg_rdtsc << 32) | eax_reg_rdtsc;

	return uptime;

}



static inline __attribute__((always_inline)) void delay_clock_cycles(uint64_t clock_cycles)

{

	register uint64_t start, end;



	start = current_clock_cycles();

	do {

		end = current_clock_cycles();

	} while ((end - start) < clock_cycles);



}



void *dma_rx(int *pkt_len, uint64_t pktpaddr)
{
	int extra_adj = DESC_CNT - 1, next_adj, j = 0, offset, sgdma_offset;
	int length, i;
	//volatile unsigned int read_p, rx_desc_start;
	volatile uint64_t read_p, rx_desc_start;
	unsigned int control, w;
	struct xdma_engine *engine;
	int dir_from_dev = 0;
	volatile struct xdma_result *result;

	rx_desc_start = data->rx_queue_dma_addr;
	//printf("trace :  func : %s line : %u rx_desc_start : %x\n",__func__,__LINE__,rx_desc_start);

	//read_p = data->coherent_mem_rx_dma_addr[0];
	read_p = pktpaddr;
	//length = LENGTH; /*max pkt size recv is 4096*/
	length = *pkt_len; /*max pkt size recv is 4096*/

	engine = malloc(sizeof(struct xdma_engine));
	if (!engine) {
		printf("engine allocation failed");
		return NULL;
	}

	//printf("trace :  func : %s line : %u\n",__func__,__LINE__);
	offset = H2C_CHANNEL_OFFSET + (channel * CHANNEL_SPACING);
	sgdma_offset = offset + SGDMA_OFFSET_FROM_CHANNEL;
	//printf("offset : %x sgdma_offset : %x\n",offset ,sgdma_offset);

	//printf("trace :  func : %s line : %u\n",__func__,__LINE__);
	engine->regs = (struct engine_regs *)((uintptr_t)bar_base_user + offset);
	engine->sgdma_regs = (struct engine_sgdma_regs *)((uintptr_t)bar_base_user + sgdma_offset);
	engine->rx_result_buffer_bus = data->rx_result_dma_addr;
	result = rx_result_virt;
	//printf("trace :  func : %s line : %u\n",__func__,__LINE__);
	while (j < DESC_CNT) {
		rx_desc_virt[j].dst_addr_lo = (PCI_DMA_L(read_p + (j * length)));
		rx_desc_virt[j].dst_addr_hi = (PCI_DMA_H(read_p + j * length));
		rx_desc_virt[j].src_addr_lo = (PCI_DMA_L(engine->rx_result_buffer_bus + j * sizeof(struct xdma_result)));
		rx_desc_virt[j].src_addr_hi = (PCI_DMA_H(engine->rx_result_buffer_bus + j * sizeof(struct xdma_result)));
		//rx_desc_virt[j].src_addr_lo = 0;
		//rx_desc_virt[j].src_addr_hi = 0;
		rx_desc_virt[j].next_lo = 0;// end of desc chain
		rx_desc_virt[j].next_hi = 0;
		/*
		   if (j == DESC_CNT - 1) {
		   rx_desc_virt[j].next_lo = (PCI_DMA_L(rx_desc_start));
		   rx_desc_virt[j].next_hi = (PCI_DMA_H(rx_desc_start));
		   } else {
		   rx_desc_virt[j].next_lo = (PCI_DMA_L(rx_desc_start + sizeof(struct xdma_desc) * (j + 1)));
		   rx_desc_virt[j].next_hi = (PCI_DMA_H(rx_desc_start + sizeof(struct xdma_desc) * (j + 1)));
		   }*/
		rx_desc_virt[j].bytes = (length);
		control = (DESC_MAGIC);
		if (j == DESC_CNT - 1)   /* if last packet */
			control |= XDMA_DESC_STOPPED_1; /* set to 1 to stop fetching descriptors */
		else
			control |= XDMA_DESC_STOPPED_0;

		control |= XDMA_DESC_EOP;
		control |= XDMA_DESC_COMPLETED;

		next_adj = extra_adj - j - 1;
		if (next_adj < 1)
			next_adj = 0;
		rx_desc_virt[j].control = (control) | ((next_adj << 8) & 0x00003f00);
		j++;
	}
	//printf("trace :  func : %s line : %u\n",__func__,__LINE__);
	result->length = 0;
	engine->regs->completed_desc_count = 0;
	w = (PCI_DMA_L(rx_desc_start));
	//printf("result->len : %d\n", result->length);
	/*printf("After accessing 1\n");
	  printf("engine->sgdma_regs->first_desc_lo  : %x\n",engine->sgdma_regs->first_desc_lo);
	  printf("After accessing 2\n");*/
	engine->sgdma_regs->first_desc_lo = PCI_DMA_L(rx_desc_start);
	//printf("trace :  func : %s line : %u\n",__func__,__LINE__);
	w = (PCI_DMA_H(rx_desc_start));
	engine->sgdma_regs->first_desc_hi = PCI_DMA_H(rx_desc_start);
	engine->sgdma_regs->first_desc_adjacent = extra_adj;
	//printf("trace :  func : %s line : %u\n",__func__,__LINE__);
	w = (unsigned int)XDMA_CTRL_RUN_STOP;
	w |= (unsigned int)XDMA_CTRL_IE_READ_ERROR;
	w |= (unsigned int)XDMA_CTRL_IE_DESC_ERROR;
	w |= (unsigned int)XDMA_CTRL_IE_DESC_ALIGN_MISMATCH;
	w |= (unsigned int)XDMA_CTRL_IE_MAGIC_STOPPED;
	w |= (unsigned int)XDMA_CTRL_POLL_MODE_WB;

	//wmb();
	mb();

	engine->regs->control = w;
	//printf("trace :  func : %s line : %u\n",__func__,__LINE__);
	//printf("waiting for rx dma to complete on channel %d\n",channel);
	while (engine->regs->completed_desc_count < (DESC_CNT)) {
	if(!connected_to_guest) {
			printf("returning from func : %s because of guest disconnection\n",__func__);
			*pkt_len = 0;
			xdma_engine_stop(engine);
			//wmb();
			mb();
			return NULL;
		}
		delay_clock_cycles(3400);
		//usleep(1);
	}
	//printf("read() : completed_desc_count = %d\n", engine->regs->completed_desc_count);
	//for(i=0; i < LENGTH / 4; i++)
	//      printf("%x", rd_data[i]);
	//*pkt_len = 0; 
	//printf("b4 packet received len : %d\n",*pkt_len);
	*pkt_len = result->length;
	//printf("aftr packet received len : %d\n",*pkt_len);
	xdma_engine_stop(engine);
	//wmb();
	mb();
	free(engine);
	//return (rd_data[0]);
	return pktpaddr;
}



#define RX_TIMEOUT 0

#if 1
/* pkts -> phy addr of pkts
   pkts_lengths_max -> len of each received packet
   max_num_packets -> no. of packets received
*/

#define OLD 0
static struct xdma_engine rx_engine __attribute__((aligned(4096))) ;
int dma_rx_burst(uint64_t *pkts, unsigned int *max_pkt_len, unsigned int *recv_len, int pkt_offset, int max_num_pkts)
{
	int extra_adj = max_num_pkts - 1, next_adj, j = 0, offset, sgdma_offset;
	int length, i;
	int delayed;
	//volatile unsigned int read_p, rx_desc_start;
	volatile uint64_t read_p, rx_desc_start;
	unsigned int control, w, timeout;
	struct xdma_engine *engine;
	int dir_from_dev = 0;
	volatile struct xdma_result *result;
	time_t start, stop;
	int new_rx_pkts;
	uint32_t comp_desc_cnt;
	uint64_t sclock,eclock,mclock,circles;

	//printf("max_num_pkts : %d\n",max_num_pkts);
	rx_desc_start = data->rx_queue_dma_addr;

	/*read_p = data->coherent_mem_rx_dma_addr[0];*/
	#if 0

	engine = malloc(sizeof(struct xdma_engine));
	if (!engine) {
		printf("engine allocation failed");
		return -1;
	}
	#endif
	engine = &rx_engine;

	//printf("trace :  func : %s line : %u, max_pkt_len = %d, pkt_offset = %d, max_num_pkts = %d\n",__func__,__LINE__,max_pkt_len[0],pkt_offset,max_num_pkts);
	offset = H2C_CHANNEL_OFFSET + (channel * CHANNEL_SPACING);
	sgdma_offset = offset + SGDMA_OFFSET_FROM_CHANNEL;
	
	engine->regs = (struct engine_regs *)((uintptr_t)bar_base_user + offset);
	engine->sgdma_regs = (struct engine_sgdma_regs *)((uintptr_t)bar_base_user + sgdma_offset);
	engine->rx_result_buffer_bus = data->rx_result_dma_addr;
	result = rx_result_virt;
	//printf("trace :  func : %s line : %u\n",__func__,__LINE__);
	//max_num_pkts = 1;
	for (j = 0; j < max_num_pkts; j++) {
		rx_desc_virt[j].dst_addr_lo = (PCI_DMA_L(pkts[j] + pkt_offset)); /* phy addr of pkts */
		rx_desc_virt[j].dst_addr_hi = (PCI_DMA_H(pkts[j] + pkt_offset));
		rx_desc_virt[j].src_addr_lo = (PCI_DMA_L(engine->rx_result_buffer_bus + j * sizeof(struct xdma_result)));
		rx_desc_virt[j].src_addr_hi = (PCI_DMA_H(engine->rx_result_buffer_bus + j * sizeof(struct xdma_result)));

		if (j == max_num_pkts - 1) {
			rx_desc_virt[j].next_lo = 0; // end of desc chain
			rx_desc_virt[j].next_hi = 0;
			//rx_desc_virt[j].next_lo = (PCI_DMA_L(rx_desc_start));
			//rx_desc_virt[j].next_hi = (PCI_DMA_H(rx_desc_start));
		} else {
			rx_desc_virt[j].next_lo = (PCI_DMA_L(rx_desc_start + sizeof(struct xdma_desc) * (j + 1)));
			rx_desc_virt[j].next_hi = (PCI_DMA_H(rx_desc_start + sizeof(struct xdma_desc) * (j + 1)));
		}

		rx_desc_virt[j].bytes = max_pkt_len[j]; /*max pkt size recv is 4096*/
		//rx_desc_virt[j].bytes = 4096; /*max pkt size recv is 4096*/

		control = DESC_MAGIC | XDMA_DESC_EOP | XDMA_DESC_COMPLETED;
		//if (j == DESC_CNT - 1)   /* if last packet */
		if (j == (max_num_pkts - 1))   /* if last packet */
			control |= XDMA_DESC_STOPPED_1; /* set to 1 to stop fetching descriptors */
		else
			control |= XDMA_DESC_STOPPED_0;
		next_adj = extra_adj - j - 1;
		if (next_adj < 1)
			next_adj = 0;
		rx_desc_virt[j].control = (control) | ((next_adj << 8) & 0x00003f00);
		rx_desc_virt[j].control = (control);

	#if 0
	printf("trace :  func : %s line : %u \n",__func__,__LINE__);
	printf("rx_desc_virt[j].dst_addr_lo = %x\n", rx_desc_virt[j].dst_addr_lo);
	printf("rx_desc_virt[j].dst_addr_hi = %x\n", rx_desc_virt[j].dst_addr_hi);
	printf("rx_desc_virt[j].src_addr_lo = %x\n", rx_desc_virt[j].src_addr_lo);
	printf("rx_desc_virt[j].src_addr_hi = %x\n", rx_desc_virt[j].src_addr_hi);
	printf("rx_desc_virt[j].next_lo = %x\n", rx_desc_virt[j].next_lo);
	printf("rx_desc_virt[j].next_hi = %x\n", rx_desc_virt[j].next_hi);
	printf("rx_desc_virt[j].control = %x\n", rx_desc_virt[j].control);
	printf("rx_desc_virt[j].bytes = %x\n", rx_desc_virt[j].bytes);
	#endif

	}

	//engine->regs->completed_desc_count = 0;
	w = PCI_DMA_L(rx_desc_start);
	engine->sgdma_regs->first_desc_lo = PCI_DMA_L(rx_desc_start);
	w = (PCI_DMA_H(rx_desc_start));
	engine->sgdma_regs->first_desc_hi = PCI_DMA_H(rx_desc_start);
//	engine->sgdma_regs->first_desc_adjacent = extra_adj;
	engine->sgdma_regs->first_desc_adjacent = max_num_pkts -1; // num of pkts -1
	//printf("trace :  func : %s line : %u\n",__func__,__LINE__);
	w = (unsigned int)XDMA_CTRL_RUN_STOP;
	w |= (unsigned int)XDMA_CTRL_IE_READ_ERROR;
	w |= (unsigned int)XDMA_CTRL_IE_DESC_ERROR;
	w |= (unsigned int)XDMA_CTRL_IE_DESC_ALIGN_MISMATCH;
	w |= (unsigned int)XDMA_CTRL_IE_MAGIC_STOPPED;
	w |= (unsigned int)XDMA_CTRL_POLL_MODE_WB;
	//wmb();
	mb();
	engine->regs->control = w;
	mb();
	//printf("trace :  func : %s line : %u\n",__func__,__LINE__);
	//printf("waiting for rx dma to complete on channel %d\n",channel);
	//printf("control = %x, first_desc_hi = %x, first_desc_lo = %x, first_desc_adjacent = %x\n", engine->regs->control, engine->sgdma_regs->first_desc_hi, engine->sgdma_regs->first_desc_lo, engine->sgdma_regs->first_desc_adjacent);
	//time(&start);
circles = 0;
sclock = current_clock_cycles();
	new_rx_pkts = 0;
	comp_desc_cnt = engine->regs->completed_desc_count;
	while (comp_desc_cnt < 1) {
		//time(&stop);
		if(!connected_to_guest) {
			printf("returning from func : %s because of guest disconnection\n",__func__);
			xdma_engine_stop(engine);
			//wmb();
			mb();
			return 0;
		}
		/*
		if (difftime(stop, start) > TIMEOUT) {
                        printf("Polling timeout occurred for rx_burst\n");
                        break;
                }
		*/
		//usleep(1);
		delay_clock_cycles(3400);
		circles++;
		#if RX_TIMEOUT /*timeout for RX*/
		if(circles >= 1000000) {
			printf("no packets for long time\n");
			break;
		}
		#endif
		comp_desc_cnt = engine->regs->completed_desc_count;
	}
mclock = current_clock_cycles();
#if OLD
	new_rx_pkts = engine->regs->completed_desc_count;
	while(new_rx_pkts < max_num_pkts) {
		delay_clock_cycles(3400);
		comp_desc_cnt = engine->regs->completed_desc_count;
		if (comp_desc_cnt == new_rx_pkts) {
			// no more new packets
			//printf("no more new pkts, breaking\n");
			break;
		}
		else {
			/*new_rx_pkts = engine->regs->completed_desc_count;*/
			new_rx_pkts = comp_desc_cnt;
		}
	}
	//rmb();
	mb();
	xdma_engine_stop(engine);
	//wmb();
	mb();
	//comp_desc_cnt = engine->regs->completed_desc_count;
#endif
	/*
	if(engine->regs->completed_desc_count > 1) {
	printf("read() : completed_desc_count = %d\n", engine->regs->completed_desc_count);
	}
	*/

#if OLD
	for(i=0; i < comp_desc_cnt; i++) {
		recv_len[i] = result->length;
		result++;
	}
#else

//sclock = current_clock_cycles();

	//rmb();
	mb();
	for(i=0;i<10;i++) {
		if((result->status & C2H_WB) == C2H_WB){
			break;
		}
		delay_clock_cycles(3400);
	}

	i = 0;delayed = 0;
	while(1) {
		while ((result->status & C2H_WB) == C2H_WB) {
			delayed = 0;
			recv_len[i] = result->length;
			result->status = 0;
			i++;
			result++; /*may need ring mgmt -TODO*/
			if(i == max_num_pkts) { break; }
		}

		if(!delayed) {
			delayed= 1;
			delay_clock_cycles(20000);
		}
		else {
			break;
		}

	}

	xdma_engine_stop(engine);
	//wmb();
	mb();
// If we get any new packets in between our previous check and stopping engine
	for(;i<max_num_pkts;i++) {
		if((result->status & C2H_WB) == C2H_WB) {
			recv_len[i] = result->length;
			result->status = 0;
			result++;
		}
		else {
			break;
		}
	}
eclock = current_clock_cycles();
//printf("%d pkts took  p1  : %llu p2 : %llu  cpp : %d circles : %d\n",i,mclock-sclock,eclock-mclock,i == 0 ? 0 : (eclock-mclock)/i,circles);
	//printf ("rxps: %d\n", i);
	if(i == 0 ){
		printf("zero packets came when engine->regs->completed_desc_count : %u\n",engine->regs->completed_desc_count);
	}
	return i;
#endif
	//printf("%x", rd_data[i]);
	//*pkt_len = 0;	
	//printf("b4 packet received len : %d\n",*pkt_len);
	//printf("aftr packet received len : %d\n",*pkt_len);
	//xdma_engine_stop(engine);
	//free(engine);
	/*return engine->regs->completed_desc_count;*/
	//printf ("rxps: %d\n", comp_desc_cnt);
eclock = current_clock_cycles();
//printf("%d pkts took  p0  : %llu p2 : %llu  cpp : %d circles : %d\n",comp_desc_cnt,mclock-sclock,eclock-mclock,(eclock-mclock)/comp_desc_cnt,circles);
	return comp_desc_cnt;
}
#endif

static struct xdma_engine tx_engine __attribute__((aligned(4096))) ;

int dma_tx(char * pkt, int pkt_len, int pkt_offset)
{
	//printf("func : %s\n",__func__);
    int extra_adj = DESC_CNT - 1, next_adj, j = 0, offset, sgdma_offset;
        int length;
    unsigned int control, control_field, w;
    uint64_t write_p, tx_desc_start;
    //unsigned int write_p, tx_desc_start;
    struct xdma_engine *engine;
    int dir_from_dev = 0;

    //printf("pkt : %p pkt_len: %d pkt_offset : %d\n",pkt,pkt_len,pkt_offset);
//  pkt_offset = 0; //overridden

    //printf("trace :  func : %s line : %u\n",__func__,__LINE__);

        tx_desc_start = data->tx_queue_dma_addr;

    //write_p = (uintptr_t)data->coherent_mem_tx_dma_addr[0] + pkt_offset;
    write_p = pkt + pkt_offset;
    //printf("trace :  func : %s line : %u\n",__func__,__LINE__);

    engine = malloc(sizeof(struct xdma_engine));
    if (!engine) {
        printf("engine allocation failed");
        return -1;
    }
//  printf("trace :  func : %s line : %u\n",__func__,__LINE__);

    offset = (channel * CHANNEL_SPACING);
    sgdma_offset = offset + SGDMA_OFFSET_FROM_CHANNEL;
    engine->regs = (struct engine_regs *)((uintptr_t)bar_base_user + offset);
    engine->sgdma_regs = (struct engine_sgdma_regs *)((uintptr_t)bar_base_user + sgdma_offset);
//  printf("trace :  func : %s line : %u\n",__func__,__LINE__);
    while (j < DESC_CNT) {
        tx_desc_virt[j].src_addr_lo = (PCI_DMA_L(write_p + j * length));
        tx_desc_virt[j].src_addr_hi = (PCI_DMA_H(write_p + j * length));
        tx_desc_virt[j].dst_addr_lo = (0);
        tx_desc_virt[j].dst_addr_hi = (0);
        if (j == DESC_CNT - 1) {
            tx_desc_virt[j].next_lo = (0);
            tx_desc_virt[j].next_hi = (0);
        } else {
            tx_desc_virt[j].next_lo = (PCI_DMA_L(tx_desc_start + sizeof(struct xdma_desc) * (j + 1)));
            tx_desc_virt[j].next_hi = (PCI_DMA_H(tx_desc_start + sizeof(struct xdma_desc) * (j + 1)));
        }
        tx_desc_virt[j].bytes = pkt_len;
        control = 0;
        control |= XDMA_DESC_STOPPED_0;
        control |= XDMA_DESC_EOP;
        control |= XDMA_DESC_COMPLETED;
        tx_desc_virt[j].control = control | DESC_MAGIC;

        j++;
    }
//  printf("trace :  func : %s line : %u\n",__func__,__LINE__);

    w = (PCI_DMA_L((uint64_t)tx_desc_start));
    engine->sgdma_regs->first_desc_lo = w;
//  printf("trace :  func : %s line : %u\n",__func__,__LINE__);
    w = (PCI_DMA_H(tx_desc_start));
    engine->sgdma_regs->first_desc_hi = w;
    engine->sgdma_regs->first_desc_adjacent = extra_adj;
    w = (unsigned int)XDMA_CTRL_RUN_STOP;
    w |= (unsigned int)XDMA_CTRL_IE_READ_ERROR;
    w |= (unsigned int)XDMA_CTRL_IE_DESC_ERROR;
    w |= (unsigned int)XDMA_CTRL_IE_DESC_ALIGN_MISMATCH;
    w |= (unsigned int)XDMA_CTRL_IE_MAGIC_STOPPED;
    w |= (unsigned int)XDMA_CTRL_POLL_MODE_WB;
	//wmb();
	mb();
    engine->regs->control = w;
    //printf("waiting for tx dma to complete on channel %d\n",channel);
    while (engine->regs->completed_desc_count < (DESC_CNT)) {

        if(!connected_to_guest) {
	    xdma_engine_stop(engine);
	    //wmb();
	    mb();
            printf("returning from func : %s because of guest disconnection\n",__func__);
	    free(engine);
            return 0;
        }
        delay_clock_cycles(3400);
        //usleep(1);

    }
    //printf("simple_write() : engine->regs->completed_desc_count = %d\n", engine->regs->completed_desc_count);
    xdma_engine_stop(engine);
	//wmb();
	mb();
    free(engine);
    return 0;
}


int dma_tx_burst(uint64_t pkts[64], unsigned int *pkt_len, int pkt_offset, int num_pkts)
{
	uint64_t delayer, total_bytes = 0;
	int extra_adj = num_pkts - 1, next_adj, i,j = 0, offset, sgdma_offset;
        int length;
	unsigned int control = 0, w = 0;
	//unsigned int write_p, tx_desc_start;
	uint64_t write_p, tx_desc_start;
	struct xdma_engine *engine;
	int dir_from_dev = 0,tx_to = 0;;
	uint64_t sclock,eclock;

        tx_desc_start = data->tx_queue_dma_addr;

	#if 0	
	engine = malloc(sizeof(struct xdma_engine));
	if (!engine) {
		printf("engine allocation failed");
		return -1;
	}
	#endif

	engine  = &tx_engine;

	//printf("trace :  func : %s line : %u\n",__func__,__LINE__);
	//printf("pkt_len = %d, pkt_offset = %d, num_pkts = %d\n", pkt_len[0], pkt_offset, num_pkts);
	offset = (channel * CHANNEL_SPACING);
	sgdma_offset = offset + SGDMA_OFFSET_FROM_CHANNEL;
	
	engine->regs = (struct engine_regs *)((uintptr_t)bar_base_user + offset);
	engine->sgdma_regs = (struct engine_sgdma_regs *)((uintptr_t)bar_base_user + sgdma_offset);
	for (j = 0; j < num_pkts; j++) {
		tx_desc_virt[j].src_addr_lo = (PCI_DMA_L(pkts[j] + pkt_offset));
		tx_desc_virt[j].src_addr_hi = (PCI_DMA_H(pkts[j] + pkt_offset));
		tx_desc_virt[j].dst_addr_lo = (0);
		tx_desc_virt[j].dst_addr_hi = (0);
		if (j == num_pkts - 1) {
			tx_desc_virt[j].next_lo = (0);
			tx_desc_virt[j].next_hi = (0);
			control = XDMA_DESC_STOPPED_1;
		} else {
			tx_desc_virt[j].next_lo = (PCI_DMA_L(tx_desc_start + sizeof(struct xdma_desc) * (j + 1)));
			tx_desc_virt[j].next_hi = (PCI_DMA_H(tx_desc_start + sizeof(struct xdma_desc) * (j + 1)));
			control = XDMA_DESC_STOPPED_0;
		}

		tx_desc_virt[j].bytes = pkt_len[j];
		total_bytes += pkt_len[j];
		control |= XDMA_DESC_EOP | XDMA_DESC_COMPLETED;
		tx_desc_virt[j].control = control | DESC_MAGIC;
		/*printf("tx_desc_virt[%d].dst_addr_lo = %x\n", j, tx_desc_virt[j].dst_addr_lo);
		printf("tx_desc_virt[j].dst_addr_hi = %x\n", tx_desc_virt[j].dst_addr_hi);
		printf("tx_desc_virt[j].src_addr_lo = %x\n", tx_desc_virt[j].src_addr_lo);
		printf("tx_desc_virt[j].src_addr_hi = %x\n", tx_desc_virt[j].src_addr_hi);
		printf("tx_desc_virt[j].control = %x\n", tx_desc_virt[j].control);
		printf("tx_desc_virt[j].bytes = %x\n", tx_desc_virt[j].bytes);
*/
	}
	
	w = (PCI_DMA_L((uint64_t)tx_desc_start));
	engine->sgdma_regs->first_desc_lo = w;
//	printf("trace :  func : %s line : %u\n",__func__,__LINE__);
	w = (PCI_DMA_H(tx_desc_start));
	engine->sgdma_regs->first_desc_hi = w;
	engine->sgdma_regs->first_desc_adjacent = extra_adj;
	w = (unsigned int)XDMA_CTRL_RUN_STOP;
	w |= (unsigned int)XDMA_CTRL_IE_READ_ERROR;
	w |= (unsigned int)XDMA_CTRL_IE_DESC_ERROR;
	w |= (unsigned int)XDMA_CTRL_IE_DESC_ALIGN_MISMATCH;
	w |= (unsigned int)XDMA_CTRL_IE_MAGIC_STOPPED;
	w |= (unsigned int)XDMA_CTRL_POLL_MODE_WB;
	//wmb();
	mb();
	engine->regs->control = w;
	//printf("control = %x, first_desc_hi = %x, first_desc_lo = %x, first_desc_adjacent = %x\n", engine->regs->control, engine->sgdma_regs->first_desc_hi, engine->sgdma_regs->first_desc_lo, engine->sgdma_regs->first_desc_adjacent);
	//printf("waiting for tx dma to complete on channel %d\n",channel);
	delayer = (total_bytes/5)*4;
	delay_clock_cycles(delayer);
	delayer = 3400;
	sclock = current_clock_cycles();
	while (engine->regs->completed_desc_count < num_pkts) {

		if(!connected_to_guest) {
	    		xdma_engine_stop(engine);
			//wmb();
			mb();
			printf("returning from func : %s because of guest disconnection\n",__func__);
			return 0;
		}
		eclock = current_clock_cycles();
		//delay_clock_cycles(delayer);

		if(eclock - sclock > 4000000) {
			printf("tx timeout\n");
			tx_to = 1;
			break;
		}
		//usleep(1);
	}
	//if(engine->regs->completed_desc_count > 1) {
	//printf("simple_write() : engine->regs->completed_desc_count = %d\n", engine->regs->completed_desc_count);
	//}
	xdma_engine_stop(engine);
	//wmb();
	mb();

	if(tx_to) {
		for(i=0;i<20;i++) {
			usleep(10);
			printf("stop status : %x\n",engine->regs->status);
		}
	}
	//free(engine);

	return 0;
}

int configfd;

int init(int ch)
{
        int waitint;
        int i;
        char *address = NULL;
        void *tx_queue;

	channel = ch;
        configfd = open("/sys/kernel/debug/coherent_buffers",O_RDWR);

        if(configfd < 0) {
                perror("Open call failed\n");
                return -1;
        }

        data = malloc(sizeof(struct mydata));

        read(configfd,data,sizeof(struct mydata));

        for(i = 0; i < NUM_BUFFERS; i++) {
		//              printf("i : %d tx phys : %p rx phys : %p\n",i,
		//i//(void *)data->coherent_mem_tx_dma_addr[i],(void *)data->coherent_mem_rx_dma_addr[i]);
		coherent_tx_hw_addresses[i] = data->coherent_mem_tx_dma_addr[i];
		coherent_rx_hw_addresses[i] = data->coherent_mem_rx_dma_addr[i];

        }


        tx_desc_virt =  mmap(NULL,PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, configfd, 0*PAGE_SIZE);
	
        if(tx_desc_virt == MAP_FAILED) {
                perror("mmap operation failed\n");
                return -1;
        } else {
                printf("tx_desc_virt at vaddr : %p\n",tx_desc_virt);
        }
	
	rx_desc_virt =  mmap(NULL,PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, configfd, 1*PAGE_SIZE);

        if(rx_desc_virt == MAP_FAILED) {
                perror("mmap operation failed\n");
                return -1;
        } else {
                printf("rx_desc_virt at vaddr : %p\n",rx_desc_virt);
        }

	rx_result_virt =  mmap(NULL,PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, configfd, 2*PAGE_SIZE);

	if(rx_result_virt == MAP_FAILED) {
		perror("mmap operation failed\n");
		return -1;
	} else {
		printf("rx_result_virt at vaddr : %p\n", rx_result_virt);
	}


	bar_base_user =  mmap(NULL, 64*1024, PROT_READ|PROT_WRITE, MAP_SHARED, configfd, 3*PAGE_SIZE);

        if(bar_base_user == MAP_FAILED) {
                perror("mmap operation failed\n");
                return -1;
        } else {
                printf("bar_base_user at vaddr : %p\n", bar_base_user);
        }


	for(i=0;i<64;i++) {

		rd_data[i] =  mmap(NULL, PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, configfd, (68+i)*PAGE_SIZE);

		if(rd_data[i]== MAP_FAILED) {
			perror("mmap operation failed\n");
			return -1;
		} else {

			rx_packet_buff[i] = (unsigned char *)rd_data[i];
			printf("rd_data[%d] at vaddr : %p whose paddr : %llx\n", i,rd_data[i],coherent_rx_hw_addresses[i]);
		}

	}

	for(i=0;i<64;i++) {

		wr_data[i] =  mmap(NULL, PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, configfd, (4+i)*PAGE_SIZE);

		if(wr_data[i] == MAP_FAILED) {
			perror("mmap operation failed\n");
			return -1;
		} else {
			tx_packet_buff[i] = (unsigned char *)wr_data[i];
			printf("wr_data[%d] at vaddr :  %p whose paddr  : %llx\n", i,wr_data[i],coherent_tx_hw_addresses[i]);
		}
	}


	
	
	printf(" bar 0 -> h2c eng -> %x\n", *bar_base_user);
        
	//close(configfd);
        return 0;
}

void uinit()
{
  close(configfd);
}



