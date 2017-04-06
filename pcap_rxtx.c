#include"virtio.h"
#include <pcap.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/eventfd.h>
#include<semaphore.h>
#include"dma_rxtx.h"

#define CACHE_ALIGN 64
static inline clflush_multiple(volatile void *__p,int size)
{
        int i,cnt;
        void *addr;

        addr = (void *) ((uintptr_t)__p - ((uintptr_t)__p)%CACHE_ALIGN);
        //printf("__p  : %llx and addr : %llx\n",__p,addr);
        cnt =  size/CACHE_ALIGN + 1;

        for(i=0;i<cnt;i++) {
                //asm volatile("clflush %0" : "+m" (*(volatile char *)__p));

                asm volatile("clflush %0" : "+m" (*(volatile  char *) addr));
                addr += CACHE_ALIGN;
        }
}


static bpf_u_int32 net;		/* Our IP */
pcap_t *pcap_init(char *iname)
{
	pcap_t *handle;			/* Session handle */
	char *dev;			/* The device to sniff on */
	char errbuf[PCAP_ERRBUF_SIZE];	/* Error string */
	bpf_u_int32 mask;		/* Our netmask */

	/* Define the device */

	printf("vvdn debug :opening pcap on interface %s\n",iname);
	dev = iname;

	/* Find the properties for the device */
	if (pcap_lookupnet(dev, &net, &mask, errbuf) == -1) {
		fprintf(stderr, "Couldn't get netmask for device %s: %s\n", dev, errbuf);
		net = 0;
		mask = 0;
	}
	/* Open the session in promiscuous mode */
	handle = pcap_open_live(dev, BUFSIZ, 1, 1, errbuf);
	if (handle == NULL) {
		fprintf(stderr, "Couldn't open device %s: %s\n", dev, errbuf);
		return;
	}

	return handle;
}

extern sem_t tx_start_wait_sem,rx_start_wait_sem;
extern sem_t tx_clean_wait_sem,rx_clean_wait_sem;

extern volatile struct vring_desc *rx_desc_base; 
extern volatile struct vring_used  *rx_used; 
extern volatile struct vring_avail *rx_avail;
extern volatile int rxirqfd;
extern int rx_desc_count;
extern volatile connected_to_guest;
extern int vhost_hlen;
extern uint64_t *coherent_rx_hw_addresses;
extern char **rx_packet_buff;
extern uint64_t *rx_packet_physaddrs;

uint64_t guestphyddr_to_vhostvadd(uint64_t gpaddr);

unsigned char mac_address[6] = {0xb8,0x2a,0x72,0xc4,0x26,0x45};
unsigned char broadcast_mac_address[6] = {0xff,0xff,0xff,0xff,0xff,0xff};


#define RX_BURST 0
#define RX_BATCH_SIZE 64
void *pcap_rx_thread(void *arg)
{
	const u_char *packet;		/* The actual packet */
	void  *tmp;
	int i,j,rx_len;
	uint16_t rx_desc_num = 0,rx_header_desc_num = 0,rx_avail_ring_no = 0,rx_used_ring_no = 0;
	unsigned char  *packet_addr;
	uint32_t packet_len,packet_len2;
	uint16_t avail_idx,used_idx;
	struct virtio_net_hdr_mrg_rxbuf *tmpheader;
	int rx_cleanup_required;
	int new_pkts_count;
	int rx_max_pkts_len[64];
	int recv_pkts_len[64];
	int rx_desc_depth;
	int rx_recv_len;
	int copylen;
	int new_avail_descs;
	int rx_avail_ring_no_tmp;


	printf("starting rx thread\n");
	while(1) {

		if(connected_to_guest) {
			avail_idx = rx_avail->idx;
			used_idx = rx_used->idx;
			new_avail_descs = avail_idx- used_idx;
			if(!new_avail_descs) {
				usleep(1);
				continue;
			}

			if(new_avail_descs > RX_BATCH_SIZE) {
				new_avail_descs = RX_BATCH_SIZE;
			}

			#if !RX_BURST
			if( VHOST_SUPPORTED_FEATURES &( 1ULL << VIRTIO_NET_F_MRG_RXBUF) ) {
				rx_desc_num = rx_avail->ring[rx_avail_ring_no];
				tmp = (void *)guestphyddr_to_hostphysaddr(rx_desc_base[rx_desc_num].addr);
				rx_packet_physaddrs[0] = (uint64_t) (tmp + vhost_hlen);
				rx_len = rx_desc_base[rx_desc_num].len - vhost_hlen;
				//tmp = (void *)guestphyddr_to_vhostvadd(rx_desc_base[rx_desc_num].addr);
				//clflush_multiple(tmp+vhost_hlen,rx_len);
			}
			else {
				rx_desc_num = rx_avail->ring[rx_avail_ring_no];
				rx_desc_num = rx_desc_base[rx_desc_num].next;
				tmp = (void *)guestphyddr_to_hostphysaddr(rx_desc_base[rx_desc_num].addr);
				rx_packet_physaddrs[0] = (uint64_t) tmp;
				rx_len = rx_desc_base[rx_desc_num].len;
				//tmp = (void *)guestphyddr_to_vhostvadd(rx_desc_base[rx_desc_num].addr);
				//clflush_multiple(tmp,rx_len);
			}

			//printf("received packet buff : %d bytes at physical address %llx\n",rx_len,rx_packet_physaddrs[0]);
			packet = dma_rx(&rx_len,rx_packet_physaddrs[0]);
			if(packet) {
				new_pkts_count = 1;
				recv_pkts_len[0] = rx_len;
				//printf("received packet : %d bytes at physical address %llx\n",rx_len,rx_packet_physaddrs[0]);
			}else {
				new_pkts_count = 0;
				recv_pkts_len[0] = 0;
				//printf("Not received packet\n");
			}
			#else
			rx_avail_ring_no_tmp = rx_avail_ring_no;
			for(j=0;j<new_avail_descs;j++) {

				if( VHOST_SUPPORTED_FEATURES &( 1ULL << VIRTIO_NET_F_MRG_RXBUF) ) {
					rx_desc_num = rx_avail->ring[rx_avail_ring_no_tmp];
					tmp = (void *)guestphyddr_to_hostphysaddr(rx_desc_base[rx_desc_num].addr);
					rx_packet_physaddrs[j] = (uint64_t) tmp + (uint64_t) vhost_hlen;
					recv_pkts_len[j] = rx_desc_base[rx_desc_num].len - vhost_hlen;
				}
				else {
					rx_desc_num = rx_avail->ring[rx_avail_ring_no_tmp];
					rx_desc_num = rx_desc_base[rx_desc_num].next;
					tmp = (void *)guestphyddr_to_hostphysaddr(rx_desc_base[rx_desc_num].addr);
					rx_packet_physaddrs[j] = (uint64_t) tmp;
					recv_pkts_len[j] = rx_desc_base[rx_desc_num].len;
				}
				rx_avail_ring_no_tmp  = (rx_avail_ring_no_tmp + 1)%rx_desc_count;

			}

			//printf("recv pkts max cnt  : %d\n",new_avail_descs);
			new_pkts_count = dma_rx_burst(rx_packet_physaddrs,recv_pkts_len,recv_pkts_len,0,new_avail_descs);
			if(!new_pkts_count) {
				//printf("zero pkts received, received new_pkts_count : %d\n",new_pkts_count);
				usleep(1);
				continue;
			}
			else {
				//printf("received new_pkts_count : %d\n",new_pkts_count);
			}
			#endif
		}

		if(new_pkts_count > 0 && connected_to_guest) {

			for(i=0;i<new_pkts_count;i++) {

				rx_desc_num = rx_avail->ring[rx_avail_ring_no];
				rx_header_desc_num = rx_desc_num;
				tmp = (void *)guestphyddr_to_vhostvadd(rx_desc_base[rx_desc_num].addr);
				memset(tmp,0,vhost_hlen);
				
				if( VHOST_SUPPORTED_FEATURES &( 1ULL << VIRTIO_NET_F_MRG_RXBUF) ) {
					tmpheader = (struct virtio_net_hdr_mrg_rxbuf *)tmp;
					tmpheader->num_buffers = 1;
				}


				packet_len = recv_pkts_len[i];

				rx_avail_ring_no = (rx_avail_ring_no + 1)%rx_desc_count;
				wmb();

				rx_used->ring[rx_used_ring_no].id = rx_header_desc_num;
				rx_used->ring[rx_used_ring_no].len = vhost_hlen + packet_len;
				rx_used_ring_no = (rx_used_ring_no +1)%rx_desc_count;
				wmb();
				rx_used->idx++;
				wmb();
				eventfd_write(rxirqfd, (eventfd_t)1);
				wmb();
				//printf("Informed VM about new packet , rx_used->idx : %u packet_len : %u rx_avail_ring_no : %u rx_used_ring_no : %u\n"
				//,rx_used->idx,packet_len,rx_avail_ring_no,rx_used_ring_no);
			}
		}
		else if(!connected_to_guest) {
			rx_avail_ring_no = 0;
			rx_used_ring_no = 0;
			if(rx_cleanup_required) {
				rx_cleanup_required = 0;
				printf("rx thread , cleanup done\n");
				sem_post(&rx_clean_wait_sem);
			}
			printf("rx  thread , waiting for connection\n");
			sem_wait(&rx_start_wait_sem);
			rx_cleanup_required = 1;
			printf("rx thread , starting processing now\n");
			{
				int i;
				for(i=0;i<64;i++) {
					rx_max_pkts_len[i] = 4096-vhost_hlen;
				}
			}
		}
	}
	/* And close the session */
}

char pcap_tx_err_str[1024];
void pcap_tx(pcap_t *handle, void *packet,int size)
{
	int ret;
	//printf("to tx : %d  bytes\n",size);
	ret = pcap_inject(handle,packet,size);

	if(ret == -1) {
		printf("tx packet failed : %s\n",pcap_geterr(handle));
	}
	
}
