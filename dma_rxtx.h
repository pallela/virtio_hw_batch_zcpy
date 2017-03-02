int dma_rx_burst(uint64_t *pkts, unsigned int *max_pkt_len, unsigned int *recv_len, int pkt_offset, int max_num_pkts);
int dma_tx_burst(uint64_t *pkts, unsigned int *pkt_len, int pkt_offset, int num_pkts);
void *dma_rx(int *pkt_len);
int dma_tx(char * pkt, int pkt_len, int pkt_offset);
int init(int ch);
void uinit();



