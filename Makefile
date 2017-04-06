all:
	gcc -g -O0 vhost_funcs.c  pcap_rxtx.c dma_rxtx.c  vhostnetpci_test.c address_trans_api.c -o xvhostusernet -lpthread -lpcap 
clean:
	rm -rf  xvhostusernet

