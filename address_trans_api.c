#include"address_trans_api.h"
#include<stdio.h>
#include<string.h>
#include<stdlib.h>

#define PAGE_SIZE 4096
#define PAGE_ALIGN(addr) ((addr/PAGE_SIZE)*PAGE_SIZE)

struct address_translation_table * get_physaddr_translation(void *mem, int len,int fd)
{
	char address_and_len[100];
	int wr_bytes,rd_bytes,ret;
	int entry_count,i;
	struct address_translation_table *table;

	sprintf(address_and_len,"%lx %x",(unsigned long int)mem,(unsigned int)len);

	wr_bytes = write(fd,address_and_len,strlen(address_and_len));

	perror("write : ");

	printf("Done with address translation : ret : %d\n",wr_bytes);


	if(wr_bytes == strlen(address_and_len)) {

		read(fd,&entry_count,sizeof(entry_count));
		printf("entry_count : %d\n",entry_count);

		rd_bytes = entry_count*sizeof(struct address_translation_entry) + sizeof(uint64_t);
		table = (struct address_translation_table *) malloc(rd_bytes);
		ret = read(fd,table,rd_bytes);
		perror("read: ");

		if(ret == rd_bytes) {
			for(i=0;i<table->nr_entries;i++) {
				printf("physaddr : %llx and size : %llx\n",(unsigned long long int)table->table[i].physaddr,(unsigned long long int)table->table[i].len);
			}
		}

		//free(table); /* To be freed by caller */

	}
	else {
		printf("write issue : wr_bytes : %d and strlen(address_and_len = %d\n",wr_bytes,(int)strlen(address_and_len));
	}

	return table;

}


void free_addr_trans_table(struct address_translation_table *table)
{
	free(table);
}

