#ifndef __ADDR_TRANS_API__
#define __ADDR_TRANS_API__
#include<stdint.h>
struct address_translation_entry
{
        uint64_t physaddr;
        uint64_t len;
}__attribute__ ((packed));

struct address_translation_table
{
        uint64_t nr_entries;
        struct address_translation_entry table[0];
}__attribute__ ((packed));


struct address_translation_table * get_physaddr_translation(void *mem, int len, int fd);
void free_addr_trans_table(struct address_translation_table *table);


#endif
