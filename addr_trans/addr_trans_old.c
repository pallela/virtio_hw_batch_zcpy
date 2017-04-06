#include<linux/module.h>
#include<linux/kernel.h>
#include<linux/init.h>
#include<linux/fs.h>
#include<linux/debugfs.h>
#include<linux/slab.h>
#include<linux/mm.h>
#include<linux/sched.h>
#include<linux/uaccess.h>
#include<asm/mman.h>
#include<linux/vmalloc.h>
#include<linux/mm_types.h>
#include<linux/page-flags.h>
#include<asm/io.h>
#include<linux/delay.h>
#include<linux/pagemap.h>
#include<asm/uaccess.h>

#define MAX_PAGES_TO_RECORD 1024*1024

static struct dentry *file;
static char userspace_data_buff[1024];

static struct page  *temppages[MAX_PAGES_TO_RECORD];
static struct page *temppage[1];
struct vm_area_struct *vmas[MAX_PAGES_TO_RECORD];
static int nr_pages;
static uint64_t address_offsets[MAX_PAGES_TO_RECORD];
static uint64_t chunk_sizes[MAX_PAGES_TO_RECORD];

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

ssize_t example_read (struct file *filp, char __user * userdata, size_t size, loff_t *offset)
{
	struct address_translation_table  *mem;
	int i;
	int copysize;
	uint64_t nr_entries;
	static int read_times = 0;

	printk("read_times : %d\n",read_times);

	if(read_times == 0) {
		nr_entries = nr_pages;
		copy_to_user(userdata,&nr_entries,sizeof(nr_entries));
		read_times = 1;
		return sizeof(nr_entries);
	}

	read_times = 0;

	copysize = nr_pages*sizeof(struct address_translation_entry) + sizeof(uint64_t);

	if(copysize > size) {
		printk("size : %d is less than minimum expected size : %d\n",(int) size,copysize);
		return -ENODATA;
	}

	mem = (struct address_translation_table *) kmalloc(copysize,GFP_KERNEL);

	if(!mem) {
		printk("func : %s kmalloc failed\n",__func__);
		return - ENODATA;
	}

	

	mem->nr_entries = nr_pages;
	printk("mem->nr_entries : %u\n",(unsigned int)mem->nr_entries);

	for(i=0;i<nr_pages;i++) {
		mem->table[i].physaddr = ( (uint64_t) page_to_phys(temppages[i]) ) + address_offsets[i];
		//mem->table[i].len = PAGE_SIZE*(1<<compound_order(temppages[i]));
		mem->table[i].len = chunk_sizes[i];

		printk("i : %d mem->table[i].physaddr : %llx mem->table[i].len : %llx\n",
		i,mem->table[i].physaddr,mem->table[i].len);
	}


	printk("copied %d bytes to userspace\n",copysize);
	

	copy_to_user(userdata,mem,copysize);

	kfree(mem);

	nr_pages = 0;

	return size;
}

ssize_t example_write (struct file *filp, const char __user * userdata, size_t size, loff_t *offset)
{
	uint64_t processaddr = 0;
	uint32_t datalen = 0;
	uint64_t physaddr;
	int i,j;
	int cur_page_size,length;
	int ret;

	nr_pages = 0;

	copy_from_user(userspace_data_buff,userdata,size%1024);

	userspace_data_buff[size%1024] = '\0';

	printk("\n user data (%d bytes) is \n %s\n",(uint32_t)size,userspace_data_buff);

	sscanf(userspace_data_buff,"%lx%x",&processaddr,&datalen);

	printk("userpace address : %llx and datalen : %x\n",processaddr,datalen);

	i = 0;
	nr_pages = 0;
	length = datalen;


	while(length > 0) {

		if(nr_pages >= MAX_PAGES_TO_RECORD) {
			printk("MAX Translation at a time reached max of %d ,please try with less range\n",MAX_PAGES_TO_RECORD);
			goto err_end_of_func;
		}

		down_read(&current->mm->mmap_sem);
		ret = get_user_pages(current,current->mm,(unsigned long int)processaddr,1,0,0,temppage,vmas);
		//printk("ret : %d\n",ret);

		if(ret >0) {
			nr_pages++;
		} else {
			printk("get_user_pages  errno : %d\n",ret);
			up_read(&current->mm->mmap_sem);
			goto err_end_of_func;
		}

		cur_page_size = PAGE_SIZE*(1<<compound_order(temppage[0]));
		address_offsets[i] = processaddr % cur_page_size;
		length -= (cur_page_size - address_offsets[i]);
		chunk_sizes[i] = (cur_page_size - address_offsets[i]);
		processaddr += (cur_page_size - address_offsets[i]);
		//printk("i : %d cur_page_size : %d  current page offset : %d current_processed_length : %d remaining length : %d\n",
		//	i,cur_page_size,address_offsets[i],chunk_sizes[i],length);
		temppages[i] = temppage[0];
		i++;
		page_cache_release(temppage[0]);
		up_read(&current->mm->mmap_sem);
	}


	printk("nr_pages : %d\n",nr_pages);
	return size;

err_end_of_func:

	return -ENOMEM;

}

int mmapfop_close(struct inode *inode,struct file *filp)
{
	return 0;
}

int mmapfop_open(struct inode *inode, struct file *filp)
{

	return 0;
}

static const struct file_operations mmap_fops = {
	.open  = mmapfop_open,
	.release = mmapfop_close,
	.read =  example_read,
	.write = example_write
};

static int __init mmap_module_init(void)
{
	file = debugfs_create_file("userpagesexample",0644,NULL,NULL,&mmap_fops);
	return 0;
}

static void __exit mmap_module_exit(void)
{
	debugfs_remove(file);
}

module_init(mmap_module_init);
module_exit(mmap_module_exit);
MODULE_LICENSE("GPL");
