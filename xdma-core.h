/* Switch debug printing on/off */
#define XDMA_DEBUG 1

/* Switch to enable/disable SD Accel extensions */
#define SD_ACCEL 0

/*
 * optimistic back-to-back I/O chaining
 * this is not compatible with descriptors above 32-bit address range,
 * as the implementation depends on atomically writing 32-bits in host
 * memory to link descriptors
 */
#define CHAIN_MULTIPLE_TRANSFERS 0

/* Enable/disable XDMA FPGA status during operation */
#define XDMA_STATUS_DUMPS 0

/* Use this definition to poll several times between calls to schedule */
#define NUM_POLLS_PER_SCHED 100

/* for test purposes only, not in default IP! */
#define DESC_COUNTER 0

/* testing purposes; request interrupt on each descriptor */
#define FORCE_IR_DESC_COMPLETED 0

/* Switch to control module licence */
#define XDMA_GPL 1

/* SECTION: Preprocessor macros/constants */

#define DRV_NAME "xdma"
#define XDMA_MINOR_BASE (0)
#define XDMA_MINOR_COUNT (255)

#define XDMA_KNOWN_REVISION (0x01)
#define XDMA_BAR_NUM (6)

/* maximum amount of register space to map */
#define XDMA_BAR_SIZE (0x8000UL)

#define XDMA_CHANNEL_NUM_MAX (2)
/*
 * interrupts per engine, rad2_vul.sv:237
 * .REG_IRQ_OUT	(reg_irq_from_ch[(channel*2) +: 2]),
 */
#define XDMA_ENG_IRQ_NUM (1)
#define MAX_EXTRA_ADJ (15)
#define RX_STATUS_EOP (1)

/* Target internal components on XDMA control BAR */
#define XDMA_OFS_INT_CTRL	(0x2000UL)
#define XDMA_OFS_CONFIG		(0x3000UL)

/* maximum number of bytes per transfer request */
#define XDMA_TRANSFER_MAX_BYTES (2048 * 4096)

/* maximum size of a single DMA transfer descriptor */
#define XDMA_DESC_MAX_BYTES ((1 << 18) - 1)

/* bits of the SG DMA control register */
#define XDMA_CTRL_RUN_STOP			(1UL << 0)
#define XDMA_CTRL_IE_DESC_STOPPED		(1UL << 1)
#define XDMA_CTRL_IE_DESC_COMPLETED		(1UL << 2)
#define XDMA_CTRL_IE_DESC_ALIGN_MISMATCH	(1UL << 3)
#define XDMA_CTRL_IE_MAGIC_STOPPED		(1UL << 4)
#define XDMA_CTRL_IE_IDLE_STOPPED		(1UL << 6)
#define XDMA_CTRL_IE_READ_ERROR			(0x1FUL << 9)
#define XDMA_CTRL_IE_DESC_ERROR			(0x1FUL << 19)
#define XDMA_CTRL_NON_INCR_ADDR			(1UL << 25)
#define XDMA_CTRL_POLL_MODE_WB			(1UL << 26)

/* bits of the SG DMA status register */
#define XDMA_STAT_BUSY			(1UL << 0)
#define XDMA_STAT_DESC_STOPPED		(1UL << 1)
#define XDMA_STAT_DESC_COMPLETED	(1UL << 2)
#define XDMA_STAT_ALIGN_MISMATCH	(1UL << 3)
#define XDMA_STAT_MAGIC_STOPPED		(1UL << 4)
#define XDMA_STAT_FETCH_STOPPED		(1UL << 5)
#define XDMA_STAT_IDLE_STOPPED		(1UL << 6)
#define XDMA_STAT_READ_ERROR		(0x1FUL << 9)
#define XDMA_STAT_DESC_ERROR		(0x1FUL << 19)

/* bits of the SGDMA descriptor control field */
#define XDMA_DESC_STOPPED_0	(0UL << 0)
#define XDMA_DESC_STOPPED_1	(1UL << 0)
#define XDMA_DESC_COMPLETED	(1UL << 1)
#define XDMA_DESC_EOP		(1UL << 4)

#define XDMA_PERF_RUN	(1UL << 0)
#define XDMA_PERF_CLEAR	(1UL << 1)
#define XDMA_PERF_AUTO	(1UL << 2)

#define MAGIC_ENGINE	0xEEEEEEEEUL
#define MAGIC_DEVICE	0xDDDDDDDDUL
#define MAGIC_CHAR	0xCCCCCCCCUL
#define MAGIC_BITSTREAM 0xBBBBBBBBUL

/* upper 16-bits of engine identifier register */
#define XDMA_ID_H2C 0x1fc0U
#define XDMA_ID_C2H 0x1fc1U

/* Specifies buffer size used for C2H AXI-ST mode */
#define RX_BUF_BLOCK 4096
#define RX_BUF_PAGES 256
#define RX_BUF_SIZE (RX_BUF_PAGES * RX_BUF_BLOCK)
#define RX_RESULT_BUF_SIZE (RX_BUF_PAGES * sizeof(struct xdma_result))

#define LS_BYTE_MASK 0x000000FFUL

#define BLOCK_ID_MASK 0xFFF00000UL
#define BLOCK_ID_HEAD 0x1FC00000UL

#define IRQ_BLOCK_ID 0x1fc20000UL
#define CONFIG_BLOCK_ID 0x1fc30000UL

#define WB_COUNT_MASK 0x00ffffffUL
#define WB_ERR_MASK (1UL << 31)
#define POLL_TIMEOUT_SECONDS 10

#define MAX_USER_IRQ 16

#define MAX_DESC_BUS_ADDR (0xffffffffULL)

#define DESC_MAGIC 0xAD4B0000UL

#define C2H_WB 0x52B4UL

#define MAX_NUM_ENGINES (XDMA_CHANNEL_NUM_MAX * 2)
#define H2C_CHANNEL_OFFSET 0x1000
#define SGDMA_OFFSET_FROM_CHANNEL 0x4000
#define CHANNEL_SPACING 0x100

#define BYPASS_MODE_SPACING 0x0100

/* obtain the 32 most significant (high) bits of a 32-bit or 64-bit address */
#define PCI_DMA_H(addr) ((addr >> 16) >> 16)
/* obtain the 32 least significant (low) bits of a 32-bit or 64-bit address */
#define PCI_DMA_L(addr) (addr & 0xffffffffUL)

/* SECTION: Enum definitions */

enum chardev_type {
	CHAR_XDMA_H2C,
	CHAR_XDMA_C2H
};

enum transfer_state {
	TRANSFER_STATE_NEW = 0,
	TRANSFER_STATE_SUBMITTED,
	TRANSFER_STATE_COMPLETED,
	TRANSFER_STATE_FAILED
};

enum shutdown_state {
	ENGINE_SHUTDOWN_NONE = 0,	/* No shutdown in progress */
	ENGINE_SHUTDOWN_REQUEST = 1,	/* engine requested to shutdown */
	ENGINE_SHUTDOWN_IDLE = 2	/* engine has shutdown and is idle */
};

enum dev_capabilities {
	CAP_64BIT_DMA = 2,
	CAP_64BIT_DESC = 4,
	CAP_ENGINE_WRITE = 8,
	CAP_ENGINE_READ = 16
};

/* SECTION: Structure definitions */

struct config_regs {
	unsigned int identifier;
	unsigned int reserved_1[4];
	unsigned int msi_enable;
};

/**
 * SG DMA Controller status and control registers
 *
 * These registers make the control interface for DMA transfers.
 *
 * It sits in End Point (FPGA) memory BAR[0] for 32-bit or BAR[0:1] for 64-bit.
 * It references the first descriptor which exists in Root Complex (PC) memory.
 *
 * @note The registers must be accessed using 32-bit (PCI DWORD) read/writes,
 * and their values are in little-endian byte ordering.
 */
struct engine_regs {
	volatile unsigned int identifier;
	volatile unsigned int control;
	volatile unsigned int control_w1s;
	volatile unsigned int control_w1c;
	unsigned int reserved_1[12];	/* padding */

	volatile unsigned int status;
	volatile unsigned int status_rc;
	volatile unsigned int completed_desc_count;
	volatile unsigned int alignments;
	unsigned int reserved_2[14];	/* padding */

	unsigned int poll_mode_wb_lo;
	unsigned int poll_mode_wb_hi;
	unsigned int interrupt_enable_mask;
	unsigned int interrupt_enable_mask_w1s;
	unsigned int interrupt_enable_mask_w1c;
	unsigned int reserved_3[9];	/* padding */

	unsigned int perf_ctrl;
	unsigned int perf_cyc_lo;
	unsigned int perf_cyc_hi;
	unsigned int perf_dat_lo;
	unsigned int perf_dat_hi;
	unsigned int perf_pnd_lo;
	unsigned int perf_pnd_hi;
} ;

struct engine_sgdma_regs {
	unsigned int identifier;
	unsigned int reserved_1[31];	/* padding */

	/* bus address to first descriptor in Root Complex Memory */
	volatile unsigned int first_desc_lo;
	volatile unsigned int first_desc_hi;
	/* number of adjacent descriptors at first_desc */
	unsigned int first_desc_adjacent;
} ;


struct interrupt_regs {
	unsigned int identifier;
	unsigned int user_int_enable;
	unsigned int user_int_enable_w1s;
	unsigned int user_int_enable_w1c;
	unsigned int channel_int_enable;
	unsigned int channel_int_enable_w1s;
	unsigned int channel_int_enable_w1c;
	unsigned int reserved_1[9];	/* padding */

	unsigned int user_int_request;
	unsigned int channel_int_request;
	unsigned int user_int_pending;
	unsigned int channel_int_pending;
	unsigned int reserved_2[12];	/* padding */

	unsigned int user_msi_vector[8];
	unsigned int channel_msi_vector[8];

} ;
/**
 * Descriptor for a single contiguous memory block transfer.
 *
 * Multiple descriptors are linked by means of the next pointer. An additional
 * extra adjacent number gives the amount of extra contiguous descriptors.
 *
 * The descriptors are in root complex memory, and the bytes in the 32-bit
 * words must be in little-endian byte ordering.
 */
struct xdma_desc {
	unsigned int control;
	unsigned int bytes;		/* transfer length in bytes */
	unsigned int src_addr_lo;	/* source address (low 32-bit) */
	unsigned int src_addr_hi;	/* source address (high 32-bit) */
	unsigned int dst_addr_lo;	/* destination address (low 32-bit) */
	unsigned int dst_addr_hi;	/* destination address (high 32-bit) */
	/*
	 * next descriptor in the single-linked list of descriptors;
	 * this is the PCIe (bus) address of the next descriptor in the
	 * root complex memory
	 */
	unsigned int next_lo;		/* next desc address (low 32-bit) */
	unsigned int next_hi;		/* next desc address (high 32-bit) */
} ;

/* 32 bytes (four 32-bit words) or 64 bytes (eight 32-bit words) */
struct xdma_result {
	unsigned int status;
	unsigned int length;
	unsigned int reserved_1[6];	/* padding */
} ;

/* Structure for polled mode descriptor writeback */
struct xdma_poll_wb {
	unsigned int completed_desc_count;
	unsigned int reserved_1[7];
} ;

struct xdma_engine {
	unsigned long magic;	/* structure ID for sanity checks */
	char *name;		/* name of this engine */

	/* HW register address offsets */
	volatile struct engine_regs *regs;		/* Control reg BAR offset */
	volatile struct engine_sgdma_regs *sgdma_regs;	/* SGDAM reg BAR offset */
	unsigned int bypass_offset;			/* Bypass mode BAR offset */

	/* Engine state, configuration and flags */
	enum shutdown_state shutdown;	/* engine shutdown mode */
	int running;		/* flag if the driver started engine */
	int streaming;		/* flag if AXI-ST engine */
	int non_incr_addr;	/* flag if non-incremental addressing used */
	int dir_to_dev;		/* direction of this engine */
	int addr_align;		/* source/dest alignment in bytes */
	int len_granularity;	/* transfer length multiple */
	int addr_bits;		/* HW datapath address width */
	int channel;		/* engine indices */
	int number_in_channel;	/* engine indices */
	int max_extra_adj;	/* descriptor prefetch capability */
	int desc_dequeued;	/* num descriptors of completed transfers */
	unsigned int status;		/* last known status of device */
	unsigned int interrupt_enable_mask_value;/* only used for MSIX mode to store per-engine interrupt mask value */

	/* Transfer list management */

	/* Members applicable to AXI-ST C2H (cyclic) transfers */
	unsigned int *rx_result_buffer_virt;		/* virt addr for transfer */
	unsigned long int rx_result_buffer_bus;	/* bus addr for transfer */

	/* Members associated with polled mode support */
	unsigned int *poll_mode_addr_virt;	/* virt addr for descriptor writeback */
	unsigned long int poll_mode_bus;	/* bus addr for descriptor writeback */

};

struct xdma_ocl_clockwiz {
	/* target frequency */
	unsigned ocl;
	/* clockout divider */
	unsigned divide;
	/* clockout divider fractional part */
	unsigned divide_frac;
};

struct xdma_bitstream_container {
	/* MAGIC_BITSTREAM == 0xBBBBBBBBUL */
	unsigned long magic;
	char *clear_bitstream;
	unsigned int clear_bitstream_length;
};


struct xdma_irq {
	struct xdma_dev *lro;		/* parent device */
	unsigned int events_irq;			/* accumulated IRQs */
};
