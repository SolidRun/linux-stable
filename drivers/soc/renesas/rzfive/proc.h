#include <asm/io.h>
#include <asm/page.h>

int cpu_l1c_status(void);
void cpu_icache_enable(void *info);
void cpu_icache_disable(void *info);
void cpu_dcache_enable(void *info);
void cpu_dcache_disable(void *info);
uint32_t cpu_l2c_ctl_status(void);

void cpu_dma_inval_range(unsigned long start, unsigned long end);
void cpu_dma_wb_range(unsigned long start, unsigned long end);
void cpu_l2c_inval_range(unsigned long pa, unsigned long size);
void cpu_l2c_wb_range(unsigned long pa, unsigned long size);

/*
 * struct andesv5_cache_info
 * The member of this struct is dupilcated to some content of struct cacheinfo
 * to reduce the latence of searching dcache inforamtion in andesv5/cache.c.
 * At current only dcache-line-size is needed. when the content of
 * andesv5_cache_info has been initilized by function fill_cpu_cache_info(),
 * member init_done is set as true
 */
struct andesv5_cache_info {
	bool init_done;
	int dcache_line_size;
};
