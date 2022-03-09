

#ifndef _RISCV_PMA_H
#define _RISCV_PMA_H

#include <linux/types.h>

struct pma_arg_t {
	phys_addr_t offset;
	unsigned long vaddr;
	size_t size;
	size_t entry_id;
};

long sbi_set_pma(void *arg);
void sbi_free_pma(unsigned long entry_id);
long sbi_probe_pma(void);

#endif
