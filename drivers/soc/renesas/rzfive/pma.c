#include <asm/sbi.h>
#include <asm/csr.h>

#include "pma.h"
#include "proc.h"
#include "csr.h"

long sbi_set_pma(void *arg)
{
	struct sbiret ret;

	phys_addr_t offset = ((struct pma_arg_t*)arg)->offset;
	unsigned long vaddr = ((struct pma_arg_t*)arg)->vaddr;
	size_t size = ((struct pma_arg_t*)arg)->size;
	size_t entry_id = ((struct pma_arg_t*)arg)->entry_id;
	ret = sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_SET_PMA, offset, vaddr, size, entry_id, 0, 0);

	return ret.value;
}

void sbi_free_pma(unsigned long entry_id)
{
	sbi_ecall(SBI_EXT_ANDES, SBI_EXT_ANDES_FREE_PMA, entry_id, 0, 0, 0, 0, 0);
}
