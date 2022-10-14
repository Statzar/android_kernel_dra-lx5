/*
 *  IBM System z Huge TLB Page Support for Kernel.
 *
 *    Copyright IBM Corp. 2007
 *    Author(s): Gerald Schaefer <gerald.schaefer@de.ibm.com>
 */

#include <linux/mm.h>
#include <linux/hugetlb.h>

static inline pmd_t __pte_to_pmd(pte_t pte)
{
	pmd_t pmd;


	if (pte_present(pte)) {
		pmd_val(pmd) = pte_val(pte) & PAGE_MASK;
		pmd_val(pmd) |= (pte_val(pte) & _PAGE_READ) >> 4;
		pmd_val(pmd) |= (pte_val(pte) & _PAGE_WRITE) >> 4;
		pmd_val(pmd) |=	(pte_val(pte) & _PAGE_INVALID) >> 5;
		pmd_val(pmd) |= (pte_val(pte) & _PAGE_PROTECT);
		pmd_val(pmd) |= (pte_val(pte) & _PAGE_DIRTY) << 10;
		pmd_val(pmd) |= (pte_val(pte) & _PAGE_YOUNG) << 10;
		pmd_val(pmd) |= (pte_val(pte) & _PAGE_SOFT_DIRTY) << 13;
	} else
		pmd_val(pmd) = _SEGMENT_ENTRY_INVALID;
	return pmd;
}

static inline pte_t __pmd_to_pte(pmd_t pmd)
{
	pte_t pte;


	if (pmd_present(pmd)) {
		pte_val(pte) = pmd_val(pmd) & _SEGMENT_ENTRY_ORIGIN_LARGE;
		pte_val(pte) |= _PAGE_LARGE | _PAGE_PRESENT;
		pte_val(pte) |= (pmd_val(pmd) & _SEGMENT_ENTRY_READ) << 4;
		pte_val(pte) |= (pmd_val(pmd) & _SEGMENT_ENTRY_WRITE) << 4;
		pte_val(pte) |= (pmd_val(pmd) & _SEGMENT_ENTRY_INVALID) << 5;
		pte_val(pte) |= (pmd_val(pmd) & _SEGMENT_ENTRY_PROTECT);
		pte_val(pte) |= (pmd_val(pmd) & _SEGMENT_ENTRY_DIRTY) >> 10;
		pte_val(pte) |= (pmd_val(pmd) & _SEGMENT_ENTRY_YOUNG) >> 10;
		pte_val(pte) |= (pmd_val(pmd) & _SEGMENT_ENTRY_SOFT_DIRTY) >> 13;
	} else
		pte_val(pte) = _PAGE_INVALID;
	return pte;
}

void set_huge_pte_at(struct mm_struct *mm, unsigned long addr,
		     pte_t *ptep, pte_t pte)
{
	pmd_t pmd = __pte_to_pmd(pte);

	pmd_val(pmd) |= _SEGMENT_ENTRY_LARGE;
	*(pmd_t *) ptep = pmd;
}

pte_t huge_ptep_get(pte_t *ptep)
{
	pmd_t pmd = *(pmd_t *) ptep;

	return __pmd_to_pte(pmd);
}

pte_t huge_ptep_get_and_clear(struct mm_struct *mm,
			      unsigned long addr, pte_t *ptep)
{
	pmd_t *pmdp = (pmd_t *) ptep;
	pte_t pte = huge_ptep_get(ptep);

	pmdp_flush_direct(mm, addr, pmdp);
	pmd_val(*pmdp) = _SEGMENT_ENTRY_EMPTY;
	return pte;
}

pte_t *huge_pte_alloc(struct mm_struct *mm,
			unsigned long addr, unsigned long sz)
{
	pgd_t *pgdp;
	pud_t *pudp;
	pmd_t *pmdp = NULL;

	pgdp = pgd_offset(mm, addr);
	pudp = pud_alloc(mm, pgdp, addr);
	if (pudp)
		pmdp = pmd_alloc(mm, pudp, addr);
	return (pte_t *) pmdp;
}

pte_t *huge_pte_offset(struct mm_struct *mm, unsigned long addr)
{
	pgd_t *pgdp;
	pud_t *pudp;
	pmd_t *pmdp = NULL;

	pgdp = pgd_offset(mm, addr);
	if (pgd_present(*pgdp)) {
		pudp = pud_offset(pgdp, addr);
		if (pud_present(*pudp))
			pmdp = pmd_offset(pudp, addr);
	}
	return (pte_t *) pmdp;
}

int pmd_huge(pmd_t pmd)
{
	return pmd_large(pmd);
}

int pud_huge(pud_t pud)
{
	return 0;
}
