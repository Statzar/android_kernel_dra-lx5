

#ifndef BFR_CHIPSETS_H
#define BFR_CHIPSETS_H


/*----includes-----------------------------------------------------------------------*/

#include <linux/types.h>
#include <chipset_common/bfmr/public/bfmr_public.h>


/*----c++ support--------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif


/*----export prototypes---------------------------------------------------------------*/


/*----export macroes-----------------------------------------------------------------*/


/*----global variables-----------------------------------------------------------------*/


/*----export function prototypes--------------------------------------------------------*/

bool bfr_safe_mode_has_been_enabled(void);
int bfr_get_full_path_of_rrecord_part(char **path_buf);

#ifdef __cplusplus
}
#endif

#endif /* BFR_CHIPSETS_H */


