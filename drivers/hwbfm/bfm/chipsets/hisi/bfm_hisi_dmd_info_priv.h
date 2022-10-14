

#ifndef BFM_HISI_DMD_INFO_H
#define BFM_HISI_DMD_INFO_H


/*----c++ support-------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif


/************************************************************
Function:       get_dmd_err_num
Description:   get the most err number from DMD file about boot fail.
Input:           NA
Output:         errNumber     the dmd err number
                    count            the times of err occur 
                    errName       the mod name of this err
Return:         0:   find adapt err number
                    -1: not find.
************************************************************/
int get_dmd_err_num(unsigned int* errNum, unsigned int* count, char* errName);

#ifdef __cplusplus
}
#endif

#endif

