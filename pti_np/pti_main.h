#ifndef pti_main_h_123
#define pti_main_h_123

#include "pti_public.h"

/* *****************************
 * Global vars
 */

/* not good but for first shot we make vars globally accessible */
extern STPTI_TCParameters_t tc_params;
extern TCDevice_t* myTC;

/* ******************************
 * Functions
 */

void pti_main_loadtc(struct stpti* pti);

#endif
