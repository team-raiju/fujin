#ifndef BSP_VCP_H
#define BSP_VCP_H

#ifdef RELEASE
#define DEBUG_PRINT(...)
#else
#include <stdio.h>
#define DEBUG_PRINT(...) printf(__VA_ARGS__);
#endif

/* Virtual Com port */
void bsp_vcp_init();

#endif /* BSP_VCP_H */
