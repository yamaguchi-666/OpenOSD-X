#ifndef __TARGET_H
#define __TARGET_H

#include <stdint.h>


#define CAL_FREQ_SIZE 9
#define POWER_LEVEL_LABEL_LENGTH    3


#ifdef TARGET_BREAKOUTBOARD

#define TEMP_WARNING_DEG        70  /* warning temperrature (degree) */
#define TEMP_WARNING_POWERINDEX 1   /* warning power (vpd_table.calVpd[TEMP_WARNING_POWERINDEX]) */
#define TEMP_DANGER_DEG         90  /* danger temparature (degree) */
#define TEMP_DANGER_POWERINDEX  0   /* danger power (vpd_table.calVpd[TEMP_DANGER_POWERINDEX]) */
#define VTX_DEFAULT_POWER_INDEX     3    /* Default power level index (25mW) */
#define VTX_TABLE_NEW_POWER_COUNT   4    /* vtx table power count */
#define SA_NUM_POWER_LEVELS         VTX_TABLE_NEW_POWER_COUNT
#define CAL_DBM_SIZE 2
#define VREF_MAX_MV  2850           /* max vref voltage */

#endif


#ifdef TARGET_YK044_V3

#define TEMP_WARNING_DEG        70  /* warning temperrature (degree) */
#define TEMP_WARNING_POWERINDEX 1   /* warning power (vpd_table.calVpd[TEMP_WARNING_POWERINDEX]) */
#define TEMP_DANGER_DEG         90  /* danger temparature (degree) */
#define TEMP_DANGER_POWERINDEX  0   /* danger power (vpd_table.calVpd[TEMP_DANGER_POWERINDEX]) */
#define VTX_DEFAULT_POWER_INDEX     3    /* Default power level index (25mW) */
#define VTX_TABLE_NEW_POWER_COUNT   4    /* vtx table power count */
#define SA_NUM_POWER_LEVELS         VTX_TABLE_NEW_POWER_COUNT
#define CAL_DBM_SIZE 2
#define VREF_MAX_MV  2950           /* max vref voltage */

#endif


#ifndef TARGET_NOVTX
typedef struct vpd_table_def {
    char magic[4];
    uint16_t calFreqs[CAL_FREQ_SIZE];
    uint8_t calDBm[CAL_DBM_SIZE];
    uint16_t calVpd[CAL_DBM_SIZE][CAL_FREQ_SIZE];
} vpd_table_t;
extern const vpd_table_t vpd_table;
extern uint8_t saPowerLevelsLut[SA_NUM_POWER_LEVELS];
extern uint8_t saPowerLevelsLabel[SA_NUM_POWER_LEVELS * POWER_LEVEL_LABEL_LENGTH];
#endif


#endif

