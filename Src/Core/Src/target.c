
#include "target.h"

#ifdef TARGET_BREAKOUTBOARD

const vpd_table_t vpd_table = {
    .calFreqs = {5600, 5650, 5700, 5750, 5800, 5850, 5900, 5950, 6000},
    .calDBm = {14, 20},
    .calVpd = {
        {1300,1330,1345,1400,1480,1590,1670,1710,1760},
        {1910,1970,1980,2120,2270,2430,2540,2620,2750}
    }
};

#define RACE_MODE                   2
uint8_t saPowerLevelsLut[SA_NUM_POWER_LEVELS] = {1, RACE_MODE, 14, 20};
uint8_t saPowerLevelsLabel[SA_NUM_POWER_LEVELS * POWER_LEVEL_LABEL_LENGTH] = {'0', ' ', ' ',
                                                                              'R', 'C', 'E',
                                                                              '2', '5', ' ',
                                                                              '1', '0', '0'};

#endif

#ifdef TARGET_YK044_V3

const vpd_table_t vpd_table = {
    .calFreqs = {5600, 5650, 5700, 5750, 5800, 5850, 5900, 5950, 6000},
    .calDBm = {14, 20},
    .calVpd = { //T.B.D.
        {1300,1330,1345,1400,1480,1590,1670,1710,1760},
        {1910,1970,1980,2120,2270,2430,2540,2620,2750}
    }
};

#define RACE_MODE                   2
uint8_t saPowerLevelsLut[SA_NUM_POWER_LEVELS] = {1, RACE_MODE, 14, 20};
uint8_t saPowerLevelsLabel[SA_NUM_POWER_LEVELS * POWER_LEVEL_LABEL_LENGTH] = {'0', ' ', ' ',
                                                                              'R', 'C', 'E',
                                                                              '2', '5', ' ',
                                                                              '1', '0', '0'};

#endif
