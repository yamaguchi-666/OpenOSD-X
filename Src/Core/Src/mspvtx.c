/**
 * @file mspVtx.c
 * @brief Module for handling MSP communication between OpenVTx and a Flight Controller (FC).
 *
 * It verifies and synchronizes the VTX table on the FC at startup, and then monitors 
 * and applies setting changes from the FC during operation.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "target.h"
#include "vtx.h"
#include "log.h"
#include "msp.h"
#include "uart_dma.h"
#include "setting.h"
#include "mspvtx.h"

#ifndef TARGET_NOVTX

// ====================================================================================
// MSP Protocol Definitions
// ====================================================================================
#define MSP_HEADER_SIZE             8    // MSP v2 header size


// ====================================================================================
// Internal Constants
// ====================================================================================
#define FC_QUERY_PERIOD_MS          200  // Query interval to FC (ms)

// VTX Table Default Values
#define VTX_DEFAULT_BAND_CHAN_INDEX 27   // Default channel index (F4, 5800MHz)
#define VTX_DEFAULT_PITMODE_STATE   0    // Default PIT mode state (OFF)
#define VTX_DEFAULT_LP_DISARM_STATE 0    // Default Low Power Disarm state (OFF)

// Values used in clearVtxTable()
#define VTX_TABLE_SHOULD_BE_CLEARED 1
#define VTX_TABLE_NEW_BAND_COUNT    5//6
#define CHANNEL_COUNT 8
#define FREQ_TABLE_SIZE 40//48
#define IS_FACTORY_BAND                 0
#define RACE_MODE_POWER                 14 // dBm

/**
 * @brief State machine for managing MSP communication between VTX and FC.
 */
typedef enum
{
    /** @brief Startup: Check the size of the VTX table (number of bands, power levels) on the FC. */
    MSP_STATE_GET_VTX_TABLE_SIZE = 0,
    /** @brief Check if the power level definitions on the FC match the OpenVTx definitions, one by one. */
    CHECK_POWER_LEVELS,
    /** @brief Check if the band definitions on the FC match the OpenVTx definitions, one by one. */
    CHECK_BANDS,
    /** @brief After table correction, set safe default values on the FC. */
    SET_DEFAULTS,
    /** @brief Request the FC to write the corrected table to its EEPROM. */
    SEND_EEPROM_WRITE,
    /** @brief After initialization, monitor for setting changes from the FC (normal operation). */
    MONITORING,
} mspState_e;

char* mspStaetString[] = {
    "MSP_STATE_GET_VTX_TABLE_SIZE",
    "CHECK_POWER_LEVELS",
    "CHECK_BANDS",
    "SET_DEFAULTS",
    "SEND_EEPROM_WRITE",
    "MONITORING",
};

// ====================================================================================
// Struct Definitions
// ====================================================================================
/**
 * @brief Structure to hold the payload of an MSP_VTX_CONFIG message.
 * @note Corresponds to Betaflight's mspVtxConfig_t.
 */
typedef struct
{
    uint8_t vtxType;
    uint8_t band;
    uint8_t channel;
    uint8_t power;
    uint8_t pitmode;
    uint8_t freqLSB;
    uint8_t freqMSB;
    uint8_t deviceIsReady;
    uint8_t lowPowerDisarm;
    uint8_t pitModeFreqLSB;
    uint8_t pitModeFreqMSB;
    uint8_t vtxTableAvailable;
    uint8_t bands;
    uint8_t channels;
    uint8_t powerLevels;
} mspVtxConfigStruct;


#ifdef TARGET_BREAKOUTBOARD

const uint8_t channelFreqLabel[48] = {
    'B', 'A', 'N', 'D', '_', 'A', ' ', ' ', // A
    'B', 'A', 'N', 'D', '_', 'B', ' ', ' ', // B
    'B', 'A', 'N', 'D', '_', 'E', ' ', ' ', // E
    'F', 'A', 'T', 'S', 'H', 'A', 'R', 'K', // F
    'R', 'A', 'C', 'E', ' ', ' ', ' ', ' ', // R
    'R', 'A', 'C', 'E', '_', 'L', 'O', 'W', // L
};

const uint8_t bandLetter[6] = {'A', 'B', 'E', 'F', 'R', 'L'};

uint16_t channelFreqTable[FREQ_TABLE_SIZE] = {
    5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725, // A
    5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866, // B
    5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945, // E
    5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880, // F
    5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917, // R
    5333, 5373, 5413, 5453, 5493, 5533, 5573, 5613  // L
};

#endif

#ifdef TARGET_YK044_V3

const uint8_t channelFreqLabel[40] = {
    'B', 'A', 'N', 'D', '_', 'A', 'X', ' ', // A
    'B', 'A', 'N', 'D', '_', 'B', 'X', ' ', // B
    'B', 'A', 'N', 'D', '_', 'E', 'X', ' ', // E
    'F', 'A', 'T', 'S', 'H', 'A', 'R', 'K', // F
    'R', 'A', 'C', 'E', '_', 'X', ' ', ' ', // R
};

const uint8_t bandLetter[5] = {'A', 'B', 'E', 'F', 'R'};

uint16_t channelFreqTable[FREQ_TABLE_SIZE] = {
    5675, 5680, 5825, 5805, 5785, 5765, 5745, 5725, // A
    5733, 5752, 5771, 5790, 5809, 5690, 5700, 5710, // B
    5705, 5685, 5715, 5720, 5730, 5735, 5750, 5755, // E
    5740, 5766, 5780, 5800, 5820, 5773, 5775, 5794, // F
    5795, 5695, 5732, 5769, 5806, 5797, 5802, 5823, // R
};

#endif

uint8_t pitMode = 0;

// ====================================================================================
// Module-Scope Variables
// ====================================================================================
static uint32_t nextFlightControllerQueryTime = 0; // Next time to query the FC
static mspState_e mspState = MSP_STATE_GET_VTX_TABLE_SIZE; // Current MSP communication state
static uint8_t eepromWriteRequired = 0; // Flag indicating if an EEPROM write on the FC is needed
static uint8_t checkingIndex = 0; // Current index during VTX table verification

// ====================================================================================
// Private Function Prototypes (Internal Helpers)
// ====================================================================================
static bool verifyBandData(const uint8_t* rxPayload, uint8_t bandIndex);


uint8_t getFreqTableChannels(void)
{
    return CHANNEL_COUNT;
}

uint8_t getFreqTableSize(void)
{
    return sizeof(channelFreqTable)/sizeof(channelFreqTable[0]);
}

uint8_t getFreqTableBands(void)
{
    return getFreqTableSize() / getFreqTableChannels();
}


uint16_t getFreqByIdx(uint8_t idx)
{
    return channelFreqTable[idx];
}

uint8_t channelFreqLabelByIdx(uint8_t idx)
{
    return channelFreqLabel[idx];
}

uint8_t getBandLetterByIdx(uint8_t idx)
{
    return bandLetter[idx];
}

/**
 * @brief Sends a simple MSP request with no payload.
 * @param opCode The MSP command ID.
 */
void mspSendSimpleRequest(uint16_t opCode)
{
    msp_send_reply(opCode, NULL, 0, MSP_V2);
}

/**
 * @brief Requests the FC to write to its EEPROM. Only sends if required.
 */
void sendEepromWrite()
{
    if (eepromWriteRequired) {
        mspSendSimpleRequest(MSP_EEPROM_WRITE);
    }
    eepromWriteRequired = 0;
    mspState = MONITORING;
    DEBUG_PRINTF("mspState:%s", mspStaetString[(uint8_t)mspState]);
}

/**
 * @brief Sets the VTX table information for a specific band on the FC.
 * @param band The index of the band to set (1-based).
 */
void setVtxTableBand(uint8_t band)
{
    uint8_t payload[29];
    uint8_t band_idx = band - 1;

    payload[0] = band; // Band index (1-based)
    payload[1] = 8;     // BAND_NAME_LENGTH;

    for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
        payload[2 + i] = channelFreqLabelByIdx(band_idx * CHANNEL_COUNT + i);
    }

    payload[2 + CHANNEL_COUNT] = getBandLetterByIdx(band_idx);
    payload[3 + CHANNEL_COUNT] = IS_FACTORY_BAND;
    payload[4 + CHANNEL_COUNT] = CHANNEL_COUNT;

    for(int i = 0; i < CHANNEL_COUNT; i++) {
        uint16_t freq = channelFreqTable[ band_idx * CHANNEL_COUNT + i ];
        payload[(5 + CHANNEL_COUNT) + (i * 2)] = freq & 0xFF;
        payload[(6 + CHANNEL_COUNT) + (i * 2)] = (freq >> 8) & 0xFF;
    }

    msp_send_reply(MSP_SET_VTXTABLE_BAND, payload, sizeof(payload), MSP_V2);
    eepromWriteRequired = 1;
}

/**
 * @brief Queries the FC for VTX table information of a specific band.
 * @param idx The index of the band to query (1-based).
 */
void queryVtxTableBand(uint8_t idx)
{
    msp_send_reply(MSP_VTXTABLE_BAND, &idx, 1, MSP_V2);
}

/**
 * @brief Sets the VTX table information for a specific power level on the FC.
 * @param idx The index of the power level to set (1-based).
 */
void setVtxTablePowerLevel(uint8_t idx)
{
    uint8_t payload[7];
    uint8_t power_level_idx = idx - 1;
    uint16_t powerValue = saPowerLevelsLut[power_level_idx];

    payload[0] = idx;
    payload[1] = powerValue & 0xFF;
    payload[2] = (powerValue >> 8) & 0xFF;
    payload[3] = POWER_LEVEL_LABEL_LENGTH;
    memcpy(&payload[4], &saPowerLevelsLabel[power_level_idx * POWER_LEVEL_LABEL_LENGTH], POWER_LEVEL_LABEL_LENGTH);

    msp_send_reply(MSP_SET_VTXTABLE_POWERLEVEL, payload, sizeof(payload), MSP_V2);
    eepromWriteRequired = 1;
}

/**
 * @brief Queries the FC for VTX table information of a specific power level.
 * @param idx The index of the power level to query (0-based).
 */
void queryVtxTablePowerLevel(uint8_t idx)
{
    uint8_t query_idx = idx + 1; // Power level is 1-based in MSP.
    msp_send_reply(MSP_VTXTABLE_POWERLEVEL, &query_idx, 1, MSP_V2);
}

/**
 * @brief Sets the default band/channel/power on the FC.
 */
void setDefaultBandChannelPower()
{
    uint8_t payload[4];
    payload[0] = VTX_DEFAULT_BAND_CHAN_INDEX & 0xFF;
    payload[1] = (VTX_DEFAULT_BAND_CHAN_INDEX >> 8) & 0xFF;
    payload[2] = VTX_DEFAULT_POWER_INDEX;
    payload[3] = VTX_DEFAULT_PITMODE_STATE;

    msp_send_reply(MSP_SET_VTX_CONFIG, payload, sizeof(payload), MSP_V2);
    eepromWriteRequired = 1;
}

/**
 * @brief Clears the VTX table on the FC and sets a new table size.
 */
void clearVtxTable(void)
{
    uint8_t payload[15];
    uint16_t freq = 0; // Not used when clearing

    // Set some default operational values
    payload[0] = VTX_DEFAULT_BAND_CHAN_INDEX & 0xFF;
    payload[1] = (VTX_DEFAULT_BAND_CHAN_INDEX >> 8) & 0xFF;
    payload[2] = VTX_DEFAULT_POWER_INDEX;
    payload[3] = VTX_DEFAULT_PITMODE_STATE;
    payload[4] = VTX_DEFAULT_LP_DISARM_STATE;
    payload[5] = freq & 0xFF;
    payload[6] = (freq >> 8) & 0xFF;
    // Set new table structure
    payload[7] = 1; // newBand
    payload[8] = 1; // newChannel
    payload[9] = freq & 0xFF;
    payload[10] = (freq >> 8) & 0xFF;
    payload[11] = VTX_TABLE_NEW_BAND_COUNT;
    payload[12] = CHANNEL_COUNT;
    payload[13] = VTX_TABLE_NEW_POWER_COUNT;
    payload[14] = VTX_TABLE_SHOULD_BE_CLEARED;

    msp_send_reply(MSP_SET_VTX_CONFIG, payload, sizeof(payload), MSP_V2);
    eepromWriteRequired = 1;
}

/**
 * @brief Handles the response for MSP_VTX_CONFIG.
 */
void mspvtx_VtxConfig(uint8_t *packet)
{
    mspVtxConfigStruct *vtxconfig = (void*)packet;

    uint8_t powerIndex = vtxconfig->power > 0 ? vtxconfig->power - 1 : 0;
    uint8_t channelIndex = ((vtxconfig->band - 1) * 8) + (vtxconfig->channel - 1);

    if (mspState == MSP_STATE_GET_VTX_TABLE_SIZE)
    {
        // Temporarily store initial settings.
        pitMode = vtxconfig->pitmode;
        if (vtxconfig->lowPowerDisarm) {
            vtxconfig->power = 0;
        }
        setting()->powerIndex = powerIndex;
        setting()->channel = channelIndex;

        // Check if the FC's VTX table size matches OpenVTx's definition.
        if (vtxconfig->bands == getFreqTableBands() &&
            vtxconfig->channels == getFreqTableChannels() &&
            vtxconfig->powerLevels == SA_NUM_POWER_LEVELS)
        {
            // If sizes match, proceed to check the content.
            mspState = CHECK_POWER_LEVELS;
            DEBUG_PRINTF("mspState:%s", mspStaetString[(uint8_t)mspState]);
            nextFlightControllerQueryTime = HAL_GetTick();
        } else {
            // If sizes mismatch, clear and reconfigure the FC's table.
            clearVtxTable();
        }
    }
    else if (mspState == MONITORING)
    {
        // Received a settings change from the FC during normal operation.
        pitMode = vtxconfig->pitmode;

        // Betaflight power levels are 1-based, adjust for 0-based array.
        uint8_t powerIndex = vtxconfig->power > 0 ? vtxconfig->power - 1 : 0;
        
        // Set power before changing frequency to avoid interference on other frequencies.
        uint8_t channelIndex = ((vtxconfig->band - 1) * 8) + (vtxconfig->channel - 1);
        if (channelIndex < getFreqTableSize()) {
            setting()->powerIndex = powerIndex;
            setting()->channel = channelIndex;
            setVtx(channelFreqTable[channelIndex], saPowerLevelsLut[powerIndex]);
        }
    }
}


/**
 * @brief Handles the response for MSP_VTXTABLE_POWERLEVEL.
 */
void mspvtx_VtxTablePowerLevel(uint8_t *packet)
{
    if (mspState != CHECK_POWER_LEVELS) return;

    // Get payload data from the FC's response.
    uint16_t fc_powerValue = ((uint16_t)packet[2] << 8) | packet[1];
    uint8_t fc_labelLength = packet[3];
    const char* fc_label = (const char*)&packet[4];

    // Get OpenVTx's definition for the same index.
    uint16_t ovtx_powerValue = saPowerLevelsLut[checkingIndex];
    const char* ovtx_label = (char*)&saPowerLevelsLabel[checkingIndex * POWER_LEVEL_LABEL_LENGTH];

    // Check if the value and label match.
    if (fc_powerValue == ovtx_powerValue &&
        fc_labelLength == POWER_LEVEL_LABEL_LENGTH &&
        memcmp(fc_label, ovtx_label, POWER_LEVEL_LABEL_LENGTH) == 0)
    {
        // If match, move to the next index.
        checkingIndex++;
        if (checkingIndex >= SA_NUM_POWER_LEVELS) {
            checkingIndex = 0;
            mspState = CHECK_BANDS; // All power levels checked, move to bands.
            DEBUG_PRINTF("mspState:%s", mspStaetString[(uint8_t)mspState]);
        }
        nextFlightControllerQueryTime = HAL_GetTick();
    } else {
        // If mismatch, send the correct value to the FC.
        setVtxTablePowerLevel(checkingIndex + 1);
    }
}

/**
 * @brief Handles the response for MSP_VTXTABLE_BAND.
 */
void mspvtx_VtxTableBand(uint8_t *packet)
{
    if (mspState != CHECK_BANDS) return;

    // Verify if the band data is correct.
    if (verifyBandData(packet, checkingIndex))
    {
        // If correct, move to the next band.
        checkingIndex++;
        if (checkingIndex >= getFreqTableBands()) {
            // All bands have been checked.
            if (eepromWriteRequired) {
                mspState = SET_DEFAULTS; // If table was modified, set defaults.
                DEBUG_PRINTF("mspState:%s", mspStaetString[(uint8_t)mspState]);
            } else {
                mspState = MONITORING; // Otherwise, move to monitoring mode.
                DEBUG_PRINTF("mspState:%s", mspStaetString[(uint8_t)mspState]);
            }
        }
        nextFlightControllerQueryTime = HAL_GetTick();
    } else {
        // If data is incorrect, send the correct values to the FC.
        setVtxTableBand(checkingIndex + 1);
    }
}

/**
 * @brief Verifies if the band data received from the FC matches OpenVTx's definition.
 * @param rxPayload Pointer to the received MSP payload.
 * @param bandIndex The index of the band to verify (0-based).
 * @return true if data matches, false otherwise.
 */
static bool verifyBandData(const uint8_t* rxPayload, uint8_t bandIndex)
{
    uint8_t bandNameLength = rxPayload[1];
    if (bandNameLength != 8) return false;       // BAND_NAME_LENGTH

    // Verify metadata (band letter, channel count, etc.).
    if (rxPayload[2 + bandNameLength] != getBandLetterByIdx(bandIndex) ||
        rxPayload[3 + bandNameLength] != IS_FACTORY_BAND ||
        rxPayload[4 + bandNameLength] != CHANNEL_COUNT) {
        return false;
    }

    // Verify channel labels.
    for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
        if (rxPayload[2 + i] != channelFreqLabelByIdx(bandIndex * CHANNEL_COUNT + i)) {
            return false;
        }
    }

    // Verify frequency values.
    const uint8_t* freqPayload = &rxPayload[5 + bandNameLength];
    for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
        uint16_t value = ((uint16_t)freqPayload[1 + (2*i)] << 8) | freqPayload[0 + (2*i)];
        if (value != channelFreqTable[ bandIndex * CHANNEL_COUNT + i]) {
            return false;
        }
    }
    return true;
}


/**
 * @brief Handles acknowledgement responses for SET and EEPROM_WRITE commands.
 * @param function The command ID that was acknowledged.
 */
void mspvtx_Ack(uint16_t function)
{
    if (function == MSP_SET_VTX_CONFIG && mspState == SET_DEFAULTS) {
        mspState = SEND_EEPROM_WRITE;
        DEBUG_PRINTF("mspState:%s", mspStaetString[(uint8_t)mspState]);
    }
    else if (function == MSP_EEPROM_WRITE) {
        mspState = MONITORING;
        DEBUG_PRINTF("mspState:%s", mspStaetString[(uint8_t)mspState]);
    }
    nextFlightControllerQueryTime = HAL_GetTick();
}


/**
 * @brief Main update loop for MSP communication.
 *
 * This function should be called periodically. It handles serial data processing
 * and sends queries to the FC based on the current state.
 * @param now The current time from millis().
 */
void mspUpdate(void)
{
    static bool initFreqPacketRecived = false;
    static uint16_t logcount = 0;

    uint32_t now = HAL_GetTick();

    if ( now - nextFlightControllerQueryTime < FC_QUERY_PERIOD_MS ) {
        return; // Do nothing if the query interval has not been reached.
    }
    // Update the next query time (acts as a timeout if no reply is received).
    nextFlightControllerQueryTime = now;

    if ( logcount % 10 == 0){
        DEBUG_PRINTF("mspState:%s", mspStaetString[(uint8_t)mspState]);
    }
    logcount = (logcount +1 ) % 10;

    switch (mspState)
    {
        case MSP_STATE_GET_VTX_TABLE_SIZE:
            mspSendSimpleRequest(MSP_VTX_CONFIG);
            break;

        case CHECK_POWER_LEVELS:
            queryVtxTablePowerLevel(checkingIndex);
            break;

        case CHECK_BANDS:
            queryVtxTableBand(checkingIndex + 1);
            break;

        case SET_DEFAULTS:
            setDefaultBandChannelPower();
            // Apply default settings to the VTX immediately, in sync with the FC.
            initFreqPacketRecived = true;
            pitMode = VTX_DEFAULT_PITMODE_STATE;
            setting()->powerIndex = VTX_DEFAULT_POWER_INDEX - 1;
            setting()->channel = VTX_DEFAULT_BAND_CHAN_INDEX;
            setVtx(channelFreqTable[setting()->channel], saPowerLevelsLut[setting()->powerIndex]);
            break;

        case SEND_EEPROM_WRITE:
            sendEepromWrite();
            break;

        case MONITORING:
            // This ensures the VTX operates with EEPROM values until the first
            // valid settings are received from the FC at startup.
            if (!initFreqPacketRecived) {
                initFreqPacketRecived = true;
                setVtx(channelFreqTable[setting()->channel], saPowerLevelsLut[setting()->powerIndex]);
            }
            break;
            
        default:
            // Undefined state.
            break;
    }
}

void mspVtx_init(void)
{
    mspState = MSP_STATE_GET_VTX_TABLE_SIZE;
    DEBUG_PRINTF("call setVtx()");
    setVtx(channelFreqTable[setting()->channel], saPowerLevelsLut[setting()->powerIndex]);
}

#endif
