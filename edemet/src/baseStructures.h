
#ifndef BASESTRUCTURES_H    
#define BASESTRUCTURES_H
#include "stdint.h"
#include "stdbool.h"
#include "FreeRTOS.h"

#define MAX_MAGNETIC_FIELD 3000
#define NUM_EM 30
#define MAX_SAMPLES_PER_EM 900000
#define BITS_CFL_POWER 10
#define BITS_MAGNETIC_FIELD 13
#define BITS_TEMPERATURE 11
#define BITS_POWER 11

#define EM_VECTOR_SIZE MAX_SAMPLES_PER_EM*BITS_MAGNETIC_FIELD/8 // size in bytes, not actually the number of samples
#define EM_MEASURE_VECTOR_SIZE MAX_SAMPLES_PER_EM*BITS_MAGNETIC_FIELD/8
#define EM_TEMP_VECTOR_SIZE MAX_SAMPLES_PER_EM*BITS_TEMPERATURE/8
#define EM_POWER_VECTOR_SIZE MAX_SAMPLES_PER_EM*BITS_POWER/8
#define CFL_POWER_VECTOR_SIZE MAX_SAMPLES_PER_EM*BITS_CFL_POWER/8 

// Command codes
#define CMD_CODE_CONFIG         0x0001
#define CMD_CODE_VECTOR_HEADER  0x0002
#define CMD_CODE_READ_STATE     0x0003
#define CMD_CODE_PLAY           0x0009
#define CMD_CODE_PAUSE          0x000A
#define CMD_CODE_STOP           0x000B
#define CMD_CODE_DISCOVER       0x000C
#define CMD_CODE_WRAPPER        0xF1F2

// Response codes
#define RESPONSE_SUCCESS                      0x00
#define RESPONSE_ERROR_WRONG_IDX_WRAPPER_PKT  0x01
#define RESPONSE_ERROR_INVALID_RING           0x02
#define RESPONSE_ERROR_INVALID_EM             0x03
#define RESPONSE_ERROR_INVALID_MODE           0x04
#define RESPONSE_ERROR_VECTOR_TOO_LARGE       0x05
#define RESPONSE_ERROR_ALLOCATION_FAILED      0x06


#define OPERATION_CONTROL_INIT {    \
    .outputsStatus = false,         \
    .operationLed = false,          \
    .warningLed = false,            \
    .currentState = STOP_STATE,     \
    .operationTime = 0,             \
    .delayTime = 0,                 \
    .currentTimeTick = 0,           \
    .relativeTimeTick = 0,          \
    .currentStage = STAGE_UNDEFINED,\
    .currentStatus = STATUS_STOP,   \
    .cmd = 0,                       \
    .signalSamplePeriodMs = 1000    \
}


int16_t get10s(const uint8_t *buf, uint32_t index);
static inline void put10s(uint8_t *buf, uint32_t index, int16_t value);
static inline void put11s(uint8_t *buf, uint32_t index, int16_t value);
int16_t get11s(const uint8_t *buf, uint32_t index);


typedef enum
{
    IDLE_STATE = 0,
    PLAY_STATE,
    PAUSE_STATE,
    STOP_STATE,
    DONE_STATE
} function_state_t;

typedef enum {
    STATUS_STOP = 0,
    STATUS_PLAY,
    STATUS_PAUSE
} operation_status_t;

typedef enum {
    CMD_STOP = 0,
    CMD_PLAY,
    CMD_PAUSE,
    CMD_NONE,
} operation_cmd_t;

typedef enum {    
    STAGE_DELAY,
    STAGE_OPERATION,
    STAGE_UNDEFINED    
} operation_stage_t;

typedef struct {
    // Sent in one config message
    uint8_t ring_idx;
    uint8_t em_idx;
    uint8_t mode; // 0: file, 1: parametrized
    uint8_t type; // 0: constant, 1: square, 2: sinusoidal, 3: sawtooth
    uint8_t duty_cycle; // 0~100
    uint8_t frequency_factor; // 0~100. frequency_factor x 0.1 = 0.0~10.0 
    int16_t magnetic_field_offset;
    int16_t magnetic_field_amplitude; //-3000 a 3000
    uint8_t static_mode; // 0: static, 1: rotational
    uint8_t rotational_frequency_factor; //0~100. frequency_factor x 0.1 = 0.0~10.0 
    uint8_t coils_columns;
    // Send in another config message
    uint16_t sample_period; // in ms
    uint32_t vector_length;
    uint32_t playbackIndex;
    int16_t last_em_ctrl;
    int16_t last_em_measure;
    uint16_t last_em_pwr;
    int16_t last_em_temp;
    uint8_t *em_ctrl; // fixed size, initialized on main
    uint8_t *em_measure; // fixed size, initialized on main
    uint8_t *em_pwr; // fixed size, initialized on main
    uint8_t *em_temp; // fixed size, initialized on main
} em_t;

typedef struct {    
    bool outputsStatus;
    bool operationLed;
    bool warningLed;    
    uint32_t operationTime;             // Total operation time (ms, ticks, etc.)
    uint32_t delayTime;                 // Delay time between stages
    uint32_t relativeTimeTick;          // Current elapsed time
    uint32_t currentTimeTick;
    uint16_t signalSamplePeriodMs;      // Period of reproduction of the signal    
    uint32_t generalPlaybackIndex;
    bool availableEms[NUM_EM];
    uint8_t *cfle_pwr;
    uint16_t last_cfle_pwr;
    function_state_t currentState; 
    operation_stage_t currentStage;
    operation_status_t currentStatus;
    operation_cmd_t cmd;       
    SemaphoreHandle_t mutex; 
    em_t em[30];
} operation_control_t;

// typedef struct {
//     operation_control_t *op; // Shared status
//     uint32_t baseAddr;       // Hardware-specific address
//     bool *prevTaskReady;
// } io_expander_cfg_t;

// typedef struct {
//     operation_control_t *op; // Shared status
//     uint32_t baseAddr;       // Hardware-specific address
//     bool *prevTaskReady;
// } i2c_multiplexer_t;

typedef struct {
    operation_control_t *op; // Shared status
    uint32_t baseAddr;       // Hardware-specific address
    SemaphoreHandle_t waitSem;   // Wait for this to start
    SemaphoreHandle_t signalSem; // Give this when done
} task_manager_t;

#endif
