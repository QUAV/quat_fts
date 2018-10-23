
#include "board.h"
#include "third_party/canard_stm32.h"
#include "third_party/canard.h"
#include <machine/hal.h>
#include <support/stm32f1/cpu.h>
#include <support/stm32f1/gpio.h>
#include <support/gpio_hal.h>
#include <support/i2c_hal.h>
#include <support/pwm_hal.h>


#define UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE                    0xee468a8121c46a9e
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_ID                           1
#define UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE                      ((3015 + 7) / 8)

#define UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID                          1030
#define UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE                   0x217f5c87d7ec951d
#define UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_MAX_VALUE                   8192

#define UAVCAN_NODE_STATUS_MESSAGE_SIZE                             7
#define UAVCAN_NODE_STATUS_DATA_TYPE_ID                             341
#define UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE                      0x0f0868d0c1a7c6f1

#define UAVCAN_PROTOCOL_PARAM_GETSET_ID                             11
#define UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE                      0xa7b622f939d1a4d5 

#define UAVCAN_NODE_HEALTH_OK                                       0
#define UAVCAN_NODE_HEALTH_WARNING                                  1
#define UAVCAN_NODE_HEALTH_ERROR                                    2
#define UAVCAN_NODE_HEALTH_CRITICAL                                 3

#define UAVCAN_NODE_MODE_OPERATIONAL                                0
#define UAVCAN_NODE_MODE_INITIALIZATION                             1

#define UNIQUE_ID_LENGTH_BYTES                                      16



#define APP_VERSION_MAJOR                       99
#define APP_VERSION_MINOR                       99
#define APP_NODE_NAME                           "Quaternium FTS"
#define GIT_HASH                                0xBABEBABE            // Normally this should be queried from the VCS when building the firmware
#define UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE  ((3015 + 7) / 8)


typedef struct
{
uint8_t* name;
int64_t val; 
int64_t min;
int64_t max;
int64_t defval;
} param_t;

static param_t parameters[] = 
{
  {"param0", 0, 10,20, 15},
  {"param1", 1, 0, 100, 25},
  {"param2", 2, 2, 8,  3 },
};

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))


static CanardInstance g_canard;             // The library instance
static uint8_t g_canard_memory_pool[1024]; 	// Arena for memory allocation, used by the library


// HACK TEST DELETE DELETE 

static unsigned int getUptime ()
{
	static unsigned int t = 0;
	t += 2;
	return t;
}

static inline param_t * getParamByName(uint8_t * name)
{
  for(uint16_t i = 0; i < ARRAY_SIZE(parameters); i++)
  {
    if(strncmp((char const*)name, (char const*)parameters[i].name,strlen((char const*)parameters[i].name)) == 0) 
    {
      return &parameters[i];
    }
  }      
  return NULL;
}


static inline param_t * getParamByIndex(uint16_t index)
{
  if(index >= ARRAY_SIZE(parameters)) 
  {
    return NULL;
  }

  return &parameters[index];
}


static uint16_t encodeParamCanard(param_t * p, uint8_t * buffer)
{
    uint8_t n     = 0;
    int offset    = 0;
    uint8_t tag   = 1;
    if(p==NULL)
    {   
        tag = 0;
        canardEncodeScalar(buffer, offset, 5, &n);
        offset += 5;
        canardEncodeScalar(buffer, offset,3, &tag);
        offset += 3;
        
        canardEncodeScalar(buffer, offset, 6, &n);
        offset += 6;
        canardEncodeScalar(buffer, offset,2, &tag);
        offset += 2;
        
        canardEncodeScalar(buffer, offset, 6, &n);
        offset += 6;
        canardEncodeScalar(buffer, offset, 2, &tag);
        offset += 2;
        buffer[offset / 8] = 0;
        return ( offset / 8 + 1 );
    }
    canardEncodeScalar(buffer, offset, 5,&n);
    offset += 5;
    canardEncodeScalar(buffer, offset, 3, &tag);
    offset += 3;
    canardEncodeScalar(buffer, offset, 64, &p->val);
    offset += 64;
    
    canardEncodeScalar(buffer, offset, 5, &n);
    offset += 5;
    canardEncodeScalar(buffer, offset, 3, &tag);
    offset += 3;
    canardEncodeScalar(buffer, offset, 64, &p->defval);
    offset += 64;
    
    canardEncodeScalar(buffer, offset, 6, &n);
    offset += 6;
    canardEncodeScalar(buffer, offset, 2, &tag);
    offset += 2;
    canardEncodeScalar(buffer, offset, 64, &p->max);
    offset += 64;
    
    canardEncodeScalar(buffer, offset, 6, &n);
    offset += 6;
    canardEncodeScalar(buffer, offset,2,&tag);
    offset += 2;
    canardEncodeScalar(buffer, offset,64,&p->min);
    offset += 64;
    
    memcpy(&buffer[offset / 8], p->name, strlen((char const*)p->name));
    return  (offset/8 + strlen((char const*)p->name)); 
}

static void readUniqueID(uint8_t* out_uid)
{
    for (uint8_t i = 0; i < UNIQUE_ID_LENGTH_BYTES; i++)
    {
        out_uid[i] = i;
    }
}

static bool shouldAcceptTransfer(const CanardInstance* ins,
                          uint64_t* out_data_type_signature,
                          uint16_t data_type_id,
                          CanardTransferType transfer_type,
                          uint8_t source_node_id)
{
    if ((transfer_type == CanardTransferTypeRequest) &&
        (data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID))
    {
        *out_data_type_signature = UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE;
        return true;
    }
  
    return false;
}

static void makeNodeStatusMessage(uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE])
{
    const uint8_t node_health = UAVCAN_NODE_HEALTH_OK;
    const uint8_t node_mode   = UAVCAN_NODE_MODE_OPERATIONAL;
    memset(buffer, 0, UAVCAN_NODE_STATUS_MESSAGE_SIZE);
    const uint32_t uptime_sec = getUptime() / 1000;
    canardEncodeScalar(buffer,  0, 32, &uptime_sec);
    canardEncodeScalar(buffer, 32,  2, &node_health);
    canardEncodeScalar(buffer, 34,  3, &node_mode);
}


static uint16_t makeNodeInfoMessage(uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE])
{
    memset(buffer, 0, UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE);
    makeNodeStatusMessage(buffer);
 
    buffer[7] = APP_VERSION_MAJOR;
    buffer[8] = APP_VERSION_MINOR;
    buffer[9] = 1;  // Optional field flags, VCS commit is set
    const uint32_t git_hash = GIT_HASH;
    canardEncodeScalar(buffer, 80, 32, &git_hash);
 
    readUniqueID(&buffer[24]);
    const size_t name_len = strlen(APP_NODE_NAME);
    memcpy(&buffer[41], APP_NODE_NAME, name_len);
    return 41 + name_len ;
}

static void getNodeInfoHandleCanard(CanardRxTransfer* transfer)
{
    uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE];
    memset(buffer, 0, UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE);
    const uint16_t len = makeNodeInfoMessage(buffer);
    int result = canardRequestOrRespond(&g_canard,
                                        transfer->source_node_id,
                                        UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE,
                                        UAVCAN_GET_NODE_INFO_DATA_TYPE_ID,
                                        &transfer->transfer_id,
                                        transfer->priority,
                                        CanardResponse,
                                        &buffer[0],
                                        (uint16_t)len);
    if (result < 0)
    {
        // TODO: handle the error
    }
}


static void getsetHandleCanard(CanardRxTransfer* transfer)
{
    uint16_t index = 0xFFFF;
    uint8_t tag    = 0;
    int offset     = 0;
    int64_t val    = 0;

    canardDecodeScalar(transfer, offset,  13, false, &index);
    offset += 13;
    canardDecodeScalar(transfer, offset, 3, false, &tag);
    offset += 3;

    if(tag == 1)
    {
        canardDecodeScalar(transfer, offset, 64, false, &val);
        offset += 64;
    } 

    uint16_t n = transfer->payload_len - offset / 8 ;
    uint8_t name[16]      = "";
    for(int i = 0; i < n; i++)
    {
        canardDecodeScalar(transfer, offset, 8, false, &name[i]);
        offset += 8;
    }

    param_t * p = NULL;

    if(strlen((char const*)name))
    {
        p = getParamByName(name);
    }
    else
    {
        p = getParamByIndex(index);
    }

    if((p)&&(tag == 1))
    {
        p->val = val;
    }

    uint8_t  buffer[64] = "";
    uint16_t len = encodeParamCanard(p, buffer);
    int result = canardRequestOrRespond(&g_canard,
                                        transfer->source_node_id,
                                        UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                                        UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                                        &transfer->transfer_id,
                                        transfer->priority,
                                        CanardResponse,
                                        &buffer[0],
                                        (uint16_t)len);
  
}


/*
static void rcpwmUpdate(int16_t ar[6])
{
    for(char i = 0; i < 6; i++ )
    {
        uint32_t val = RCPWM_NEUTRAL_uS;
        ar[i] = ar[i] * RCPWM_MAGNITUDE_uS / UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_MAX_VALUE;
        val += ar[i];
        TIM_SetCompare(rcpwm_outputs[i].timer,rcpwm_outputs[i].channel, uS_to_ticks(val));
    }
}
*/

static void rawcmdHandleCanard(CanardRxTransfer* transfer)
{
    int16_t ar[6] = {0,0,0,0,0,0};
    int offset = 0;
    for(int i = 0; i<6; i++)
    {
        if(canardDecodeScalar(transfer, offset, 14, true, &ar[i])<14) {break;}
        offset += 14;
    }
    //rcpwmUpdate(ar);
}

static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{
    if ((transfer->transfer_type == CanardTransferTypeRequest) &&
        (transfer->data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID))
    {
        getNodeInfoHandleCanard(transfer);
    } 
    
    if(transfer->data_type_id == UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID)
    { 
        rawcmdHandleCanard(transfer);
    }
    
    if(transfer->data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_ID)
    { 
        getsetHandleCanard(transfer);
    }
}

#define CANARD_SPIN_PERIOD    1000
 
static void spinCanard(void)
{
    static uint32_t spin_time = 0;
    if (getUptime() < spin_time + CANARD_SPIN_PERIOD) { return; }  // rate limiting
    spin_time = getUptime();
    //gpioToggle(GPIOE, GPIO_Pin_8);                           // some indication
 
    uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE];
    static uint8_t transfer_id = 0;                          // This variable MUST BE STATIC; refer to the libcanard documentation for the background
 
    makeNodeStatusMessage(buffer);
 
    canardBroadcast(&g_canard,
                    UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE,
                    UAVCAN_NODE_STATUS_DATA_TYPE_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    UAVCAN_NODE_STATUS_MESSAGE_SIZE);
}



static void sendCanard(void)
{
    const CanardCANFrame* txf = canardPeekTxQueue(&g_canard);
    while(txf)
    {
        const int tx_res = canardSTM32Transmit(txf);
        if (tx_res < 0)         // Failure - drop the frame and report
        {
            __ASM volatile("BKPT #01");   // TODO: handle the error properly
        }
        if(tx_res > 0)
        {
            canardPopTxQueue(&g_canard);
        }
        txf = canardPeekTxQueue(&g_canard);
    }
}

static void receiveCanard(void)
{
    CanardCANFrame rx_frame;
    int res = canardSTM32Receive(&rx_frame);
    if(res)
    {
        canardHandleRxFrame(&g_canard, &rx_frame, getUptime() * 1000);
    }   
}


void board_uavcan_init ()
{
	CanardSTM32CANTimings can_timings;

	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
	RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;

	canardSTM32ComputeCANTimings(cpu_get_pclk1(), 1000000, &can_timings);

	int res = canardSTM32Init(&can_timings, CanardSTM32IfaceModeNormal);

	canardInit(&g_canard,                         // Uninitialized library instance
               g_canard_memory_pool,              // Raw memory chunk used for dynamic allocation
               sizeof(g_canard_memory_pool),      // Size of the above, in bytes
               onTransferReceived,                // Callback, see CanardOnTransferReception
               shouldAcceptTransfer,              // Callback, see CanardShouldAcceptTransfer
               NULL); 

    canardSetLocalNodeID(&g_canard, 100); // node ID 100, a saco

    while(1)
    {
        sendCanard();
        receiveCanard();
        spinCanard();
    }

}

