#include "hwt605.h"

static uint32_t s_uiWitDataCnt = 0, s_uiProtoclo = 0;
static uint8_t s_ucAddr = 0xff;


int32_t WitInit(uint32_t uiProtocol, uint8_t ucAddr)
{
	if(uiProtocol > WIT_PROTOCOL_I2C)
	{
		return WIT_HAL_INVAL;
	}
    s_uiProtoclo = uiProtocol;
    s_ucAddr = ucAddr;
    s_uiWitDataCnt = 0;
    return WIT_HAL_OK;
}


int32_t WitSerialWriteRegister(SerialWrite Write_func)
{
    if(!Write_func)return WIT_HAL_INVAL;
    p_WitSerialWriteFunc = Write_func;
    return WIT_HAL_OK;
}



