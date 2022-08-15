#include "flash.h"


#define FSL_SUPPORT_ERASE_SECTOR_NON_BLOCKING 1U


/*! @brief Flash driver Structure */
//static flash_config_t s_flashDriver;
///*! @brief Flash cache driver Structure */
//static ftfx_cache_config_t s_cacheDriver;



void init_flash(flash_config_t *s_flashDriver)
{
	ftfx_security_state_t securityStatus = kFTFx_SecurityStateNotSecure; /* Return protection status */
	status_t result;   /* Return code from each flash driver function */
	uint32_t destAddr; /* Address of the target location */
	uint32_t i, failAddr, failDat;

	uint32_t pflashBlockBase  = 0;
	uint32_t pflashTotalSize  = 0;
	uint32_t pflashSectorSize = 0;

    /* Clean up Flash, Cache driver Structure*/
    memset(s_flashDriver, 0, sizeof(flash_config_t));
//    memset(&s_cacheDriver, 0, sizeof(ftfx_cache_config_t));

    result = FLASH_Init(s_flashDriver);
    if (kStatus_FTFx_Success != result)
    {
        error_trap();
    }
//    /* Setup flash cache driver structure for device and initialize variables. */
//    result = FTFx_CACHE_Init(&s_cacheDriver);
//    if (kStatus_FTFx_Success != result)
//    {
//        error_trap();
//    }
    /* Get flash properties*/
    FLASH_GetProperty(s_flashDriver, kFLASH_PropertyPflash0BlockBaseAddr, &pflashBlockBase);
    FLASH_GetProperty(s_flashDriver, kFLASH_PropertyPflash0TotalSize, &pflashTotalSize);
    FLASH_GetProperty(s_flashDriver, kFLASH_PropertyPflash0SectorSize, &pflashSectorSize);

    /* Print flash information - PFlash. */
	#ifdef DEBUG_PRINT_ENABLED
		PRINTF("\r\n PFlash Information: ");
		PRINTF("\r\n Total Program Flash Size:\t%d KB, Hex: (0x%x)", (pflashTotalSize / 1024), pflashTotalSize);
	#endif
    result = FLASH_GetSecurityState(s_flashDriver, &securityStatus);
    if (kStatus_FTFx_Success != result)
    {
        error_trap();
    }

    //flash security status checked ignored

//	#ifndef SECTOR_INDEX_FROM_END
//	#define SECTOR_INDEX_FROM_END 1U
//	#endif
//
//	#ifdef TEST_TARGET_ADDRESS
//		destAddr = TEST_TARGET_ADDRESS;
//	#else
//		/* Erase a sector from destAddr. */
//	#if defined(FSL_FEATURE_FLASH_HAS_PFLASH_BLOCK_SWAP) && FSL_FEATURE_FLASH_HAS_PFLASH_BLOCK_SWAP
//		/* Note: we should make sure that the sector shouldn't be swap indicator sector*/
//		destAddr = pflashBlockBase + (pflashTotalSize - (SECTOR_INDEX_FROM_END * pflashSectorSize * 2));
//	#else
//		destAddr = pflashBlockBase + (pflashTotalSize - (SECTOR_INDEX_FROM_END * pflashSectorSize));
//	#endif
//	#endif
//
//	return destAddr;

}

/*
 * @brief Gets called when an error occurs.
 *
 * @details Print error message and trap forever.
 */
void error_trap(void)
{
//    PRINTF("\r\n\r\n\r\n\t---- HALTED DUE TO FLASH ERROR! ----");
    while (1)
    {
    }
}

/*
 *
 */
void clear_flash(flash_config_t *s_flashDriver, uint32_t base_address)
{

	status_t result;
	/*
	 *
	 * (end_address - base_address) / 4096 = # of blocks to clear
	 * (0x200000 - 0xFF000) / 0x1000 = 257 blocks
	 * (0x140000 - 0xFF000) / 0x1000 = (266240/4096) 65
	 * 0x100000-0xFF000
	 */
	result   = FLASH_Erase(s_flashDriver, base_address, 4096 * 1, kFTFx_ApiEraseKey);

	if (kStatus_FTFx_Success != result)
	{
		error_trap();
	}

	/* Verify sector if it's been erased. */
	result = FLASH_VerifyErase(s_flashDriver, base_address, 4096 *1, kFTFx_MarginValueUser);
	if (kStatus_FTFx_Success != result)
	{
		error_trap();
	}
	/* Print message for user. */
	//PRINTF("\r\n Successfully Erased Sector 0x%x -> 0x%x\r\n", base_address, (base_address + 4096 *257));


}

void write_data(flash_config_t *s_flashDriver, uint32_t write_base_address, uint64_t data_counter, sensor_data_t *data)
{
	status_t result;
	// result = FLASH_Program(&s_flashDriver, destAddr + (sizeof(data_structure_t)* index), (uint8_t *)&dummy_array[index], sizeof(data_structure_t));

	//write_data(s_flashDriver, data_write_base + (sizeof(sensor_data_t)* data_counter), g_sensor_data_handle);
	//data size optimisation
	typedef struct {
		int16_t gyro_x;
		int16_t gyro_y;
		int16_t gyro_z;
		float accel_x;
		float accel_y;
		float accel_z;
		uint8_t state;
		int16_t padding;
	} o_flash_data_t;
	o_flash_data_t temp;

	//multiples of 8
	temp.gyro_x = (int16_t)data->gyro_x;
	temp.gyro_y = (int16_t)data->gyro_y;
	temp.gyro_z = (int16_t)data->gyro_z;
	temp.accel_x = (float)data->accel_x;
	temp.accel_y = (float)data->accel_y;
	temp.accel_z = (float)data->accel_z;
	temp.state = (uint8_t)data->state;
	temp.padding = 255;

//	PRINTF("\r\n%d %d %d", temp.gyro_x, temp.gyro_y, temp.gyro_z);

	result = FLASH_Program(s_flashDriver,  write_base_address + (sizeof(o_flash_data_t)* data_counter), (uint8_t *)&temp, sizeof(o_flash_data_t));

    if (kStatus_FTFx_Success != result)
    {
        error_trap();
    }

}

void update_data_counter(flash_config_t *s_flashDriver, uint32_t write_base_address, uint64_t data_counter)
{

	status_t result;
	int temp_count = (int)data_counter;

	typedef struct metadata{
		int data_counter;
		int padding;
	} metadata_t;

	metadata_t meta;
	meta.data_counter =  (int)data_counter;
	meta.padding = -255;

	result   = FLASH_Erase(s_flashDriver, write_base_address, 4096, kFTFx_ApiEraseKey);

	if (kStatus_FTFx_Success != result)
	{
		error_trap();
	}


	result = FLASH_Program(s_flashDriver,  write_base_address, (uint8_t *)&meta, sizeof(metadata_t));

	if (kStatus_FTFx_Success != result)
	{
		error_trap();
	}


}


//void ClearBlockFlash(uint32_t location)
//{
//	/* Mandatory Variables */
//	ftfx_security_state_t securityStatus = kFTFx_SecurityStateNotSecure; /* Return protection status */
//	status_t result;   /* Return code from each flash driver function */
//	uint32_t destAddr; /* Address of the target location */
//	uint32_t i, failAddr, failDat;
//
//
//#if defined(FSL_SUPPORT_ERASE_SECTOR_NON_BLOCKING) && FSL_SUPPORT_ERASE_SECTOR_NON_BLOCKING
//	result = FLASH_EraseSectorNonBlocking(&s_flashDriver, destAddr, kFTFx_ApiEraseKey);
//	if (kStatus_FTFx_Success != result)
//	{
//		error_trap();
//	}
//	/* Before programming the flash, check whether the erase sector command is completed,*/
//	/* and get the flash status. */
//	result = FLASH_GetCommandState();
//	if (kStatus_FTFx_Success != result)
//	{
//		error_trap();
//	}
//#else
//	result   = FLASH_Erase(&s_flashDriver, destAddr, pflashSectorSize, kFTFx_ApiEraseKey);
//	if (kStatus_FTFx_Success != result)
//	{
//		error_trap();
//	}
//#endif
//	/* Verify sector if it's been erased. */
//	result = FLASH_VerifyErase(&s_flashDriver, destAddr, pflashSectorSize, kFTFx_MarginValueUser);
//	if (kStatus_FTFx_Success != result)
//	{
//		error_trap();
//	}
//	/* Print message for user. */
//	PRINTF("\r\n Successfully Erased Sector 0x%x -> 0x%x\r\n", destAddr, (destAddr + pflashSectorSize));
//
//
//}

//void write_data(uint32_t location)
//{
//
//	ftfx_security_state_t securityStatus = kFTFx_SecurityStateNotSecure; /* Return protection status */
//	status_t result;   /* Return code from each flash driver function */
//	uint32_t destAddr; /* Address of the target location */
//	uint32_t i, failAddr, failDat;
//
//	uint32_t pflashBlockBase  = 0;
//	uint32_t pflashTotalSize  = 0;
//	uint32_t pflashSectorSize = 0;
//
//	/* Clean up Flash, Cache driver Structure*/
//	memset(&s_flashDriver, 0, sizeof(flash_config_t));
//	memset(&s_cacheDriver, 0, sizeof(ftfx_cache_config_t));
//
//	/* Setup flash driver structure for device and initialize variables. */
//	result = FLASH_Init(&s_flashDriver);
//	if (kStatus_FTFx_Success != result)
//	{
//		error_trap();
//	}
//	/* Setup flash cache driver structure for device and initialize variables. */
//	result = FTFx_CACHE_Init(&s_cacheDriver);
//	if (kStatus_FTFx_Success != result)
//	{
//		error_trap();
//	}
//	/* Get flash properties*/
//	FLASH_GetProperty(&s_flashDriver, kFLASH_PropertyPflash0BlockBaseAddr, &pflashBlockBase);
//	FLASH_GetProperty(&s_flashDriver, kFLASH_PropertyPflash0TotalSize, &pflashTotalSize);
//	FLASH_GetProperty(&s_flashDriver, kFLASH_PropertyPflash0SectorSize, &pflashSectorSize);
//
//
//#ifndef SECTOR_INDEX_FROM_END
//#define SECTOR_INDEX_FROM_END 1U
//#endif
//
//#ifdef TEST_TARGET_ADDRESS
//	destAddr = TEST_TARGET_ADDRESS;
//#else
//	/* Erase a sector from destAddr. */
//#if defined(FSL_FEATURE_FLASH_HAS_PFLASH_BLOCK_SWAP) && FSL_FEATURE_FLASH_HAS_PFLASH_BLOCK_SWAP
//	/* Note: we should make sure that the sector shouldn't be swap indicator sector*/
//	destAddr = pflashBlockBase + (pflashTotalSize - (SECTOR_INDEX_FROM_END * pflashSectorSize * 2));
//	destAddr = location;
//#else
//	destAddr = pflashBlockBase + (pflashTotalSize - (SECTOR_INDEX_FROM_END * pflashSectorSize));
//#endif
//#endif
//
//
//		/* Program user buffer into flash*/
//
//		for (int index = 0; index < BUFFER_LEN ; index++)
//		{
//			result = FLASH_Program(&s_flashDriver, (destAddr + (sizeof(acc_structure_t)* index)), (uint8_t *)&str_ptr, sizeof(acc_structure_t));
//			if (kStatus_FTFx_Success != result)
//			{
//				error_trap();
//			}
//			else
//			{
//
//			}
//		}
//
//
//
//	FTFx_CACHE_ClearCachePrefetchSpeculation(&s_cacheDriver, false);
//
//#if defined(FSL_FEATURE_HAS_L1CACHE) && FSL_FEATURE_HAS_L1CACHE
//	L1CACHE_InvalidateCodeCache();
//#endif /* FSL_FEATURE_HAS_L1CACHE */
//
//#if defined(__DCACHE_PRESENT) && __DCACHE_PRESENT
//	/* Clean the D-Cache before reading the flash data*/
//	SCB_CleanInvalidateDCache();
//#endif
//
//
//	printf("Written %d values to flash\n", BUFFER_LEN * 3);
//}
//
//void error_trap(void)
//{
//	PRINTF("\r\n\r\n\r\n\t---- HALTED DUE TO FLASH ERROR! ----");
//	while (1)
//	{
//	}
//}
//

