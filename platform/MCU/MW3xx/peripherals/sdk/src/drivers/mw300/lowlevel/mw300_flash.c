/****************************************************************************//**
 * @file     mw300_flash.c
 * @brief    This file provides FLASH functions.
 * @version  V1.0.0
 * @date     12-Aug-2013
 * @author   CE Application Team
 *
 * @note
 * Copyright (C) 2012 Marvell Technology Group Ltd. All rights reserved.
 *
 * @par
 * Marvell is supplying this software which provides customers with programming
 * information regarding the products. Marvell has no responsibility or 
 * liability for the use of the software. Marvell not guarantee the correctness 
 * of this software. Marvell reserves the right to make changes in the software 
 * without notification. 
 * 
 *******************************************************************************/

#include "mw300.h"
#include "mw300_driver.h"
#include "mw300_flash.h"
#include "mw300_qspi.h"

/** @addtogroup  MW300_Periph_Driver
 *  @{
 */

/** @defgroup FLASH FLASH
 *  @brief FLASH driver modules
 *  @{
 */

/** @defgroup FLASH_Private_Type
 *  @{
 */

/*@} end of group FLASH_Private_Type*/

/** @defgroup FLASH_Private_Defines
 *  @{
 */


/*@} end of group FLASH_Private_Defines */

#define	FLASH_DEFAULT_INDEX	1
/** @defgroup FLASH_Private_Variables
 *  @{
 */

const struct flash_device_config fl_dev_list[] = {
	{"W25Q80BL", 0xef4014, 1 * MEGA_BYTE, 4 * KILO_BYTE, 64 * KILO_BYTE,
		256},
	{"W25Q16CL", 0xef4015, 2 * MEGA_BYTE, 4 * KILO_BYTE, 64 * KILO_BYTE,
		256},
	{"W25Q32BV", 0xef4016, 4 * MEGA_BYTE, 4 * KILO_BYTE, 64 * KILO_BYTE,
		256},
	{"W25Q64CV", 0xef4017, 8 * MEGA_BYTE, 4 * KILO_BYTE, 64 * KILO_BYTE,
		256},
	{"W25Q128BV", 0xef4018, 16 * MEGA_BYTE, 4 * KILO_BYTE, 64 * KILO_BYTE,
		256},
	{"GD25Q16B", 0xc84015, 2 * MEGA_BYTE, 4 * KILO_BYTE, 64 * KILO_BYTE,
		256},
	{"GD25Q16C", 0xc84015, 2 * MEGA_BYTE, 4 * KILO_BYTE, 64 * KILO_BYTE,
		256},
	{"GD25Q32C", 0xc84016, 4 * MEGA_BYTE, 4 * KILO_BYTE, 64 * KILO_BYTE,
		256},
	{"GD25Q128C", 0xc84018, 16 * MEGA_BYTE, 4 * KILO_BYTE, 64 * KILO_BYTE,
		256},
	{"MX25L8035E", 0xc22014, 1 * MEGA_BYTE, 4 * KILO_BYTE, 64 * KILO_BYTE,
		256},
	{"MX25L6433F", 0xc22017, 8 * MEGA_BYTE, 4 * KILO_BYTE, 64 * KILO_BYTE,
		256},
	{"MX25V1635F", 0xc22315, 2 * MEGA_BYTE, 4 * KILO_BYTE, 64 * KILO_BYTE,
		256},/*yhb added */
  {"FM25Q16A", 0xa14015, 2 * MEGA_BYTE, 4 * KILO_BYTE, 64 * KILO_BYTE,
    256},/*swyang added */
  {"XT25F16B", 0x0b4015, 2 * MEGA_BYTE, 4 * KILO_BYTE, 64 * KILO_BYTE,
    256},/*swyang added */
};

/*@} end of group FLASH_Private_Variables */

/** @defgroup FLASH_Global_Variables
 *  @{
 */
static const struct flash_device_config *g_fl_cfg =
				&fl_dev_list[FLASH_DEFAULT_INDEX];
static int flash_config_set=0;

#ifdef CONFIG_ENABLE_MXCHIP
void feeddog(void);
#endif

/*@} end of group FLASH_Global_Variables */

/** @defgroup FLASH_Private_FunctionDeclaration
 *  @{
 */


/*@} end of group FLASH_Private_FunctionDeclaration */

/** @defgroup FLASH_Private_Functions
 *  @{
 */


void FLASH_PowerDown(FunctionalState newCmd)
{
  /* Clear QSPI1 FIFO */
  QSPI_FlushFIFO();

  /* Set Header count register: instruction counter, address counter, read mode counter and dummy counter */
  QSPI_SetHdrcnt(QSPI_INSTR_CNT_1BYTE, QSPI_ADDR_CNT_0BYTE, QSPI_RM_CNT_0BYTE, QSPI_DUMMY_CNT_0BYTE);

  /* Set data in counter */
  QSPI_SetDInCnt(0);

  if(newCmd == ENABLE)
  {
    /* Set instruction */
    QSPI_SetInstr(FLASH_INS_CODE_PD);
  }
  else
  {
    /* Set instruction */
    QSPI_SetInstr(FLASH_INS_CODE_RPD_DI);
  }

  /* Set QSPI1 write */
  QSPI_StartTransfer(QSPI_W_EN);

  /* Stop QSPI1 transfer */
  QSPI_StopTransfer();
}


/****************************************************************************//**
 * @brief      Set flash write enable / disable 
 *
 * @param[in]  newCmd:  Enable/disable flash write
 *
 * @return none
 *
 *******************************************************************************/
void FLASH_SetWriteEnableBit(FunctionalState newCmd) 
{
  /* Clear QSPI FIFO */ 
  QSPI_FlushFIFO();
  
  /* Set Header count register: instruction counter, address counter, read mode counter and dummy counter */
  QSPI_SetHdrcnt(QSPI_INSTR_CNT_1BYTE, QSPI_ADDR_CNT_0BYTE, QSPI_RM_CNT_0BYTE, QSPI_DUMMY_CNT_0BYTE);
  
  /* Set data in counter */
  QSPI_SetDInCnt(0);
  
  if(newCmd == ENABLE)
  {
    /* Set instruction */
    QSPI_SetInstr(FLASH_INS_CODE_WE);
  }
  else
  {
    /* Set instruction */
    QSPI_SetInstr(FLASH_INS_CODE_WD);
  }
        
  /* Set QSPI write */
  QSPI_StartTransfer(QSPI_W_EN);
  
  /* Stop QSPI transfer */
  QSPI_StopTransfer();
}

/****************************************************************************//**
 * @brief      Write enable for volatile status register
 *
 * @param none
 *
 * @return none
 *
 *******************************************************************************/
void FLASH_WriteEnableVSR(void)
{
  /* Clear QSPI FIFO */ 
  QSPI_FlushFIFO();
  
  /* Set Header count register: instruction counter, address counter, read mode counter and dummy counter */
  QSPI_SetHdrcnt(QSPI_INSTR_CNT_1BYTE, QSPI_ADDR_CNT_0BYTE, QSPI_RM_CNT_0BYTE, QSPI_DUMMY_CNT_0BYTE);
  
  /* Set data in counter */
  QSPI_SetDInCnt(0);
  
  /* Set instruction */
  QSPI_SetInstr(FLASH_INS_CODE_WE_VSR);
  
  /* Set QSPI write */
  QSPI_StartTransfer(QSPI_W_EN);
  
  /* Stop QSPI transfer */
  QSPI_StopTransfer();  
}

/****************************************************************************//**
 * @brief      Reset Flash Continuous Dual Read mode
 *
 * @param none
 *
 * @return none
 *
 *******************************************************************************/
void FLASH_ResetFastReadDual(void)
{
  /* Clear QSPI FIFO */ 
  QSPI_FlushFIFO();
  
  QSPI->CONF.BF.DATA_PIN = QSPI_DATA_PIN_DUAL; 
    QSPI->CONF.BF.ADDR_PIN = QSPI_ADDR_PIN_SINGLE;
  /* Set Header count register: instruction counter, address counter, read mode counter and dummy counter */
  QSPI_SetHdrcnt(QSPI_INSTR_CNT_2BYTE, QSPI_ADDR_CNT_0BYTE, QSPI_RM_CNT_1BYTE, QSPI_DUMMY_CNT_0BYTE);
  
  QSPI_SetRMode(0);  
  /* Set data in counter */
  QSPI_SetDInCnt(0);
  
  /* Set instruction */
  QSPI_SetInstr(FLASH_INS_CODE_FDRST);
  
  /* Set QSPI write */
  QSPI_StartTransfer(QSPI_W_EN);
  
  /* Stop QSPI transfer */
  QSPI_StopTransfer();  
}

/****************************************************************************//**
 * @brief      Reset Flash Continuous Quad Read mode
 *
 * @param none
 *
 * @return none
 *
 *******************************************************************************/
void FLASH_ResetFastReadQuad(void)
{
  /* Clear QSPI FIFO */ 
  QSPI_FlushFIFO();
  
  QSPI->CONF.BF.DATA_PIN = QSPI_DATA_PIN_QUAD; 
    QSPI->CONF.BF.ADDR_PIN = QSPI_ADDR_PIN_SINGLE;
  /* Set Header count register: instruction counter, address counter, read mode counter and dummy counter */
  QSPI_SetHdrcnt(QSPI_INSTR_CNT_1BYTE, QSPI_ADDR_CNT_0BYTE, QSPI_RM_CNT_1BYTE, QSPI_DUMMY_CNT_0BYTE);
  
  QSPI_SetRMode(0);  
  /* Set data in counter */
  QSPI_SetDInCnt(0);
  
  /* Set instruction */
  QSPI_SetInstr(FLASH_INS_CODE_FQRST);
  
  /* Set QSPI write */
  QSPI_StartTransfer(QSPI_W_EN);
  
  /* Stop QSPI transfer */
  QSPI_StopTransfer();  
}

/****************************************************************************//**
 * @brief      Get the flash status
 *
* @param[in]   statusIdx:  Status[7:0] or Status[15:8]
 *
 * @return     Specified status
 *
 *******************************************************************************/
uint8_t FLASH_GetStatus(FLASH_Status_Type statusIdx)
{
  uint8_t status;
  
  /* Clear QSPI FIFO */ 
  QSPI_FlushFIFO();  
  
  /* Set Header count register: instruction counter, address counter, read mode counter and dummy counter */
  QSPI_SetHdrcnt(QSPI_INSTR_CNT_1BYTE, QSPI_ADDR_CNT_0BYTE, QSPI_RM_CNT_0BYTE, QSPI_DUMMY_CNT_0BYTE);
  
  /* Set data in counter */
  QSPI_SetDInCnt(1);
  
  /* Set QSPI data pin mode */
  QSPI->CONF.BF.DATA_PIN = QSPI_DATA_PIN_SINGLE; 
  
  if(statusIdx == FLASH_STATUS_LO)
  {
    /* Set instruction */
    QSPI_SetInstr(FLASH_INS_CODE_RSR1);
  }
  else if(statusIdx == FLASH_STATUS_HI)
  {
    /* Set instruction */
    QSPI_SetInstr(FLASH_INS_CODE_RSR2);
  }
  
  /* QSPI one-byte length mode */
  QSPI->CONF.BF.BYTE_LEN = QSPI_BYTE_LEN_1BYTE;

  /* Set QSPI read */
  QSPI_StartTransfer(QSPI_R_EN);
  
  /* Get flash busy status */
  status = QSPI_ReadByte();
  
  /* Disable QSPI */
  QSPI_SetSSEnable(DISABLE);  
  
  return status;
}

/****************************************************************************//**
 * @brief      Write flash status register
 *
 * @param[in]  status:  status
 *
 * @return     SUCCESS or ERROR
 *
 *******************************************************************************/
Status FLASH_WriteStatus(uint16_t status)
{
  volatile uint32_t localCnt = 0;
  uint8_t byte;

  if (g_fl_cfg->jedec_id == 0xc22014 || g_fl_cfg->jedec_id == 0xc22017)
	FLASH_SetWriteEnableBit(ENABLE);
  else if (g_fl_cfg->jedec_id == 0xc22315)
  	FLASH_SetWriteEnableBit(ENABLE);
  else
  	/* Enable flash write */
    FLASH_WriteEnableVSR();
  
  /* Set QSPI data pin mode */
  QSPI->CONF.BF.DATA_PIN = QSPI_DATA_PIN_SINGLE; 
  
  /* Set instruction */
  QSPI_SetInstr(FLASH_INS_CODE_WSR1);
  
  /* QSPI one-byte length mode */
  QSPI->CONF.BF.BYTE_LEN = QSPI_BYTE_LEN_1BYTE;

  /* Set QSPI write */
  QSPI_StartTransfer(QSPI_W_EN);
  
  /* Write status[7:0] */
  byte = status & 0xFF;
  QSPI_WriteByte(byte);

  if ((g_fl_cfg->jedec_id != 0xc84016) &&
          (g_fl_cfg->jedec_id != 0xc84018) &&
	  (g_fl_cfg->jedec_id != 0xc22014) && 
    (g_fl_cfg->jedec_id != 0xc22017) && 
	  (g_fl_cfg->jedec_id != 0xc22315)) {
	  /* Write status[15:8] */
	  byte = (status >> 8) & 0xFF;
	  QSPI_WriteByte(byte);
  }
  /* Stop QSPI transfer */
  QSPI_StopTransfer();
  
  while(localCnt++ < 100000)
  {
    /* Check flash busy status */ 
    if( FLASH_GetBusyStatus() == RESET )
    {
	    if ((g_fl_cfg->jedec_id != 0xc84016) &&
			(g_fl_cfg->jedec_id != 0xc84018)) {
		    return SUCCESS;
	    } else {
		    break;
	    }
    }
  }

  /* Following code is specific to GigaDevice Flash GD25Q32C
     only and is written as per its datasheet */
  if ((g_fl_cfg->jedec_id == 0xc84016) ||
		  (g_fl_cfg->jedec_id == 0xc84018)) {
	  /* Write enable for volatile status register */
          FLASH_WriteEnableVSR();

	  /* Set QSPI data pin mode */
	  QSPI->CONF.BF.DATA_PIN = QSPI_DATA_PIN_SINGLE;

	  /* Set instruction */
	  QSPI_SetInstr(FLASH_INS_CODE_WSR2);

	  /* QSPI one-byte length mode */
	  QSPI->CONF.BF.BYTE_LEN = QSPI_BYTE_LEN_1BYTE;

	  /* Set QSPI write */
	  QSPI_StartTransfer(QSPI_W_EN);

	  /* Write status[15:8] */
	  byte = (status >> 8) & 0xFF;
	  QSPI_WriteByte(byte);

	  /* Stop QSPI transfer */
	  QSPI_StopTransfer();

	  localCnt = 0;
	  while(localCnt++ < 100000)
	  {
		  /* Check flash busy status */
		  if( FLASH_GetBusyStatus() == RESET )
		  {
			  return SUCCESS;
		  }
	  }
  }

  return ERROR;
}

/****************************************************************************//**
 * @brief      Check sector is blank or not
 *
 * @param[in]  sectorNumber:  Sector to be checked
 *
 * @return     Status
 *
 *******************************************************************************/
Status FLASH_IsSectorBlank(uint32_t sectorNumber) 
{
  Status funcStatus = SUCCESS;
  uint32_t sectorAddress;
  uint32_t count;
  uint32_t data;
  uint32_t lastSector = (g_fl_cfg->chip_size / g_fl_cfg->sector_size) - 1;

  if(!(sectorNumber > lastSector)) 
  {
    /* Get sector start address */
    sectorAddress = sectorNumber * g_fl_cfg->sector_size;
    
    for(count = 0; count < g_fl_cfg->sector_size; count++)
    {
      data = FLASH_WordRead(FLASH_NORMAL_READ, sectorAddress+count);
      if(data != 0xFFFFFFFF) 
      {
        funcStatus = ERROR;
        break;
      }
    }
  } 
  else
  {
    funcStatus = ERROR;
  }

  return funcStatus;
}

/****************************************************************************//**
 * @brief      Check flash is blank or not
 *
 * @param none
 *
 * @return     Status
 *
 *******************************************************************************/
Status FLASH_IsBlank(void)
{
  Status funcStatus = SUCCESS;
  uint32_t flashData;
  uint32_t i;
  uint32_t maxWordAddr;
  
  maxWordAddr = g_fl_cfg->chip_size >> 2;
  
  for(i=0; i<maxWordAddr; i++)
  {
    flashData = FLASH_WordRead(FLASH_NORMAL_READ, i<<2);
    if(flashData != 0xFFFFFFFF)
    {
      funcStatus = ERROR;
      break;
    }
  }
  
  return funcStatus;
}

/*@} end of group FLASH_Private_Functions */

/** @defgroup FLASH_Public_Functions
 *  @{
 */

/****************************************************************************//**
 * @brief      Select the interface to the flash
 *
 * @param[in]  interface: the interface type
 *
 * @return     none
 *
 *******************************************************************************/
void FLASH_SelectInterface(FLASH_Interface_Type interface)
{
  FLASHC->FCCR.BF.FLASHC_PAD_EN = interface;
}

/****************************************************************************//**
 * @brief      Get the flash busy status
 *
 * @param none
 *
 * @return     Flash busy status
 *
 *******************************************************************************/
FlagStatus FLASH_GetBusyStatus(void)
{
  FlagStatus funcStatus;  
  
  /* Clear QSPI FIFO */ 
  QSPI_FlushFIFO();  
  
  /* Set Header count register: instruction counter, address counter, read mode counter and dummy counter */
  QSPI_SetHdrcnt(QSPI_INSTR_CNT_1BYTE, QSPI_ADDR_CNT_0BYTE, QSPI_RM_CNT_0BYTE, QSPI_DUMMY_CNT_0BYTE);
    
  /* Set data in counter */
  QSPI_SetDInCnt(1);
  
  /* Set QSPI data pin mode */
  QSPI->CONF.BF.DATA_PIN = QSPI_DATA_PIN_SINGLE; 
  
  /* Set instruction */
  QSPI_SetInstr(FLASH_INS_CODE_RSR1);
  
  /* QSPI one-byte length mode */
  QSPI->CONF.BF.BYTE_LEN = QSPI_BYTE_LEN_1BYTE;

  /* Set QSPI read */
  QSPI_StartTransfer(QSPI_R_EN);
  
  /* Get flash busy status */
  funcStatus = (QSPI_ReadByte() & 0x01) ? SET : RESET;
  
  /* Disable QSPI */
  QSPI_SetSSEnable(DISABLE);  
  
  return funcStatus;
}


/****************************************************************************//**
 * @brief      Set flash protection mode 
 *
 * @param[in]  protectMode:  Protection mode
 *
 * @return     SUCCESS or ERROR
 *
 *******************************************************************************/
Status FLASH_SetProtectionMode(FLASH_Protection_Type protectMode)
{
  volatile uint32_t localCnt = 0;
  
  /* Enable flash write */
  FLASH_SetWriteEnableBit(ENABLE);
  
  /* Set QSPI data pin mode */
  QSPI->CONF.BF.DATA_PIN = QSPI_DATA_PIN_SINGLE; 
  
  /* Set instruction */
  QSPI_SetInstr(FLASH_INS_CODE_WSR1);
  
  /* Set QSPI write */
  QSPI_StartTransfer(QSPI_W_EN);
  
  /* Write protection mode (SEC, TB, BP2, BP1, BP0) */
  QSPI_WriteByte(protectMode & 0x7F);
  
  /* Write protection mode (CMP) */
  QSPI_WriteByte((protectMode & 0x80)>>1);
  
  /* Stop QSPI transfer */
  QSPI_StopTransfer();
  
  while(localCnt++ < 100000)
  {
    /* Check flash busy status */ 
    if( FLASH_GetBusyStatus() == RESET )
    {
      return SUCCESS;
    }
  }
  
  return ERROR;
}

Status FLASH_ENSO( void )
{
  volatile uint32_t localCnt = 0;
    
  /* Enable flash write */
  FLASH_SetWriteEnableBit(ENABLE);  
  
  /* Set instruction */
  QSPI_SetInstr(FLASH_INS_CODE_ENSO);
  
  /* Set QSPI write */
  QSPI_StartTransfer(QSPI_W_EN);
  
  /* Stop QSPI transfer */
  QSPI_StopTransfer();
  
  while(localCnt++ < 0xFFFFFFF)
  {
    /* Check flash busy status */ 
    if( FLASH_GetBusyStatus() == RESET )
    {
      return SUCCESS;
    }
  }
  return ERROR;
}

Status FLASH_EXSO( void )
{
  volatile uint32_t localCnt = 0;
    
  /* Enable flash write */
  FLASH_SetWriteEnableBit(ENABLE);  
  
  /* Set instruction */
  QSPI_SetInstr(FLASH_INS_CODE_EXSO);
  
  /* Set QSPI write */
  QSPI_StartTransfer(QSPI_W_EN);
  
  /* Stop QSPI transfer */
  QSPI_StopTransfer();
  
  while(localCnt++ < 0xFFFFFFF)
  {
    /* Check flash busy status */ 
    if( FLASH_GetBusyStatus() == RESET )
    {
      return SUCCESS;
    }
  }
  
  return ERROR;
}

/****************************************************************************//**
 * @brief      Whole flash erase
 *
 * @param none
 *
 * @return     Status
 *
 *******************************************************************************/
Status FLASH_EraseAll(void) 
{
  volatile uint32_t localCnt = 0;
    
  /* Enable flash write */
  FLASH_SetWriteEnableBit(ENABLE);  
  
  /* Set instruction */
  QSPI_SetInstr(FLASH_INS_CODE_CE);
  
  /* Set QSPI write */
  QSPI_StartTransfer(QSPI_W_EN);
  
  /* Stop QSPI transfer */
  QSPI_StopTransfer();
  
  while(localCnt++ < 0xFFFFFFF)
  {
    /* Check flash busy status */ 
    if( FLASH_GetBusyStatus() == RESET )
    {
      return SUCCESS;
    }
  }
  
  return ERROR;    
}

/****************************************************************************//**
 * @brief      Flash sector erase
 *
 * @param[in]  sectorNumber:  Sector number to be erased
 *
 * @return     SUCCESS or ERROR
 *
 *******************************************************************************/
Status FLASH_SectorErase(uint32_t sectorNumber) 
{
  uint32_t sectorAddress;
  uint32_t lastSector = (g_fl_cfg->chip_size / g_fl_cfg->sector_size) - 1;
  volatile uint32_t localCnt = 0;

  if(!(sectorNumber > lastSector)) 
  {
    /* Enable flash write */
    FLASH_SetWriteEnableBit(ENABLE);
    
    /* Get start address for sector to be erased */
    sectorAddress = sectorNumber* g_fl_cfg->sector_size;
    
    /* Set address counter */
    QSPI_SetAddrCnt(QSPI_ADDR_CNT_3BYTE);
   
    /* Set QSPI address pin mode */
    QSPI->CONF.BF.ADDR_PIN = QSPI_ADDR_PIN_SINGLE;
    
    /* Set address */ 
    QSPI_SetAddr(sectorAddress);
    
    /* Set instruction */
    QSPI_SetInstr(FLASH_INS_CODE_SE);
    
    /* Set QSPI write */
    QSPI_StartTransfer(QSPI_W_EN);
    
    /* Stop QSPI transfer */
    QSPI_StopTransfer();
    
    while(localCnt++ < 1000000)
    {
      /* Check flash busy status */ 
      if( FLASH_GetBusyStatus() == RESET )
      {
        return SUCCESS;
      }
    }
  }
  
  return ERROR;
}

/****************************************************************************//**
 * @brief      Flash 32KB block erase
 *
 * @param[in]  sectorNumber:  block number to be erased
 *
 * @return     SUCCESS or ERROR
 *
 *******************************************************************************/
Status FLASH_Block32KErase(uint32_t blockNumber) 
{
  uint32_t blockAddress;
  volatile uint32_t localCnt = 0;
  uint32_t last32KBlock = (g_fl_cfg->chip_size / FLASH_32K_BLOCK_SIZE) - 1;
  
  if(!(blockNumber > last32KBlock) ) 
  {
    /* Enable flash write */
    FLASH_SetWriteEnableBit(ENABLE);
    
    /* Get start address of the block to be erased */
    blockAddress = blockNumber * FLASH_32K_BLOCK_SIZE;
    
    /* Set address counter */
    QSPI_SetAddrCnt(QSPI_ADDR_CNT_3BYTE);
    
    /* Set QSPI address pin mode */
    QSPI->CONF.BF.ADDR_PIN = QSPI_ADDR_PIN_SINGLE;
    
    /* Set address */ 
    QSPI_SetAddr(blockAddress);
    
    /* Set instruction */
    QSPI_SetInstr(FLASH_INS_CODE_BE_32K);
    
    /* Set QSPI write */
    QSPI_StartTransfer(QSPI_W_EN);
    
    /* Stop QSPI transfer */
    QSPI_StopTransfer();
    
    while(localCnt++ < 2000000)
    {
      /* Check flash busy status */ 
      if( FLASH_GetBusyStatus() == RESET )
      {
        return SUCCESS;
      }
    }
  }
  
  return ERROR;
}

/****************************************************************************//**
 * @brief      Flash 64KB block erase
 *
 * @param[in]  sectorNumber:  block number to be erased
 *
 * @return     SUCCESS or ERROR
 *
 *******************************************************************************/
Status FLASH_Block64KErase(uint32_t blockNumber) 
{
  uint32_t blockAddress;
  volatile uint32_t localCnt = 0;
  uint32_t last64KBlock = (g_fl_cfg->chip_size / FLASH_64K_BLOCK_SIZE) - 1;
  
  if(!(blockNumber > last64KBlock) ) 
  {
    /* Enable flash write */
    FLASH_SetWriteEnableBit(ENABLE);
    
    /* Get start address of the block to be erased */
    blockAddress = blockNumber * FLASH_64K_BLOCK_SIZE;
    
    /* Set address counter */
    QSPI_SetAddrCnt(QSPI_ADDR_CNT_3BYTE);
    
    /* Set QSPI address pin mode */
    QSPI->CONF.BF.ADDR_PIN = QSPI_ADDR_PIN_SINGLE;
    
    /* Set address */ 
    QSPI_SetAddr(blockAddress);
    
    /* Set instruction */
    QSPI_SetInstr(FLASH_INS_CODE_BE_64K);
    
    /* Set QSPI write */
    QSPI_StartTransfer(QSPI_W_EN);
    
    /* Stop QSPI transfer */
    QSPI_StopTransfer();
    
    while(localCnt++ < 2000000)
    {
      /* Check flash busy status */ 
      if( FLASH_GetBusyStatus() == RESET )
      {
        return SUCCESS;
      }
    }
  }
  
  return ERROR;
}

/****************************************************************************//**
 * @brief      Erase specfied address of the flash
 *
 * @param[in]  startAddr:  Start address to be erased
 * @param[in]  endAddr:  End address to be erased
 *
 * @return     SUCCESS or ERROR
 *
 *******************************************************************************/
Status FLASH_Erase(uint32_t startAddr, uint32_t endAddr) 
{
  int ret;
  uint32_t sectorNumber, blockNumber, length, validStart;

  length = endAddr - startAddr + 1;

  while (length != 0) {
#ifdef CONFIG_ENABLE_MXCHIP
	feeddog();
#endif
    if ((startAddr & (FLASH_64K_BLOCK_SIZE - 1)) == 0 &&
		length > (FLASH_64K_BLOCK_SIZE - g_fl_cfg->sector_size)) {
	/* Address is a multiple of 64K and length is > (64K block -4K sector)
	 * So directly erase 64K from this address */
	blockNumber = startAddr / FLASH_64K_BLOCK_SIZE;
	ret = FLASH_Block64KErase(blockNumber);
	endAddr = startAddr + FLASH_64K_BLOCK_SIZE;
    } else if ((startAddr & (FLASH_32K_BLOCK_SIZE - 1)) == 0 &&
		length > (FLASH_32K_BLOCK_SIZE - g_fl_cfg->sector_size)) {
	/* Address is a multiple of 32K and length is > (32K block -4K sector)
	* So directly erase 32K from this address */
	blockNumber = startAddr / FLASH_32K_BLOCK_SIZE;
	ret = FLASH_Block32KErase(blockNumber);
	endAddr = startAddr + FLASH_32K_BLOCK_SIZE;
    } else {
	/* Find 4K aligned address and erase 4K sector */
	validStart = startAddr - (startAddr &
			(g_fl_cfg->sector_size - 1));
	sectorNumber = validStart / g_fl_cfg->sector_size;
	ret = FLASH_SectorErase(sectorNumber);
	endAddr = validStart + g_fl_cfg->sector_size;
    }

    /* If erase operation fails then return error */
    if (ret != SUCCESS)
	return ERROR;

    /* Calculate the remaining length that is to be erased yet */
    if (length < (endAddr - startAddr))
	length = 0;
    else
	length -= (endAddr - startAddr);
    startAddr = endAddr;

  }
  return SUCCESS;

}

/****************************************************************************//**
 * @brief      Read flash from specified address to buffer
 *
 * @param[in]  readMode:  Flash reading mode to be set
 * @param[in]  address:  Flash address to be read
 * @param[in]  buffer:  Buffer to hold data read from flash
 * @param[in]  num:  Number of data to be read from flash
 *
 * @return     Number of data read out from flash, in byte
 *
 *******************************************************************************/
uint32_t FLASH_Read(FLASH_ReadMode_Type readMode, uint32_t address, uint8_t *buffer, uint32_t num) 
{
  uint32_t i;
  uint32_t readBytes;
  uint16_t statusLow, statusHigh, statusWrite; 
    
  readBytes = 0;
  statusLow = statusHigh = 0x0000;
  if (flash_config_set != 1)
  	FLASH_SetConfig(FLASH_GetJEDECID());
  
  if( (readMode == FLASH_FAST_READ_QUAD_OUT) || (readMode == FLASH_FAST_READ_QUAD_IO)
    ||(readMode == FLASH_WORD_FAST_READ_QUAD_IO) || (readMode == FLASH_OCTAL_WORD_FAST_READ_QUAD_IO) )
  {
    statusLow = FLASH_GetStatus(FLASH_STATUS_LO);
	if (g_fl_cfg->jedec_id == 0xc22315) {
		/* For Macronix flash, status register can only be written in
		  * non-volatile mode, hence we set quad bit only if its not set already */
		uint16_t qe_bit_set = statusLow & 0x0040;
		if (!qe_bit_set) {
			statusWrite = statusLow | 0x0040;
			FLASH_WriteStatus(statusWrite);
		}
		statusWrite = statusLow | 0x0040;
    } else if (g_fl_cfg->jedec_id != 0xc22014 && g_fl_cfg->jedec_id != 0xc22017) {
	statusHigh = FLASH_GetStatus(FLASH_STATUS_HI);
	statusWrite = ((statusHigh<<8) | statusLow) | 0x0200;
	FLASH_WriteStatus(statusWrite);
    } else {
		statusWrite = statusLow | 0x0040;

    FLASH_WriteStatus(statusWrite);
    }
  }  
  
  /* Clear QSPI FIFO */ 
  QSPI_FlushFIFO();  

  /* Set Header count register: instruction counter, address counter, read mode counter and dummy counter */
  QSPI_SetHdrcnt(QSPI_INSTR_CNT_1BYTE, QSPI_ADDR_CNT_3BYTE, QSPI_RM_CNT_0BYTE, QSPI_DUMMY_CNT_0BYTE);
    
  /* Set read mode */
  QSPI_SetRMode(0);  

  /* Set data in counter */
  QSPI_SetDInCnt(num);  
  
  /* Set address */ 
  QSPI_SetAddr(address);
  
  /* Set QSPI address pin mode */
  QSPI->CONF.BF.ADDR_PIN = QSPI_ADDR_PIN_SINGLE;     
  
  /* Set QSPI data pin mode */
  QSPI->CONF.BF.DATA_PIN = QSPI_DATA_PIN_SINGLE;   

  if(readMode == FLASH_NORMAL_READ)
  {
    /* Set instruction */
    QSPI_SetInstr(FLASH_INS_CODE_RD);
  }
  else if(readMode == FLASH_FAST_READ)
  {
    /* Set instruction */
    QSPI_SetInstr(FLASH_INS_CODE_FR);
    
    /* Set dummy counter */
    QSPI_SetDummyCnt(QSPI_DUMMY_CNT_1BYTE);
  }
  else if(readMode == FLASH_FAST_READ_DUAL_OUT)
  {
    /* Set instruction */
    QSPI_SetInstr(FLASH_INS_CODE_FRDO);
    
    /* Set dummy counter */
    QSPI_SetDummyCnt(QSPI_DUMMY_CNT_1BYTE);
    
    /* Set QSPI data pin mode */
    QSPI->CONF.BF.DATA_PIN = QSPI_DATA_PIN_DUAL;  
  }
  else if(readMode == FLASH_FAST_READ_DUAL_IO)
  {
    /* Set instruction */
    QSPI_SetInstr(FLASH_INS_CODE_FRDIO);
    
    /* Set read mode counter */
    QSPI_SetRModeCnt(QSPI_RM_CNT_1BYTE);
    
    /* Set QSPI data pin mode */
    QSPI->CONF.BF.DATA_PIN = QSPI_DATA_PIN_DUAL;
    
    /* Set QSPI address pin mode */
    QSPI->CONF.BF.ADDR_PIN = QSPI_ADDR_PIN_AS_DATA;   
  }    
  else if(readMode == FLASH_FAST_READ_QUAD_OUT)
  {
    /* Set instruction */
    QSPI_SetInstr(FLASH_INS_CODE_FRQO);
    
    /* Set dummy counter */
    QSPI_SetDummyCnt(QSPI_DUMMY_CNT_1BYTE);
    
    /* Set QSPI data pin mode */
    QSPI->CONF.BF.DATA_PIN = QSPI_DATA_PIN_QUAD;  
  }
  else if(readMode == FLASH_FAST_READ_QUAD_IO)
  {
    /* Set instruction */
    QSPI_SetInstr(FLASH_INS_CODE_FRQIO);
 
    /* Set read mode counter */
    QSPI_SetRModeCnt(QSPI_RM_CNT_1BYTE);
        
    /* Set dummy counter */
    QSPI_SetDummyCnt(QSPI_DUMMY_CNT_2BYTE);    
    
    /* Set QSPI data pin mode */
    QSPI->CONF.BF.DATA_PIN = QSPI_DATA_PIN_QUAD;
    
    /* Set QSPI address pin mode */
    QSPI->CONF.BF.ADDR_PIN = QSPI_ADDR_PIN_AS_DATA;      
  }
  else if(readMode == FLASH_WORD_FAST_READ_QUAD_IO)
  {
    /* Set instruction */
    QSPI_SetInstr(FLASH_INS_CODE_WFRQIO);
    
    /* Set read mode counter */
    QSPI_SetRModeCnt(QSPI_RM_CNT_1BYTE);
        
    /* Set dummy counter */
    QSPI_SetDummyCnt(QSPI_DUMMY_CNT_1BYTE);    
    
    /* Set QSPI data pin mode */
    QSPI->CONF.BF.DATA_PIN = QSPI_DATA_PIN_QUAD;
    
    /* Set QSPI address pin mode */
    QSPI->CONF.BF.ADDR_PIN = QSPI_ADDR_PIN_AS_DATA;         
  }
  else if(readMode == FLASH_OCTAL_WORD_FAST_READ_QUAD_IO)
  {
    /* Set instruction */
    QSPI_SetInstr(FLASH_INS_CODE_OWFRQIO);
    
    /* Set read mode counter */
    QSPI_SetRModeCnt(QSPI_RM_CNT_1BYTE);
        
    /* Set QSPI data pin mode */
    QSPI->CONF.BF.DATA_PIN = QSPI_DATA_PIN_QUAD;
    
    /* Set QSPI address pin mode */
    QSPI->CONF.BF.ADDR_PIN = QSPI_ADDR_PIN_AS_DATA;         
  }
    
  /* QSPI one-byte length mode */
  QSPI->CONF.BF.BYTE_LEN = QSPI_BYTE_LEN_1BYTE;
  
  /* Set QSPI read */
  QSPI_StartTransfer(QSPI_R_EN);
    
  for (i=0; i<num; i++)
  {
    /* Waiting for RFIFO not empty */
    while(QSPI->CNTL.BF.RFIFO_EMPTY == 1);
    
    buffer[i] = (QSPI->DIN.WORDVAL) & 0xFF;
    readBytes++;
  } 
  
  /* Disable QSPI */
  QSPI_SetSSEnable(DISABLE); 
  
  return readBytes;
}

/****************************************************************************//**
 * @brief      Read a word from specified flash address
 *
 * @param[in]  readMode:  Flash reading mode to be set
 * @param[in]  address:  Flash address to be read
 *
 * @return     Data in word
 *
 *******************************************************************************/
uint32_t FLASH_WordRead(FLASH_ReadMode_Type readMode, uint32_t address) 
{
  uint32_t data = 0;
  FLASH_Read(readMode, address, (uint8_t*)&data, 4);
  return data;
}

/****************************************************************************//**
 * @brief      Read a byte from specified flash address
 *
 * @param[in]  readMode:  Flash reading mode to be set
 * @param[in]  address:  Flash address to be read
 *
 * @return     Data in byte
 *
 *******************************************************************************/
uint8_t FLASH_ByteRead(FLASH_ReadMode_Type readMode, uint32_t address) 
{
  uint8_t data = 0;
  FLASH_Read(readMode, address, (uint8_t*)&data, 1);
  return data;
}

/****************************************************************************//**
 * @brief      Write flash within a page
 *
 * @param[in]  programMode:  Flash program mode to be set
 * @param[in]  address:  Page address
 * @param[in]  buffer:  Buffer data to be programmed to flash
 * @param[in]  num:  Number of data to be programmed to flash
 *
 * @return     SUCCESS or ERROR
 *
 *******************************************************************************/
Status FLASH_PageWrite(FLASH_ProgramMode_Type programMode, uint32_t address, uint8_t *buffer, uint32_t num) 
{
  uint32_t i;
  volatile uint32_t localCnt = 0;
  uint16_t statusLow, statusHigh, statusWrite; 

#ifdef CONFIG_ENABLE_MXCHIP
	feeddog();
#endif
  statusHigh = 0x0000;
  statusLow = FLASH_GetStatus(FLASH_STATUS_LO);
  if(programMode == FLASH_PROGRAM_QUAD)
  {
    if (g_fl_cfg->jedec_id == 0xc22315) {
		/* For Macronix flash , status register can only be written in
		  * non-volatile mode, hence we set quad bit only if its not set already */
		uint16_t qe_bit_set = statusLow & 0x0040;
		if (!qe_bit_set) {
			statusWrite = statusLow | 0x0040;
			FLASH_WriteStatus(statusWrite);
		}

	} else if (g_fl_cfg->jedec_id != 0xc22014 && g_fl_cfg->jedec_id != 0xc22017) {
	statusHigh = FLASH_GetStatus(FLASH_STATUS_HI);
	statusWrite = ((statusHigh<<8) | statusLow) | 0x0200;
	FLASH_WriteStatus(statusWrite);  
    } else {
	statusWrite = statusLow | 0x0040;

    FLASH_WriteStatus(statusWrite);    
    }
  }    
  
  /* Check address validity */
  if ((FLASH_PAGE_NUM(address+num-1) > FLASH_PAGE_NUM(address)) || num == 0)
  {
    return ERROR;
  }
  
  /* Enable flash write */
  FLASH_SetWriteEnableBit(ENABLE);
  
  /* Set Header count register: instruction counter, address counter, read mode counter and dummy counter */
  QSPI_SetHdrcnt(QSPI_INSTR_CNT_1BYTE, QSPI_ADDR_CNT_3BYTE, QSPI_RM_CNT_0BYTE, QSPI_DUMMY_CNT_0BYTE);
  
  /* Set QSPI address pin mode */
  QSPI->CONF.BF.ADDR_PIN = QSPI_ADDR_PIN_SINGLE;
  
  /* Set QSPI data pin mode */
  QSPI->CONF.BF.DATA_PIN = QSPI_DATA_PIN_SINGLE;   
  
  /* Set address */ 
  QSPI_SetAddr(address);
  
  if(programMode == FLASH_PROGRAM_NORMAL)
  {
    /* Set instruction */
    QSPI_SetInstr(FLASH_INS_CODE_PP);    
  }
  else if(programMode == FLASH_PROGRAM_QUAD)
  {
    /* Set QSPI data pin mode */
    QSPI->CONF.BF.DATA_PIN = QSPI_DATA_PIN_QUAD;

    if (g_fl_cfg->jedec_id == 0xc22315) {
		QSPI->CONF.BF.ADDR_PIN = QSPI_ADDR_PIN_AS_DATA;
		/* Set instruction */
		QSPI_SetInstr(FLASH_INS_CODE_QPP_MX);
    } else if (g_fl_cfg->jedec_id == 0xc22014 || g_fl_cfg->jedec_id == 0xc22017) {
	QSPI->CONF.BF.ADDR_PIN = QSPI_ADDR_PIN_AS_DATA;
	/* Set instruction */
	QSPI_SetInstr(FLASH_INS_CODE_QPP_MX);
    } else
	QSPI_SetInstr(FLASH_INS_CODE_QPP);
  }
  
  /* QSPI one-byte length mode */
  QSPI->CONF.BF.BYTE_LEN = QSPI_BYTE_LEN_1BYTE;
  
  /* Set QSPI write */
  QSPI_StartTransfer(QSPI_W_EN);
    
  for (i=0; i<num; i++) 
  {
    QSPI_WriteByte(buffer[i]);
  }
  
  /* Stop QSPI transfer */
  QSPI_StopTransfer();
  
  while(localCnt++ < 1000000)
  {
    /* Check flash busy status */ 
    if( FLASH_GetBusyStatus() == RESET )
    {
      return SUCCESS;
    }
  }  

  return ERROR;
}

/****************************************************************************//**
 * @brief      Write flash with any address and size
 *
 * @param[in]  programMode:  Flash program mode to be set
 * @param[in]  address:  Page address
 * @param[in]  buffer:  Buffer data to be programmed to flash
 * @param[in]  num:  Number of data to be programmed to flash
 *
 * @return     SUCCESS or ERROR
 *
 *******************************************************************************/
Status FLASH_Write(FLASH_ProgramMode_Type programMode, uint32_t address, uint8_t *buffer, uint32_t num) 
{
  uint8_t *pBuf;
  uint32_t begPgNum;
  uint32_t endPgNum;
  uint32_t step;
  uint32_t addrCur;
  uint32_t i;
  uint32_t endPgAddr;
  Status funcStatus = SUCCESS;  
  
  pBuf = buffer;
  addrCur = address;
  
  /* Get page number of start address */
  begPgNum = FLASH_PAGE_NUM(address);
  /* Get page number of end address */
  endPgNum = FLASH_PAGE_NUM(address + num - 1);
  if (flash_config_set != 1)
  	FLASH_SetConfig(FLASH_GetJEDECID());
  /* Both start address and end address are within the same page */
  if(begPgNum == endPgNum)
  {
    return( FLASH_PageWrite(programMode, address, buffer, num) );
  } 
  /* Start address and end address are not in the same page */
  else
  {
    /* For first page */
    endPgAddr = (g_fl_cfg->page_size * (FLASH_PAGE_NUM(address) + 1) - 1);
    step = endPgAddr - address + 1;
    funcStatus = FLASH_PageWrite(programMode, address, pBuf, step);
    if(funcStatus == ERROR)
    {
      return ERROR;
    }
    
    pBuf += step;
    addrCur += step;

    for(i=begPgNum+1; i<=endPgNum; i++)
    {
      /* For last page */
      if(i == endPgNum)
      {
        step = (address + num) & 0xFF;
        
        /* If step is 0, the last page has 256 bytes data to be writen ( num of data is 0x100 ) */
        if(step == 0)
        {
          step = 0x100;
        }
        
        return( FLASH_PageWrite(programMode, addrCur, pBuf, step) );
      } 
      else
      {
        funcStatus = FLASH_PageWrite(programMode, addrCur, pBuf, g_fl_cfg->page_size);
        if(funcStatus == ERROR)
        {
          return ERROR;
        }
        
        pBuf += g_fl_cfg->page_size;
        addrCur += g_fl_cfg->page_size;
      }
    }
  }
  
  return funcStatus;
}

/****************************************************************************//**
 * @brief      Write a word to specified flash address
 *
 * @param[in]  programMode:  Flash program mode to be set
 * @param[in]  address:  Flash address to be programmed
 * @param[in]  data:  Data to be programmed to flash
 *
 * @return     SUCCESS or ERROR
 *
 *******************************************************************************/
Status FLASH_WordWrite(FLASH_ProgramMode_Type programMode, uint32_t address, uint32_t data) 
{
  return FLASH_Write(programMode, address, (uint8_t*)&data, 4);
}

/****************************************************************************//**
 * @brief      Write a byte to specified flash address
 *
 * @param[in]  address:  Flash address to be programmed
 * @param[in]  data:  Data to be programmed to flash
 *
 * @return     SUCCESS or ERROR
 *
 *******************************************************************************/
Status FLASH_ByteWrite(FLASH_ProgramMode_Type programMode, uint32_t address, uint8_t data) 
{
  return FLASH_Write(programMode, address, (uint8_t*)&data, 1);
}

/****************************************************************************//**
 * @brief      Get flash unique ID
 *
 * @param none
 *
 * @return     Unique ID
 *
 *******************************************************************************/
uint64_t FLASH_GetUniqueID(void)
{
  uint64_t uniqueID;
  
  QSPI_FlushFIFO();  
  
  /* Set Header count register: instruction counter, address counter, read mode counter and dummy counter*/
  QSPI_SetHdrcnt(QSPI_INSTR_CNT_1BYTE, QSPI_ADDR_CNT_1BYTE, QSPI_RM_CNT_0BYTE, QSPI_DUMMY_CNT_3BYTE);
  
  QSPI_SetDInCnt(8);
  
  QSPI_SetAddr(0);
  
  /* Read Unique ID number */
  QSPI_SetInstr(FLASH_INS_CODE_RUID);
  
  /* Set QSPI address pin mode */
  QSPI->CONF.BF.ADDR_PIN = QSPI_ADDR_PIN_SINGLE;
  
  /* Set QSPI data pin mode */
  QSPI->CONF.BF.DATA_PIN = QSPI_DATA_PIN_SINGLE;   

  /* QSPI one-byte length mode */
  QSPI->CONF.BF.BYTE_LEN = QSPI_BYTE_LEN_1BYTE;

  QSPI_StartTransfer(QSPI_R_EN);

  uniqueID = QSPI_ReadByte();
  uniqueID <<= 8;
  uniqueID |= QSPI_ReadByte();
  uniqueID <<= 8;
  uniqueID |= QSPI_ReadByte();
  uniqueID <<= 8;
  uniqueID |= QSPI_ReadByte();
  uniqueID <<= 8;
  uniqueID |= QSPI_ReadByte();
  uniqueID <<= 8;
  uniqueID |= QSPI_ReadByte();
  uniqueID <<= 8;
  uniqueID |= QSPI_ReadByte();
  uniqueID <<= 8;
  uniqueID |= QSPI_ReadByte();  
  
  QSPI_SetSSEnable(DISABLE);
  
  return uniqueID;
}


uint32_t FLASH_GetJEDECID(void)
{
  uint32_t jedecID = 0;

  QSPI_FlushFIFO();

  /* Set Header count register: instruction counter, address counter, read mode counter and dummy counter*/
  QSPI_SetHdrcnt(QSPI_INSTR_CNT_1BYTE, QSPI_ADDR_CNT_0BYTE, QSPI_RM_CNT_0BYTE, QSPI_DUMMY_CNT_0BYTE);

  QSPI_SetDInCnt(3);

  /* Read JEDEC ID */
  QSPI_SetInstr(FLASH_INS_CODE_JEDEC_ID);

  /* Set QSPI address pin mode */
  QSPI->CONF.BF.ADDR_PIN = QSPI_ADDR_PIN_SINGLE;

  /* Set QSPI data pin mode */
  QSPI->CONF.BF.DATA_PIN = QSPI_DATA_PIN_SINGLE;

  /* QSPI one-byte length mode */
  QSPI->CONF.BF.BYTE_LEN = QSPI_BYTE_LEN_1BYTE;

  QSPI_StartTransfer(QSPI_R_EN);

  jedecID = QSPI_ReadByte();
  jedecID <<= 8;
  jedecID |= QSPI_ReadByte();
  jedecID <<= 8;
  jedecID |= QSPI_ReadByte();

  QSPI_SetSSEnable(DISABLE);

  return jedecID;
}

static const struct flash_device_config *FLASH_GetConfigFromID(uint32_t jedecID)
{
  int i, count = sizeof(fl_dev_list) / sizeof(struct flash_device_config);
  for (i = 0; i < count; i++)
  {
	if (fl_dev_list[i].jedec_id == jedecID)
		return &fl_dev_list[i];
  }
  return NULL;

}

Status FLASH_SetConfig(uint32_t jedecID)
{
   if (flash_config_set != 1) {
 	const struct flash_device_config *cfg;
	cfg = FLASH_GetConfigFromID(jedecID);
	if (cfg) {
		g_fl_cfg = cfg;
		flash_config_set = 1;
		return SUCCESS;
    }
    return ERROR;
  } else
    return SUCCESS;
}

const struct flash_device_config *FLASH_GetConfig(void)
{
	return g_fl_cfg;
}

/*@} end of group FLASH_Public_Functions */

/*@} end of group FLASH_definitions */

/*@} end of group MW300_Periph_Driver */

