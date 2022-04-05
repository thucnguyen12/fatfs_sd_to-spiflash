/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_storage_if.c
  * @version        : v1.0_Cube
  * @brief          : Memory management layer.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_storage_if.h"

/* USER CODE BEGIN INCLUDE */
#include "app_debug.h"
#include "app_spi_flash.h"
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "spi.h"
#include "app_drv_spi.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define SPI_CMD_READ            0
#define SPI_CMD_WRITE           1
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @defgroup USBD_STORAGE
  * @brief Usb mass storage device module
  * @{
  */

/** @defgroup USBD_STORAGE_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Defines
  * @brief Private defines.
  * @{
  */

#define STORAGE_LUN_NBR                  1
//#define STORAGE_BLK_NBR                  128
#define STORAGE_BLK_SIZ                  4096

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Variables
  * @brief Private variables.
  * @{
  */

/* USER CODE BEGIN INQUIRY_DATA_FS */
/** USB Mass storage Standard Inquiry Data. */
const int8_t STORAGE_Inquirydata_FS[] = {/* 36 */

  /* LUN 0 */
  0x00,
  0x80,
  0x02,
  0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00,
  0x00,
  0x00,
  'S', 'T', 'M', ' ', ' ', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
  'P', 'r', 'o', 'd', 'u', 'c', 't', ' ', /* Product      : 16 Bytes */
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '0', '.', '0' ,'1'                      /* Version      : 4 Bytes */
};
/* USER CODE END INQUIRY_DATA_FS */

/* USER CODE BEGIN PRIVATE_VARIABLES */
//uint8_t buffer[STORAGE_BLK_NBR*STORAGE_BLK_SIZ];
static app_flash_drv_t m_spi_flash;
static void spi_flash_delay(void *arg, uint32_t ms);
static void task_spi_flash_cmd(void *arg);
static QueueHandle_t m_cmd_queue;
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t STORAGE_Init_FS(uint8_t lun);
static int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size);
static int8_t STORAGE_IsReady_FS(uint8_t lun);
static int8_t STORAGE_IsWriteProtected_FS(uint8_t lun);
static int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_GetMaxLun_FS(void);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
void usbd_storage_flash_initialize(void)
{
    app_drv_spi_initialize();
    m_spi_flash.error = false;
    m_spi_flash.spi = &hspi1;
    m_spi_flash.callback.spi_cs = app_drv_spi_cs;
    m_spi_flash.callback.spi_rx_buffer = app_drv_spi_receive_frame;
    m_spi_flash.callback.spi_tx_buffer = app_drv_spi_transmit_frame;
    m_spi_flash.callback.spi_tx_rx = app_drv_spi_transmit_receive_frame;
    m_spi_flash.callback.spi_tx_byte = app_drv_spi_transmit_byte;
    m_spi_flash.callback.delay_ms = spi_flash_delay;
    
    if (app_spi_flash_initialize(&m_spi_flash) == false)
    {
        m_spi_flash.error = true;
        DEBUG_ERROR("SPI flash error\r\n");
    }
    
    if (!m_cmd_queue)
    {
        m_cmd_queue = xQueueCreate(4, sizeof(uint8_t));
    }
}
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_StorageTypeDef USBD_Storage_Interface_fops_FS =
{
  STORAGE_Init_FS,
  STORAGE_GetCapacity_FS,
  STORAGE_IsReady_FS,
  STORAGE_IsWriteProtected_FS,
  STORAGE_Read_FS,
  STORAGE_Write_FS,
  STORAGE_GetMaxLun_FS,
  (int8_t *)STORAGE_Inquirydata_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the storage unit (medium) over USB FS IP
  * @param  lun: Logical unit number.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Init_FS(uint8_t lun)
{
  /* USER CODE BEGIN 2 */
    UNUSED(lun);

    return (USBD_OK);
  /* USER CODE END 2 */
}

/**
  * @brief  Returns the medium capacity.
  * @param  lun: Logical unit number.
  * @param  block_num: Number of total block number.
  * @param  block_size: Block size.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
  /* USER CODE BEGIN 3 */
  UNUSED(lun);

  *block_num  = m_spi_flash.info.size/STORAGE_BLK_SIZ;
  *block_size = STORAGE_BLK_SIZ;
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief   Checks whether the medium is ready.
  * @param  lun:  Logical unit number.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_IsReady_FS(uint8_t lun)
{
  /* USER CODE BEGIN 4 */
  UNUSED(lun);

  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Checks whether the medium is write protected.
  * @param  lun: Logical unit number.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_IsWriteProtected_FS(uint8_t lun)
{
  /* USER CODE BEGIN 5 */
  UNUSED(lun);

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Reads data from the medium.
  * @param  lun: Logical unit number.
  * @param  buf: data buffer.
  * @param  blk_addr: Logical block address.
  * @param  blk_len: Blocks number.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  /* USER CODE BEGIN 6 */
    
  UNUSED(lun);
  UNUSED(buf);
  UNUSED(blk_addr);
  UNUSED(blk_len);
    DEBUG_INFO("%s read at block addr %u, len %u\r\n", __FUNCTION__, blk_addr, blk_len);
    app_spi_flash_read_bytes(&m_spi_flash, blk_addr*STORAGE_BLK_SIZ, buf, blk_len*STORAGE_BLK_SIZ);
//    memcpy(buf, &buffer[blk_addr*STORAGE_BLK_SIZ], blk_len*STORAGE_BLK_SIZ);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  Writes data into the medium.
  * @param  lun: Logical unit number.
  * @param  buf: data buffer.
  * @param  blk_addr: Logical block address.
  * @param  blk_len: Blocks number.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  /* USER CODE BEGIN 7 */
  UNUSED(lun);
  UNUSED(buf);
  UNUSED(blk_addr);
  UNUSED(blk_len);
    DEBUG_INFO("%s write at block addr %u, len %u\r\n", __FUNCTION__, blk_addr, blk_len);
    // memcpy(&buffer[blk_addr*STORAGE_BLK_SIZ], buf, blk_len*STORAGE_BLK_SIZ);
    for (uint32_t i = 0; i < blk_len; i++)
    {
        app_spi_flash_erase_sector_4k(&m_spi_flash, blk_addr+i);
    }
    app_spi_flash_write(&m_spi_flash, blk_addr*STORAGE_BLK_SIZ, buf, blk_len*STORAGE_BLK_SIZ);
    
    return (USBD_OK);
  /* USER CODE END 7 */
}

/**
  * @brief  Returns the Max Supported LUNs.
  * @param  None
  * @retval Lun(s) number.
  */
int8_t STORAGE_GetMaxLun_FS(void)
{
  /* USER CODE BEGIN 8 */
  return (STORAGE_LUN_NBR - 1);
  /* USER CODE END 8 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
static void spi_flash_delay(void *arg, uint32_t ms)
{
    uint32_t now = xTaskGetTickCount();
    vTaskDelayUntil(&now, ms);
}

static void task_spi_flash_cmd(void *arg)
{
    uint8_t cmd;
    for (;;)
    {
        if (!xQueueReceive(m_cmd_queue, &cmd, portMAX_DELAY))
        {
            continue;
        }
        
        switch(cmd)
        {
            case SPI_CMD_READ:
                break;
            case SPI_CMD_WRITE:
                break;
            default:
                break;
        }
    }
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

