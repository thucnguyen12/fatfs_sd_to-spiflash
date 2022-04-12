/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.h
  * @brief  Header for fatfs applications
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __fatfs_H
#define __fatfs_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "ff.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h" /* defines SD_Driver as external */
#include "user_diskio.h" /* defines USER_Driver as external */

/* USER CODE BEGIN Includes */
extern    FIL SFLASHFile;

/* USER CODE END Includes */

extern uint8_t retSD; /* Return value for SD */
extern char SDPath[4]; /* SD logical drive path */
extern FATFS SDFatFS; /* File system object for SD logical drive */
extern FIL SDFile; /* File object for SD */
extern uint8_t retUSER; /* Return value for USER */
extern char USERPath[4]; /* USER logical drive path */
extern FATFS USERFatFS; /* File system object for USER logical drive */
extern FIL USERFile; /* File object for USER */

void MX_FATFS_Init(void);

/* USER CODE BEGIN Prototypes */
void mount_sd (void);
void mount_flash (void);
void read_SD_file (char *name,  uint8_t *buff);
void writeToFlash (TCHAR* path, void * sendbuff, UINT sizeWrite);
void readFileFromFlash (char *name, void * buff, UINT sizeRead);
void writeToFlashABuff (TCHAR* path, void * sendbuff, uint8_t numOfBuff);
void read_Buff_From_SD (char *name,  uint8_t *buff, uint8_t numOfBuff);
/* USER CODE END Prototypes */
#ifdef __cplusplus
}
#endif
#endif /*__fatfs_H */
