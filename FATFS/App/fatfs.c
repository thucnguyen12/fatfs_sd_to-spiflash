/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
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
  #include "app_debug.h"
/* USER CODE END Header */
#include "fatfs.h"

uint8_t retSD;    /* Return value for SD */
char SDPath[4];   /* SD logical drive path */
FATFS SDFatFS;    /* File system object for SD logical drive */
FIL SDFile;       /* File object for SD */
uint8_t retUSER;    /* Return value for USER */
char USERPath[4];   /* USER logical drive path */
FATFS USERFatFS;    /* File system object for USER logical drive */
FIL USERFile;       /* File object for USER */

/* USER CODE BEGIN Variables */
    FILINFO fno;
    FRESULT SD_res;
    FRESULT flash_res; 
    FATFS FlashFatFs;
    FIL SFLASHFile;
    char SFLASHPath[4];     /* SFLASH logical drive path */
    BYTE gFSWork[_MAX_SS];
    UINT br, bw;  // File read/write count
    UINT fbr, fbw;  // File read/write count
/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the SD driver ###########################*/
      
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);
  /*## FatFS: Link the USER driver ###########################*/
    retUSER = FATFS_LinkDriver(&USER_Driver, USERPath);
  /* USER CODE BEGIN Init */
//FATFS_LinkDriver(&USER_Driver, SFLASHPath);
  /* additional user code for init */
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */
void mount_sd (void)
{
    SD_res = f_mount(&SDFatFS, SDPath, 1);
    if (SD_res != FR_OK) DEBUG_INFO ("error in mount sd caed \r\n");
    else DEBUG_INFO ("mount sd card ok! \r\n");
}
void mount_flash (void)
{
//    USERFatFS.drv = 1;
//    USERFile.obj.fs->drv = 1;
    flash_res = f_mount(&USERFatFS, USERPath, 1);
    if (flash_res != FR_OK)
    {
        DEBUG_INFO ("Mount flash fail \r\n");
        flash_res = f_mkfs(USERPath, FM_ANY, 0, gFSWork, sizeof gFSWork);
        flash_res = f_mount(&USERFatFS, USERPath, 1);
        if (flash_res == FR_OK)
        {
            DEBUG_INFO ("format disk and mount again \r\n");
            
        }
        else
        {
        while(1);
        }
    }
    else
    {
    DEBUG_INFO ("Mount flash ok \r\n");
    }
}
void read_SD_file (char *name,  uint8_t *buff)
{
   
	/**** check whether the file exists or not ****/
	SD_res = f_stat (name, &fno);
	if (SD_res != FR_OK)
	{
		DEBUG_INFO ("%s dosen't exist\r\n", name);
	}

	else
	{
		/* Open file to read */
		SD_res = f_open(&SDFile, name, FA_READ);

		if (SD_res != FR_OK)
		{
			DEBUG_INFO ("open file error \r\n");
		}

		/* Read data from the file
		* see the function details for the arguments */
        DEBUG_INFO ("reding file...\r\n");
        
		SD_res = f_read (&SDFile, buff, f_size(&SDFile), &br);
		if (SD_res != FR_OK)
		{
		  	
		 	DEBUG_INFO ("error in read file from sd card \r\n");
		}

		else DEBUG_INFO ("read file from sd card ok \r\n");


		/* Close file */
		SD_res = f_close(&SDFile);
		if (SD_res != FR_OK)
		{
		   	DEBUG_INFO ("close file error \r\n");
		}
	}
}

void read_Buff_From_SD (char *name,  uint8_t *buff, uint8_t numOfBuff)
{   
    //FIL*    fp;
    SD_res = f_open(&SDFile, name, FA_READ);
    if (SD_res != FR_OK)
    {
        DEBUG_INFO ("error in opening block %d of sd card\r\n", numOfBuff);
    }
    // add debug print
    SD_res = f_lseek (&SDFile, 256 *numOfBuff);
    if (SD_res != FR_OK)
    {
        DEBUG_INFO ("error in seeking block %d of sd card\r\n", numOfBuff);
    }
    SD_res = f_read (&SDFile, buff, 256, &br);
    if (SD_res != FR_OK)
    {
        DEBUG_INFO ("error in reading block %d of sd card\r\n", numOfBuff);
    }
    SD_res = f_close (&SDFile);
    if (SD_res != FR_OK)
    {
        DEBUG_INFO ("error in closing block %d of sd card\r\n");
    }
}



void writeToFlash (TCHAR* path, void * sendbuff, UINT sizeWrite)
{
    
    flash_res = f_open (&USERFile, path, FA_CREATE_ALWAYS | FA_WRITE);
    //USERFile.obj.fs->drv = 1;
    if (flash_res != FR_OK)
    {
     
      DEBUG_INFO ("open fail!! \r\n");
         while(1);
    }
    else
    {
        DEBUG_INFO ("open file ok \r\n");
//     USERFile.obj.fs->drv = 1;
     flash_res = f_write (&USERFile, sendbuff, sizeWrite, (UINT*)fbw);
      //  USERFile.obj.fs->drv = 0x82;
        if(flash_res != FR_OK)
        DEBUG_INFO ("write file error \r\n");
        else DEBUG_INFO ("write file ok \r\n");
    }
    f_close (&USERFile);
    
}
void readFileFromFlash (TCHAR* path, void * buff, UINT sizeRead)
{
    
    USERFile.obj.fs->drv = 1;
    /**** check whether the file exists or not ****/
	flash_res = f_stat (path, &fno);
	if (flash_res != FR_OK)
	{
		DEBUG_INFO ("%s dosen't exist\r\n", path);
	}
    flash_res = f_open(&USERFile, path, FA_READ);
    
    if (flash_res != FR_OK)
    {
     
      DEBUG_INFO ("open fail!! \r\n");
    }
    else
    {
        DEBUG_INFO ("open file ok \r\n");

        flash_res = f_read(&USERFile, buff, sizeRead, &fbr);
        
        if(flash_res != FR_OK)
        DEBUG_INFO ("read file error \r\n");
        else DEBUG_INFO ("read file ok \r\n");;
    }
    f_close (&USERFile);

}

void writeToFlashABuff (TCHAR* path, void * sendbuff, uint8_t numOfBuff)
{
    FIL*    fp;
    flash_res = f_open (&USERFile, path, FA_CREATE_ALWAYS | FA_WRITE);
    if (SD_res != FR_OK)
    {
        DEBUG_INFO ("error in opening block %d of flash \r\n", numOfBuff);
    }
    SD_res = f_lseek (fp, 256 *numOfBuff);
    if (SD_res != FR_OK)
    {
        DEBUG_INFO ("error in seeking block %d of sd flash\r\n", numOfBuff);
    }
    flash_res = f_write (fp, sendbuff, 256, &fbw);
        if (SD_res != FR_OK)
    {
        DEBUG_INFO ("error in writing block %d of sd flash\r\n", numOfBuff);
    }
    f_close (&USERFile);
        if (SD_res != FR_OK)
    {
        DEBUG_INFO ("error in closing block %d of sd flash\r\n", numOfBuff);
    }
}
/* USER CODE END Application */
