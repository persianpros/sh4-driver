/*****************************************************************************/
/* COPYRIGHT (C) 2009 STMicroelectronics - All Rights Reserved               */
/* ST makes no warranty express or implied including but not limited to,     */
/* any warranty of                                                           */
/*                                                                           */
/*   (i)  merchantability or fitness for a particular purpose and/or         */
/*   (ii) requirements, for a particular purpose in relation to the LICENSED */
/*        MATERIALS, which is provided AS IS, WITH ALL FAULTS. ST does not   */
/*        represent or warrant that the LICENSED MATERIALS provided here     */
/*        under is free of infringement of any third party patents,          */
/*        copyrights, trade secrets or other intellectual property rights.   */
/*        ALL WARRANTIES, CONDITIONS OR OTHER TERMS IMPLIED BY LAW ARE       */
/*        EXCLUDED TO THE FULLEST EXTENT PERMITTED BY LAW                    */
/*                                                                           */
/*****************************************************************************/
/**
 * @file     lxloader.c
 * @brief    This is the firmware loader tool.
 * @author   STMicroelectronics
 */

/* Include */
/* ------- */
#define EXPORT_SYMTAB
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/vmalloc.h>
#include <asm/uaccess.h>
#include <asm/errno.h>
#include <asm/io.h>
//#include "stddefs.h"
//#include "stsys.h"
//#include "stos.h"
#include <linux/netdevice.h>    /* ??? SET_MODULE_OWNER ??? */

#include "lxloader.h"

/* Constants */
/* --------- */
#define LXLOAD_LINUX_DEVICE_NAME "lxload"
#define LXLOAD_STAT_NAME_LEN     (sizeof(LXLOAD_LINUX_DEVICE_NAME) + 2)

/* Macros */
/* ------ */
#define LXLOADi_TraceDBG(x)
#define LXLOADi_TraceERR(x) printk x

/* Linux compatibility */
/* ------------------- */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 12)
#define class_simple                                class
#define class_simple_create(a, b)                   ((struct class *)1)
#define class_simple_destroy(a)
#define class_simple_device_add(a, b, c, d_args...) ((struct class_device *)1)
#define class_simple_device_remove(a)
#endif

/* Prototypes */
/* ---------- */
static int            lxload_init(void);
static void           lxload_exit(void);
static int            lxload_open(struct inode *inode, struct file *file);
static int            lxload_close(struct inode *inode, struct file *file);
static int            lxload_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
static ssize_t        lxload_write(struct file  *file , const char *buffer, size_t length, loff_t *offset);
static int            lxload_device_register(struct file_operations *stdevice_fops, unsigned int nbdevices, char *device_name, unsigned int *device_major, struct cdev **stdevice_cdev, struct class_simple **stdevice_class);
static int            lxload_device_unregister(unsigned int nbdevices, char *device_name, unsigned int device_major, struct cdev *stdevice_cdev, struct class_simple *stdevice_class);
static void          *lxload_map_region(void *Address_p, unsigned int Width, char *RegionName);
static void           lxload_unmap_region(void *MappedAddress_p, unsigned int Width);
static ssize_t        lxload_sysfs_show(struct class *class, char *buffer);
static ssize_t        lxload_sysfs_store(struct class *class, const char *buffer, size_t size);
static ST_ErrorCode_t lxload_create_lxmapping(void);


/* Private Variables */
/* ----------------- */
static char *type;
/* Variables to store which firmwares are configure for loading */
/* ------------------------------------------------------------ */
static int VideoFirmware1            = FALSE;
static int AudioFirmware1            = FALSE;
static int VideoFirmware2            = FALSE;
static int AudioFirmware2            = FALSE;
static int EcmFirmware1              = FALSE;
static int RtFirmware1               = FALSE;
static int OffloadFirmware1          = FALSE;
/* Variables to store physical start address of requested firmware */
/* --------------------------------------------------------------- */
static unsigned int Video1PhysicalStartAddress = 0;
static unsigned int Audio1PhysicalStartAddress = 0;
static unsigned int Video2PhysicalStartAddress = 0;
static unsigned int Audio2PhysicalStartAddress = 0;
static unsigned int Ecm1PhysicalStartAddress   = 0;
static unsigned int Rt1PhysicalStartAddress    = 0;
static unsigned int Offload1PhysicalStartAddress = 0;
/* Variables to store the size of requested firmware */
/* ------------------------------------------------- */
static unsigned int Video1VirtualSizeMapped    = 0;
static unsigned int Audio1VirtualSizeMapped    = 0;
static unsigned int Video2VirtualSizeMapped    = 0;
static unsigned int Audio2VirtualSizeMapped    = 0;
static unsigned int Ecm1VirtualSizeMapped      = 0;
static unsigned int Rt1VirtualSizeMapped       = 0;
static unsigned int Offload1VirtualSizeMapped  = 0;
static unsigned int VirtualStartAddress        = 0;
static unsigned int VirtualMappedSize          = 0;
/* Driver variables */
/* ---------------- */
typedef enum
{
	VIDEO1_FIRMWARE,
	AUDIO1_FIRMWARE,
	VIDEO2_FIRMWARE,
	AUDIO2_FIRMWARE,
	ECM1_FIRMWARE,
	RT1_FIRMWARE,
	OFFLOAD1_FIRMWARE,
	FIRMWARE_NONE,
} FirmwareType_t;

static FirmwareType_t       FirmwareType = FIRMWARE_NONE;
static char                 FirmwareName[16];
static struct cdev         *stlxload_cdev = NULL;
static struct class_simple *stlxload_class = NULL;
static unsigned int         stlxload_major;
static unsigned long        NbStoredData;

/* Module informations */
/* ------------------- */
MODULE_AUTHOR      ("Marc Chappellier/Divneil Wadhawan");
MODULE_DESCRIPTION ("Lxloader STAPI module. This is responsible for loading firmware/binary on Lx/RT/ECM");
MODULE_LICENSE     ("GPL");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 12)
module_param       (type, charp, 0644);
#else
MODULE_PARM        (type, "s");
#endif
MODULE_PARM_DESC   (type, "video[1, 2] or audio[1, 2] or ecm1 firmware string flag");
module_init        (lxload_init);
module_exit        (lxload_exit);

/* File operation structure */
/* ------------------------ */
static struct file_operations lxload_fops =
{
	owner:   THIS_MODULE,
	open:    lxload_open,
	write:   lxload_write,
	release: lxload_close,
	ioctl:   lxload_ioctl
};

/* Registration information for lxload class interface */
/* --------------------------------------------------- */
static struct class lxload_class =
{
	.name = "lxloader",
	.owner = THIS_MODULE,
};

/* Specifying class attributes */
/* --------------------------- */
static struct class_attribute lxload_attribute =
{
	.attr =
	{
		.name  = "lxname",
		.owner = THIS_MODULE,
		.mode  = S_IWUSR|S_IRUGO,
	},
	.show  = lxload_sysfs_show,
	.store = lxload_sysfs_store,
};

/* From stsys.c */

/* ========================================================================
   Name:        stsys_OpenFile
   Description: Open a file 
   ======================================================================== */
static ST_ErrorCode_t stsys_OpenFile(void)
{
	ST_ErrorCode_t Error = ST_NO_ERROR;
	char *devpath;
  
	/* Open device */
	/* ----------- */
	if (fd == 0)
	{
		devpath = getenv("STSYS_IOCTL_DEV_PATH");
		if (devpath)
        {
			if ((fd = open(devpath, O_RDWR)) == -1)   /* Open device */
            {                
				perror(" Unable to open device !\n");
				perror(strerror(errno));
				return(1);
			}
		}
		else
		{
			perror(" Unable to find stsys device !\n");
			return(1);
		}
	}
	return Error;
}

/* ========================================================================
   Name:        STSYS_MapRegisters
   Description: Map physical registers into kernel space
   ======================================================================== */

unsigned int STSYS_MapRegisters(unsigned int pAddr, unsigned int Length,char *Name)
{
	STSYS_MapReg_t MapRegInfo;
  
	if (stsys_OpenFile() != ST_NO_ERROR)
	{
		return 0;
	}
	/* Fill up the map reg informations */
	/* -------------------------------- */
	memset(&MapRegInfo, 0, sizeof(STSYS_MapReg_t));
	MapRegInfo.PhysAddress = pAddr;
	MapRegInfo.Length      = Length;
	strncpy(MapRegInfo.Name, Name, STSYS_MAX_REG_NAME_LENGTH - 1);
	if (ioctl(fd, STSYS_IOC_MAP_REGISTERS, &MapRegInfo) != 0)
	{
		perror("--> STSYS_MapRegisters() : Unable to send map register command !\n");
		perror(strerror(errno));
		return(0);
	}    
 
	/* Return value */
	/* ------------ */
	return MapRegInfo.MappedAddress;
}

/* ========================================================================
   Name:        STSYS_UnmapRegisters
   Description: Unmap physical registers
   ======================================================================== */

void STSYS_UnmapRegisters(unsigned int vAddr,unsigned int Length)
{
	STSYS_MapReg_t MapRegInfo;
  
	if (stsys_OpenFile() != ST_NO_ERROR)
	{
		return;
	}
	/* Fill up the map reg informations */
	/* -------------------------------- */
	memset(&MapRegInfo,0,sizeof(STSYS_MapReg_t));
	MapRegInfo.MappedAddress = vAddr;
	MapRegInfo.Length        = Length; 
	if (ioctl(fd,STSYS_IOC_UNMAP_REGISTERS,&MapRegInfo)!=0)
	{
		perror("--> STSYS_UnmapRegisters() : Unable to send unmap register command !\n");
		perror(strerror(errno));
		return;
	}    
}

/* LXload */

/* ========================================================================
   Name:        lxload_open
   Description: Open the device
   ======================================================================== */

static int lxload_open(struct inode* inode, struct file* file )
{
	unsigned int   Value;
	void *CONFIG_BASE_ADDRESS_MAPPED, *CONFIG_BASE_ADDRESS_MAPPED2;

	/* Map the registers */
	/* ----------------- */
	CONFIG_BASE_ADDRESS_MAPPED = STOS_MapRegisters((void *)CFG_3_BASE_ADDRESS, 0x200, "SYS_CFG");
	if (CONFIG_BASE_ADDRESS_MAPPED == NULL)
	{
		return -1;
	}
	CONFIG_BASE_ADDRESS_MAPPED2 = STOS_MapRegisters((void *)CFG_1_BASE_ADDRESS, 0x100, "SYS_CFG");
	if (CONFIG_BASE_ADDRESS_MAPPED2 == NULL)
	{
		return -1;
	}

	/* Reset companions */
	/* ---------------- */
	if (FirmwareType == VIDEO1_FIRMWARE)
	{
		Value = (STSYS_ReadRegDev32LE(CONFIG_BASE_ADDRESS_MAPPED + 0x0F0 /*SYSTEM_CONFIG460*/)) & (~(1 << 27));
		STSYS_WriteRegDev32LE(CONFIG_BASE_ADDRESS_MAPPED + 0x0F0 /*SYSTEM_CONFIG460*/, Value);
		STSYS_WriteRegDev32LE(CONFIG_BASE_ADDRESS_MAPPED2 + 0x12C /*SYSTEM_CONFIG175*/, Video1PhysicalStartAddress);
	}
	else if (FirmwareType == AUDIO1_FIRMWARE)
	{
		Value = (STSYS_ReadRegDev32LE(CONFIG_BASE_ADDRESS_MAPPED + 0x0F0 /*SYSTEM_CONFIG460*/)) & (~(1 << 26));
		STSYS_WriteRegDev32LE(CONFIG_BASE_ADDRESS_MAPPED + 0x0F0 /*SYSTEM_CONFIG460*/, Value);
		STSYS_WriteRegDev32LE(CONFIG_BASE_ADDRESS_MAPPED2 + 0x130 /*SYSTEM_CONFIG176*/, Audio1PhysicalStartAddress);
	}

	/* Unmap the registers */
	/* ------------------- */
	STOS_UnmapRegisters(CONFIG_BASE_ADDRESS_MAPPED, 0x200);
	STOS_UnmapRegisters(CONFIG_BASE_ADDRESS_MAPPED2, 0x100);

	/* Set current pointer position for write operation */
	/* ------------------------------------------------ */
	NbStoredData = 0;

	/* Create virtual mappings for firmware loading */
	/* -------------------------------------------- */
	if (lxload_create_lxmapping() != ST_NO_ERROR)
	{
		LXLOADi_TraceERR((KERN_ERR "Unable to create virtual mappings of %s Lx load memory \n", FirmwareName));
		return -1;
	}
	/* Return no errors */
	/* ---------------- */
	return 0;
}

/* ========================================================================
   Name:        lxload_close
   Description: Close the device
   ======================================================================== */

static int lxload_close(struct inode *inode, struct file *file)
{
	/* For debug */
	/* --------- */
	LXLOADi_TraceDBG((KERN_INFO "%s(): major = %d minor = %d\n", __func__, MAJOR(inode->i_rdev), MINOR(inode->i_rdev)));

	
	void *CONFIG_BASE_ADDRESS_MAPPED;
	unsigned int   Value;

	/* Map the registers */
	/* ----------------- */
	CONFIG_BASE_ADDRESS_MAPPED = STOS_MapRegisters((void *)CFG_3_BASE_ADDRESS, 0x200, "SYS_CFG");
	if (CONFIG_BASE_ADDRESS_MAPPED == NULL)
	{
		return -1;
	}
	/* Boot the lx now */
	/* --------------- */
	if (FirmwareType == VIDEO1_FIRMWARE)
	{
		Value = (STSYS_ReadRegDev32LE(CONFIG_BASE_ADDRESS_MAPPED + 0x0F0 /*SYSTEM_CONFIG460*/)) | (1 << 27);
		STSYS_WriteRegDev32LE(CONFIG_BASE_ADDRESS_MAPPED + 0x0F0 /*SYSTEM_CONFIG460*/, Value);
	}
	else if (FirmwareType == AUDIO1_FIRMWARE)
	{
		Value = (STSYS_ReadRegDev32LE(CONFIG_BASE_ADDRESS_MAPPED + 0x0F0 /*SYSTEM_CONFIG460*/)) | (1 << 26);
		STSYS_WriteRegDev32LE(CONFIG_BASE_ADDRESS_MAPPED + 0x0F0 /*SYSTEM_CONFIG460*/, Value);
	}
	/* Unmap the registers */
	/* ------------------- */
	STOS_UnmapRegisters(CONFIG_BASE_ADDRESS_MAPPED, 0x200);

	if (FirmwareType == VIDEO1_FIRMWARE)
	{
		LXLOADi_TraceERR((KERN_INFO "LXLOAD(%s)\t: LX loaded => Base = 0x%08x - Last = 0x%08x - Size = %d\n", FirmwareName, (unsigned int)Video1PhysicalStartAddress, (unsigned int)(Video1PhysicalStartAddress+NbStoredData), (unsigned int)NbStoredData));
	}
	else if (FirmwareType == AUDIO1_FIRMWARE)
	{
		LXLOADi_TraceERR((KERN_INFO "LXLOAD(%s)\t: LX loaded => Base = 0x%08x - Last = 0x%08x - Size = %d\n", FirmwareName, (unsigned int)Audio1PhysicalStartAddress, (unsigned int)(Audio1PhysicalStartAddress+NbStoredData), (unsigned int)NbStoredData));
	}
	else if (FirmwareType == VIDEO2_FIRMWARE)
	{
		LXLOADi_TraceERR((KERN_INFO "LXLOAD(%s)\t: LX loaded => Base = 0x%08x - Last = 0x%08x - Size = %d\n", FirmwareName, (unsigned int)Video2PhysicalStartAddress, (unsigned int)(Video2PhysicalStartAddress+NbStoredData), (unsigned int)NbStoredData));
	}
	else if (FirmwareType == AUDIO2_FIRMWARE)
	{
		LXLOADi_TraceERR((KERN_INFO "LXLOAD(%s)\t: LX loaded => Base = 0x%08x - Last = 0x%08x - Size = %d\n", FirmwareName, (unsigned int)Audio2PhysicalStartAddress, (unsigned int)(Video2PhysicalStartAddress+NbStoredData), (unsigned int)NbStoredData));
	}
	else if (FirmwareType == ECM1_FIRMWARE)
	{
		LXLOADi_TraceERR((KERN_INFO "LXLOAD(%s)\t: LX loaded => Base = 0x%08x - Last = 0x%08x - Size = %d\n", FirmwareName, (unsigned int)Ecm1PhysicalStartAddress, (unsigned int)(Ecm1PhysicalStartAddress+NbStoredData), (unsigned int)NbStoredData));
	}
	else if (FirmwareType == RT1_FIRMWARE)
	{
		LXLOADi_TraceERR((KERN_INFO "LXLOAD(%s)\t: LX loaded => Base = 0x%08x - Last = 0x%08x - Size = %d\n", FirmwareName, (unsigned int)Rt1PhysicalStartAddress, (unsigned int)(Rt1PhysicalStartAddress+NbStoredData), (unsigned int)NbStoredData));
	}
	else if (FirmwareType == OFFLOAD1_FIRMWARE)
	{
		LXLOADi_TraceERR((KERN_INFO "LXLOAD(%s)\t: LX loaded => Base = 0x%08x - Last = 0x%08x - Size = %d\n", FirmwareName, (unsigned int)Offload1PhysicalStartAddress, (unsigned int)(Offload1PhysicalStartAddress+NbStoredData), (unsigned int)NbStoredData));
	}
	else
	{
		LXLOADi_TraceERR((KERN_INFO "LXLOAD(%s)\t: Invalid module configuration, cat /sys/class/lxloader/lxname for more info\n", FirmwareName));
	}
	/* Unmap region */
	/* ------------ */
	lxload_unmap_region((void *)VirtualStartAddress, VirtualMappedSize);

	/* Return no errors */
	/* ---------------- */
	return 0;
}

/* ========================================================================
   Name:        lxload_ioctl
   Description: Manage ioctl to the device
   ======================================================================== */

static int lxload_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	/* There are no IOCTL commands supported here */
	/* ------------------------------------------ */
	return -ENOTTY;
}

/* ========================================================================
   Name:        lxload_write
   Description: Manage write to the device
   ======================================================================== */

static ssize_t lxload_write(struct file * file, const char *buffer, size_t length, loff_t *offset)
{
	if ((VirtualStartAddress == (unsigned int)NULL)
	||  (length == 0))
	{
		return -1;
	}
	/* Copy the code from user space to companion memory */
	/* ------------------------------------------------- */
	if (copy_from_user((void *)(VirtualStartAddress+NbStoredData), buffer, length) != 0)
	{
		LXLOADi_TraceERR((KERN_ERR "%s(): Unable to write companion code !\n", __func__));
		return 0;
	}
	/* Increase current pointer position, the write could be called many times */
	/* ----------------------------------------------------------------------- */
	NbStoredData += length;

	/* Return size written */
	/* ------------------- */
	return length;
}

/* ==========================================================================
   Name:        lxload_parse_type_pos
   Description: This is a help routine to parse the argument passed to insmod
   ========================================================================== */

static int lxload_parse_type_pos(char *str, char *pattern)
{
	unsigned int pos = 0;

	/* Check the parameter sanity to exit the loop */
	/* ------------------------------------------- */
	if (!str || (*str == '\0'))
	{
		return 0xFFFFFFFF;
	}
	/* Calculate the position of the pattern and return */
	/* ------------------------------------------------ */
	while(*str++ != *pattern)
	{
		pos++;
		if (*str == '\0')
		{
			return(pos);
		}
	}
	return pos;
}

/* ========================================================================
   Name:        lxload_init
   Description: This is the module initialization function which is
                responsible for parsing the command line argument and
                determine which firmwares will be loaded. It's a character
                driver, so, that initialization is there in place.
   ======================================================================== */

static int __init lxload_init(void)
{
	unsigned int  pos = 0;

	/* For debug */
	/* --------- */
	LXLOADi_TraceDBG((KERN_INFO "%s(): Initializing LXLOAD kernel Module\n", __func__));

	/* Check string companion type */
	/* --------------------------- */
	if (type != NULL)
	{
		do
		{
			pos = lxload_parse_type_pos(type, ",");

			if (!strncmp(type, "video1", pos))
			{
				LXLOADi_TraceDBG((KERN_INFO "%s(): Insert module for video1 firmware\n", __func__));
				VideoFirmware1 = TRUE;
			}
			if (!strncmp(type, "audio1", pos))
			{
				LXLOADi_TraceDBG((KERN_INFO "%s(): Insert module for audio1 firmware\n", __func__));
				AudioFirmware1 = TRUE;
			}
			if (!strncmp(type, "video2", pos))
			{
				LXLOADi_TraceDBG((KERN_INFO "%s(): Insert module for video2 firmware\n", __func__));
				VideoFirmware2 = TRUE;
			}
			if (!strncmp(type, "audio2", pos))
			{
				LXLOADi_TraceDBG((KERN_INFO "%s(): Insert module for audio2 firmware\n", __func__));
				AudioFirmware2 = TRUE;
			}
			if (!strncmp(type, "offload1", pos))
			{
				LXLOADi_TraceDBG((KERN_INFO "%s(): Insert module for offload1 firmware\n", __func__));
				OffloadFirmware1 = TRUE;
			}
			if (!strncmp(type, "ecm1", pos))
			{
				LXLOADi_TraceDBG((KERN_INFO "%s(): Insert module for ecm1 firmware\n", __func__));
				EcmFirmware1 = TRUE;
			}
			if (!strncmp(type, "rt1", pos))
			{
				LXLOADi_TraceDBG((KERN_INFO "%s(): Insert module for rt1 firmware\n", __func__));
				RtFirmware1 = TRUE;
			}
		}
		while ((pos != 0xFFFFFFFF) && (type[pos] != '\0') && (type += (pos + 1)));
	}

	/* If nothing detected, return an error */
	/* ------------------------------------ */
	if ((VideoFirmware1 == FALSE)
	&&  (AudioFirmware1 == FALSE)
	&&  (VideoFirmware2 == FALSE)
	&&  (AudioFirmware2 == FALSE)
	&&  (EcmFirmware1 == FALSE)
	&&  (RtFirmware1 == FALSE)
	&&  (OffloadFirmware1 == FALSE))
	{
		LXLOADi_TraceERR((KERN_ERR "%s(): Unable to identify firmware type (video1 or video2 or audio1 or audio2 or ecm1 or rt1) !\n", __func__));
		return -1;
	}

	/* Register the device with the kernel */
	/* ----------------------------------- */
	stlxload_major = LXLOAD_IOCTL_MAJOR;
	if (lxload_device_register(&lxload_fops, 1, LXLOAD_LINUX_DEVICE_NAME, &stlxload_major, &stlxload_cdev, &stlxload_class) != 0)
	{
		LXLOADi_TraceERR((KERN_ERR "%s(): Unable to register this device with the kernel !\n", __func__));
		return -1;
	}

	/* Register the regions of the requested firmware with the module */
	/* -------------------------------------------------------------- */
#if defined(VIDEO1_COMPANION_OFFSET)
	if (VideoFirmware1 == TRUE)
	{
		Video1PhysicalStartAddress = RAM1_BASE_ADDRESS + (unsigned int)VIDEO1_COMPANION_OFFSET;
		Video1VirtualSizeMapped = VIDEO1_COMPANION_SIZE;
	}
#endif
#if defined(AUDIO1_COMPANION_OFFSET)
	if (AudioFirmware1 == TRUE)
	{
		Audio1PhysicalStartAddress = RAM1_BASE_ADDRESS + (unsigned int)AUDIO1_COMPANION_OFFSET;
		Audio1VirtualSizeMapped = AUDIO1_COMPANION_SIZE;
	}
#endif
#if defined(VIDEO2_COMPANION_OFFSET)
	if (VideoFirmware2 == TRUE)
	{
		Video2PhysicalStartAddress = RAM1_BASE_ADDRESS + (unsigned int)VIDEO2_COMPANION_OFFSET;
		Video2VirtualSizeMapped = VIDEO2_COMPANION_SIZE;
	}
#endif
#if defined(AUDIO2_COMPANION_OFFSET)
	if (AudioFirmware2 == TRUE)
	{
		Audio2PhysicalStartAddress = RAM1_BASE_ADDRESS + (unsigned int)AUDIO2_COMPANION_OFFSET;
		Audio2VirtualSizeMapped = AUDIO2_COMPANION_SIZE;
	}
#endif
#if defined(OFFLOAD_COMPANION_OFFSET)
	if (OffloadFirmware1 == TRUE)
	{
		Offload1PhysicalStartAddress = RAM1_BASE_ADDRESS + (unsigned int)OFFLOAD_COMPANION_OFFSET;
		Offload1VirtualSizeMapped = OFFLOAD_COMPANION_SIZE;
	}
#endif
#if defined(ECM_START_ADDRESS)
	if (EcmFirmware1 == TRUE)
	{
		Ecm1PhysicalStartAddress = ECM_START_ADDRESS;
		Ecm1VirtualSizeMapped = ECM_SIZE;
	}
#endif
#if defined(RT_BASE_ADDRESS)
	if (RtFirmware1 == TRUE)
	{
		Rt1PhysicalStartAddress = RT_BASE_ADDRESS;
		Rt1VirtualSizeMapped = RT_SIZE;
	}
#endif
	if (!Video1VirtualSizeMapped && !Audio1VirtualSizeMapped && !Video2VirtualSizeMapped && !Audio2VirtualSizeMapped && !Offload1VirtualSizeMapped && !Ecm1VirtualSizeMapped && !Rt1VirtualSizeMapped)
	{
		LXLOADi_TraceERR((KERN_ERR "%s(): Unable to find companion size to be mapped, problem is in the platform definition !\n", __func__));
		lxload_device_unregister(1, LXLOAD_LINUX_DEVICE_NAME, stlxload_major, stlxload_cdev, stlxload_class);
		return -1;
	}
	/* Register lxloader under class subsystem */
	/*  ======================================= */
	if (class_register(&lxload_class))
	{
		LXLOADi_TraceERR((KERN_ERR "%s(): Unable to create directory entry on sysfs !\n", __func__));
		lxload_device_unregister(1, LXLOAD_LINUX_DEVICE_NAME, stlxload_major, stlxload_cdev, stlxload_class);
		return -1;
	}
	/* Create attributes for storing firmware loading request */
	/*  ====================================================== */
	if (class_create_file(&lxload_class, &lxload_attribute))
	{
		LXLOADi_TraceERR((KERN_ERR "%s(): Unable to create lxload attribute !\n", __func__));
		lxload_device_unregister(1, LXLOAD_LINUX_DEVICE_NAME, stlxload_major, stlxload_cdev, stlxload_class);
		class_unregister(&lxload_class);
		return -1;
	}
	/* Return no errors */
	/* ---------------- */
	return 0;
}

/* ========================================================================
   Name:        lxload_exit
   Description: Terminate lxloader
   ======================================================================== */

static void __exit lxload_exit(void)
{
	/* For debug only */
	/* -------------- */
	LXLOADi_TraceDBG((KERN_INFO "%s(): Exiting lxloader kernel module\n", __func__));

	/* Delete lxloader sysfs attribute */
	/* ------------------------------- */
	class_remove_file(&lxload_class, &lxload_attribute);

	/* Delete lxloader sysfs entry */
	/* --------------------------- */
	class_unregister(&lxload_class);

	/* Unregister device */
	/* ----------------- */
	lxload_device_unregister(1, LXLOAD_LINUX_DEVICE_NAME, stlxload_major, stlxload_cdev, stlxload_class);
}

/* ========================================================================
   Name:        lxload_sysfs_show
   Description: This displays, how to configure this module for firmware
                loading.
   ======================================================================== */

ssize_t lxload_sysfs_show(struct class *class, char *buffer)
{
	int length = 0;

	/* Display the usage of this sysfs entry to user-space */
	/* --------------------------------------------------- */
	length = sprintf(buffer, "This module allows to configure the load of Audio/Video/RT/ECM/Offload firmwares\n");
	length  += sprintf(buffer + length + 1, "The following is used for configuration and follow this with firmware loading\n");
	length  += sprintf(buffer + length + 1, " 1. echo \"video1\"   > /sys/class/lxloader/lxname\n");
	length  += sprintf(buffer + length + 1, " 2. echo \"audio1\"   > /sys/class/lxloader/lxname\n");
	length  += sprintf(buffer + length + 1, " 3. echo \"video2\"   > /sys/class/lxloader/lxname\n");
	length  += sprintf(buffer + length + 1, " 4. echo \"audio2\"   > /sys/class/lxloader/lxname\n");
	length  += sprintf(buffer + length + 1, " 5. echo \"ecm1\"     > /sys/class/lxloader/lxname\n");
	length  += sprintf(buffer + length + 1, " 6. echo \"rt1\"      > /sys/class/lxloader/lxname\n");
	length  += sprintf(buffer + length + 1, " 7. echo \"offload1\" > /sys/class/lxloader/lxname\n\n");
	return length;
}

/* ===========================================================================
   Name:        lxload_sysfs_store
   Description: This function is direct interface to user space and configures
                the module for a particular firmware load based on user input.
   =========================================================================== */

ssize_t lxload_sysfs_store(struct class *class, const char *buffer, size_t size)
{
	int error = size;

	/* Check which firmware loading is requested */
	/* ----------------------------------------- */
	if (!strncmp("video1", buffer, strlen("video1"))
	&&  (VideoFirmware1 == TRUE))
	{
		FirmwareType = VIDEO1_FIRMWARE;
		strlcpy(FirmwareName, "video1", sizeof(FirmwareName));
	}
	else if (!strncmp("audio1", buffer, strlen("audio1"))
	&&       (AudioFirmware1 == TRUE))
	{
		FirmwareType = AUDIO1_FIRMWARE;
		strlcpy(FirmwareName, "audio1", sizeof(FirmwareName));
	}
	else if (!strncmp("video2", buffer, strlen("video2"))
	&&       (VideoFirmware2 == TRUE))
	{
		FirmwareType = VIDEO2_FIRMWARE;
		strlcpy(FirmwareName, "video2", sizeof(FirmwareName));
	}
	else if (!strncmp("audio2", buffer, strlen("audio2"))
	&&       (AudioFirmware2 == TRUE))
	{
		FirmwareType = AUDIO2_FIRMWARE;
		strlcpy(FirmwareName, "audio2", sizeof(FirmwareName));
	}
	else if (!strncmp("ecm1", buffer, strlen("ecm1"))
	&&       (EcmFirmware1 == TRUE))
	{
		FirmwareType = ECM1_FIRMWARE;
		strlcpy(FirmwareName, "ecm1", sizeof(FirmwareName));
	}
	else if (!strncmp("rt1", buffer, strlen("rt1"))
	&&       (RtFirmware1 == TRUE))
	{
		FirmwareType = RT1_FIRMWARE;
		strlcpy(FirmwareName, "rt1", sizeof(FirmwareName));
	}
	else if (!strncmp("offload1", buffer, strlen("offload1"))
	&&       (OffloadFirmware1 == TRUE))
	{
		FirmwareType = OFFLOAD1_FIRMWARE;
		strlcpy(FirmwareName, "offload1", sizeof(FirmwareName));
	}
	else
	{
		FirmwareType = FIRMWARE_NONE;
	}
	/* If no firmware detected */
	/* ----------------------- */
	if (FirmwareType == FIRMWARE_NONE)
	{
		LXLOADi_TraceERR((KERN_ERR "Unable to configure for firmware loading, invalid firmware %s \n", buffer));
		error = -1;
	}
	return error;
}

/* ===========================================================================
   Name:        lxload_create_lxmapping
   Description: Create virtual mapping for
   =========================================================================== */

static ST_ErrorCode_t lxload_create_lxmapping()
{
	switch (FirmwareType)
	{
		case VIDEO1_FIRMWARE:
		{
			if (Video1VirtualSizeMapped != 0)
			{
				VirtualStartAddress = (unsigned int)lxload_map_region((void *)Video1PhysicalStartAddress, Video1VirtualSizeMapped, "LXLOADER_VIDEO1");
			}
			VirtualMappedSize = Video1VirtualSizeMapped;
			break;
		}
		case AUDIO1_FIRMWARE:
		{
			if (Audio1VirtualSizeMapped != 0)
			{
				VirtualStartAddress = (unsigned int)lxload_map_region((void *)Audio1PhysicalStartAddress, Audio1VirtualSizeMapped, "LXLOADER_AUDIO1");
			}
			VirtualMappedSize = Audio1VirtualSizeMapped;
			break;
		}
		case VIDEO2_FIRMWARE:
		{
			if (Video2VirtualSizeMapped != 0)
			{
				VirtualStartAddress = (unsigned int)lxload_map_region((void *)Video2PhysicalStartAddress, Video2VirtualSizeMapped, "LXLOADER_VIDEO2");
			}
			VirtualMappedSize = Video2VirtualSizeMapped;
			break;
		}
		case AUDIO2_FIRMWARE:
		{
			if (Audio2VirtualSizeMapped != 0)
			{
				VirtualStartAddress = (unsigned int)lxload_map_region((void *)Audio2PhysicalStartAddress, Audio2VirtualSizeMapped, "LXLOADER_AUDIO2");
			}
			VirtualMappedSize = Audio2VirtualSizeMapped;
			break;
		}
		case ECM1_FIRMWARE:
		{
			if (Ecm1VirtualSizeMapped != 0)
			{
				VirtualStartAddress = (unsigned int)lxload_map_region((void *)Ecm1PhysicalStartAddress, Ecm1VirtualSizeMapped, "LXLOADER_ECM1");
			}
			VirtualMappedSize = Ecm1VirtualSizeMapped;
			break;
		}
		case RT1_FIRMWARE:
		{
			if (Rt1VirtualSizeMapped != 0)
			{
				VirtualStartAddress = (unsigned int)lxload_map_region((void *)Rt1PhysicalStartAddress, Rt1VirtualSizeMapped, "LXLOADER_RT1");
			}
			VirtualMappedSize = Rt1VirtualSizeMapped;
			break;
		}
		case OFFLOAD1_FIRMWARE:
		{
			if (Offload1VirtualSizeMapped != 0)
			{
				VirtualStartAddress = (unsigned int)lxload_map_region((void *)Offload1PhysicalStartAddress, Offload1VirtualSizeMapped, "LXLOADER_Offload1");
			}
			VirtualMappedSize = Offload1VirtualSizeMapped;
			break;
		}
		default:
		{
			LXLOADi_TraceERR((KERN_ERR "%s(): Invalid firmware type, DO: $cat /sys/class/lxloader/lxname !\n", __func__));
			VirtualStartAddress = (unsigned int)NULL;
			VirtualMappedSize  = 0;
		}
	}
	if ((VirtualStartAddress == (unsigned int)NULL)
	||  (VirtualMappedSize == 0))
	{
		LXLOADi_TraceERR((KERN_ERR "%s(): Unable to map companion region !\n", __func__));
		return -1;
	}
	/* For debug only */
	/* -------------- */
	LXLOADi_TraceDBG((KERN_INFO "%s(): LXLOADER virtual io kernel address 0x%x is mapped with size 0x%x\n", __func__, VirtualStartAddress, VirtualMappedSize));


	/* This memset clean-up the region of the firmware                       */
	/* This is needed to fix a unstability seen with MPEG4P2 codec firmwares */
	memset((void *)VirtualStartAddress, 0, VirtualMappedSize);

	/* Return error code */
	/* ----------------- */
	return 0;
}

/* ========================================================================
   Name:        lxload_device_register
   Description: Register the device to the kernel
   ======================================================================== */

static int lxload_device_register(struct file_operations *stdevice_fops, unsigned int nbdevices, char *device_name, unsigned int *device_major, struct cdev **stdevice_cdev, struct class_simple **stdevice_class)
{
	struct class_device *class_err;
	int                  entries;
	dev_t                base_dev_no;
	char                *extended_name;

	/* Allocate a string device name */
	/* ----------------------------- */
	extended_name = (char *)vmalloc(strlen(device_name) + 11); /* +11 equals max place for unsigned int number + \0 */

	/* Set module owner */
	/* ---------------- */
	SET_MODULE_OWNER(stdevice_fops);

	/* Register the major number of the device */
	/* --------------------------------------- */
	/* Register the major number. If major = 0 then a major number is auto */
	/* allocated. The allocated number is returned.                        */
	/* The major number can be seen in user space in '/proc/devices'       */
	if (*device_major == 0)
	{
		if (alloc_chrdev_region(&base_dev_no, 0, nbdevices, device_name))
		{
			LXLOADi_TraceERR((KERN_ALERT "%s(): No major numbers for %s by %s (pid %i)\n", __func__, device_name, current->comm, current->pid));
			vfree(extended_name);
			return -EBUSY;
		}
		*device_major = MAJOR(base_dev_no);
	}
	else
	{
		base_dev_no = MKDEV(*device_major, 0);
		if (register_chrdev_region(base_dev_no, nbdevices, device_name))
		{
			LXLOADi_TraceERR((KERN_ALERT "%s(): No major numbers for %s by %s (pid %i)\n", __func__, device_name, current->comm, current->pid));
			vfree(extended_name);
			return -EBUSY;
		}
	}

	/* Allocate a device */
	/* ----------------- */
	if ((*stdevice_cdev = cdev_alloc()) == NULL)
	{
		LXLOADi_TraceERR((KERN_ALERT "%s(): No major numbers for %s by %s (pid %i)\n", __func__, device_name, current->comm, current->pid));
		unregister_chrdev_region(MKDEV(*device_major, 0), nbdevices);
		vfree(extended_name);
		return -EBUSY;
	}

	/* Set device name */
	/* --------------- */
	sprintf(extended_name, "%s%d", device_name, 0);
	(*stdevice_cdev)->owner = THIS_MODULE;
	(*stdevice_cdev)->ops  = stdevice_fops;
	kobject_set_name(&((*stdevice_cdev)->kobj), extended_name);

	/* Add the char device structure for this module */
	/* --------------------------------------------- */
	if (cdev_add(*stdevice_cdev, base_dev_no, nbdevices))
	{
		LXLOADi_TraceERR((KERN_ALERT "%s(): Failed to add a charactor device for %s by %s (pid %i)\n", __func__, device_name, current->comm, current->pid));
		kobject_put(&((*stdevice_cdev)->kobj));
		unregister_chrdev_region(MKDEV(*device_major, 0), nbdevices);
		vfree(extended_name);
		return -EBUSY;
	}

	/* Create a class for this module */
	/* ------------------------------ */
	*stdevice_class = class_simple_create(THIS_MODULE, device_name);
	if (IS_ERR(*stdevice_class))
	{
		LXLOADi_TraceERR((KERN_ERR "%s(): Error creating %s class\n", __func__, device_name));
		cdev_del(*stdevice_cdev);
		unregister_chrdev_region(MKDEV(*device_major, 0), nbdevices);
		vfree(extended_name);
		return -EBUSY;
	}
	/* Add entries into /sys for each minor number we use */
	/* -------------------------------------------------- */
	for (entries = 0; entries < nbdevices; entries++)
	{
		sprintf(extended_name, "%s%d", device_name, entries);
		class_err = class_simple_device_add(*stdevice_class, MKDEV(*device_major, entries), NULL, extended_name);
		if (IS_ERR(class_err))
		{
			LXLOADi_TraceERR((KERN_ERR "%s(): Error creating %s device %d\n", __func__, device_name, entries));
			for (entries--;(entries >= 0);entries--) class_simple_device_remove(MKDEV(*device_major, entries));
			class_simple_destroy(*stdevice_class);
			cdev_del(*stdevice_cdev);
			unregister_chrdev_region(MKDEV(*device_major, 0), nbdevices);
			vfree(extended_name);
			return PTR_ERR(class_err);
		}
	}
	/* For debug */
	/* --------- */
	LXLOADi_TraceDBG((KERN_INFO "%s(): Load module %s by %s (pid %i)\n", __func__, device_name, current->comm, current->pid));

	/* Return no errors */
	/* ---------------- */
	vfree(extended_name);
	return 0;
}

/* ========================================================================
   Name:        lxload_device_unregister
   Description: Unregister the device to the kernel
   ======================================================================== */

static int lxload_device_unregister(unsigned int nbdevices, char *device_name, unsigned int device_major, struct cdev *stdevice_cdev, struct class_simple *stdevice_class)
{
	unsigned int i;

	/* Remove any devices */
	/* ------------------ */
	for (i = 0; i < nbdevices; i++)
	{
		class_simple_device_remove(MKDEV(device_major, i));
	}

	/* Remove the device class entry */
	/* ----------------------------- */
	class_simple_destroy(stdevice_class);

	/* Remove the char device structure (has been added) */
	/* ------------------------------------------------- */
	cdev_del(stdevice_cdev);

	/* Unregister the module */
	/* --------------------- */
	unregister_chrdev_region(MKDEV(device_major, 0), nbdevices);

	/* For debug */
	/* --------- */
	LXLOADi_TraceDBG((KERN_INFO "%s(): Unload module %s by %s (pid %i)\n", __func__, device_name, current->comm, current->pid));

	/* Return no errors */
	/* ---------------- */
	return 0;
}

/*  ========================================================================
   Name:        lxload_map_region
   Description: Map a memory region
    ======================================================================== */

static void *lxload_map_region(void *Address_p, unsigned int Width, char *RegionName)
{
	void *MappedAddress_p = NULL;

	/* Check if address is NULL ! */
	/* -------------------------- */
	if (Address_p == NULL)
	{
		LXLOADi_TraceERR((KERN_ERR "%s(): Failed to remap region (%s) at 0x%08x (lg:0x%08x) !!!\n", __func__, RegionName, (unsigned int)Address_p, Width));
		return NULL;
	}
	/* Request the region */
	/* ------------------ */
	if (request_mem_region(((unsigned int)Address_p), Width, RegionName) == NULL)
	{
#if 0
		LXLOADi_TraceERR((KERN_ERR "%s(): Could not request region (%s) busy at %x lg %x !\n", __func__, RegionName, (unsigned int)Address_p, Width));
#endif
	}
	/* Now that we have control over the memory region, remap it to */
	/* something safe for us to use in in writel/readl accesses     */
	/* ------------------------------------------------------------ */
	MappedAddress_p = (void *)ioremap_nocache((unsigned int)Address_p, Width);
	if (MappedAddress_p == NULL)
	{
		release_mem_region((unsigned int)Address_p, Width);
		LXLOADi_TraceERR((KERN_ERR "%s(): Failed to remap region (%s) at 0x%08x (lg:0x%08x) !\n", __func__, RegionName, (unsigned int)Address_p, Width));
		return NULL;
	}

	/* Return no errors */
	/* ---------------- */
	return MappedAddress_p;
}

/* ========================================================================
   Name:        lxload_unmap_region
   Description: Unmap a memory region
   ======================================================================== */

static void lxload_unmap_region(void *MappedAddress_p, unsigned int Width)
{
	/* Check if address is NULL ! */
	/* -------------------------- */
	if (MappedAddress_p == NULL)
	{
		LXLOADi_TraceERR((KERN_ERR "%s(): Failed to unmap region 0x%08x (lg:0x%08x) !\n", __func__, (unsigned int)MappedAddress_p, Width));
		return;
	}
	/* If the memory region is used, release it */
	/* ---------------------------------------- */
	if (check_mem_region((unsigned int)MappedAddress_p, Width) < 0)
	{
		release_mem_region((unsigned int)MappedAddress_p, Width);
		LXLOADi_TraceDBG((KERN_INFO "%s(): The I/O memory region 0x%08x is released successfully\n", __func__, (unsigned int)MappedAddress_p));
	}
	else
	{
		LXLOADi_TraceDBG((KERN_INFO "%s(): 0x%08x non existing region... !\n", __func__, (unsigned int)MappedAddress_p));
	}
	/* Unmap the region */
	/* ---------------- */
	iounmap(MappedAddress_p);
}
// vim:ts=4
