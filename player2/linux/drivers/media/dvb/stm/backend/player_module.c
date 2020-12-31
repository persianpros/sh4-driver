/************************************************************************
Copyright (C) 2005 STMicroelectronics. All Rights Reserved.

This file is part of the Player2 Library.

Player2 is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License version 2 as published by the
Free Software Foundation.

Player2 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with player2; see the file COPYING. If not, write to the Free Software
Foundation, 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

The Player2 Library may alternatively be licensed under a proprietary
license from ST.

Source file name : player_module.c
Author : Julian

Implementation of the LinuxDVB interface to the DVB streamer

Date Modification Name
---- ------------ --------
24-Mar-05 Created Julian

************************************************************************/

#include <linux/sched.h>
#include <linux/syscalls.h>
#include <linux/module.h>

#include "report.h"
#include "display.h"
#include "player_module.h"
#include "backend_ops.h"
#include "player_interface_ops.h"
#include "alsa_backend_ops.h"
#include "player_backend.h"

static int __init PlayerLoadModule(void);
static void __exit PlayerUnloadModule(void);

module_init(PlayerLoadModule);
module_exit(PlayerUnloadModule);

MODULE_DESCRIPTION("Player2 backend implementation for STM streaming architecture.");
MODULE_AUTHOR("Julian Wilson");
MODULE_LICENSE("GPL");

#ifdef __TDT__
int noaudiosync = 0;
module_param(noaudiosync, int, 0444);
MODULE_PARM_DESC(noaudiosync, "Workaround, if there are problems with audio dropouts set it to 1.");

int discardlateframe = 2;
module_param(discardlateframe, int, 0444);
MODULE_PARM_DESC(discardlateframe, "Set start value for Discard Late Frame 0=never, 1=aftersync, 2=always");

int useoldaudiofw = 0;
module_param(useoldaudiofw, int, 0444);
MODULE_PARM_DESC(useoldaudiofw, "Set to 1 if you will use old audio firmware (audio.elf)");
#endif

#if defined(QBOXHD) || defined(QBOXHD_MINI)

#include <linux/proc_fs.h>	/* Necessary because we use the proc fs */
#include <asm/uaccess.h>	/* for copy_from_user */

#define PROCFS_MAX_SIZE		3
#define PROCFS_NAME 		"disable_time_ctrl"

static struct proc_dir_entry *disable_time_ctrl_proc;
static char procfs_buffer[PROCFS_MAX_SIZE];
static unsigned long procfs_buffer_size = PROCFS_MAX_SIZE;

int disable_time_ctrl=0;
module_param(disable_time_ctrl, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC (disable_time_ctrl," disable time-scontrol patch");

int debugReport = 0;
module_param (debugReport,int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC (debugReport," enable Player2 report system");

#define PLAYER2_VERSION       "0.0.6"
MODULE_VERSION(PLAYER2_VERSION);

int procfile_read(char *buffer, char **buffer_location,
				  off_t offset, int buffer_length, int *eof, void *data)
{
	int ret;

// 	printk(KERN_INFO "procfile_read (/proc/%s) called offset=%d, buffer_length=%d\n", PROCFS_NAME,offset,buffer_length);

	if (offset > 0)
	{
		/* we have finished to read, return 0 */
		ret  = 0;
	}
	else
	{
		procfs_buffer[0]=(0x30+(disable_time_ctrl?1:0)); //set buffer to module's parameter value
		procfs_buffer[1]='\n';
		procfs_buffer[2]='\0';

		/* fill the buffer, return the buffer size */
		memcpy(buffer, procfs_buffer, procfs_buffer_size);
		ret = procfs_buffer_size;
	}

// 	printk(KERN_INFO "procfile_read disable_time_ctrl =%d procfs_buffer[0]=%d ret=%d\n",disable_time_ctrl,procfs_buffer[0],ret);
	return ret;
}

/**
 * This function is called with the /proc file is written
 *
 */
int procfile_write(struct file *file, const char *buffer, unsigned long count,
				   void *data)
{
	printk(KERN_INFO "procfile_write (/proc/%s) called\n\n", PROCFS_NAME);

	/* get buffer size */
	procfs_buffer_size = count;
	if (procfs_buffer_size > PROCFS_MAX_SIZE )
	{
		procfs_buffer_size = PROCFS_MAX_SIZE;
	}

	/* write data to the buffer */
	if ( copy_from_user(procfs_buffer, buffer, procfs_buffer_size) ) {
		return -EFAULT;
	}

	if(procfs_buffer[0]==0x30)
		disable_time_ctrl = 0;
	else
	{
		disable_time_ctrl = 1;
		procfs_buffer[0]  = 0x31;
	}

	printk(KERN_INFO "procfile_write => _setTimeCtrl -\n");
	_setTimeCtrl(disable_time_ctrl==1);
	printk(KERN_INFO "procfile_write => _setTimeCtrl +\n");

// 	printk(KERN_INFO "procfile_write => set disable_time_ctrl =%d procfs_buffer[0]=%d\n", disable_time_ctrl,procfs_buffer[0]);
	return count; //just one run !!!
}
#endif

static struct dvb_backend_operations DvbBackendOps =
{
	.owner = THIS_MODULE,

	.playback_create = PlaybackCreate,
	.playback_delete = PlaybackDelete,
	.playback_add_demux = PlaybackAddDemux,
	.playback_remove_demux = PlaybackRemoveDemux,
	.playback_add_stream = PlaybackAddStream,
	.playback_remove_stream = PlaybackRemoveStream,
	.playback_get_speed = PlaybackGetSpeed,
	.playback_set_speed = PlaybackSetSpeed,
	.playback_set_native_playback_time = PlaybackSetNativePlaybackTime,
	.playback_set_option = PlaybackSetOption,
	.playback_get_player_environment = PlaybackGetPlayerEnvironment,
	.playback_set_clock_data_point = PlaybackSetClockDataPoint,

	.demux_inject_data = DemuxInjectData,

	.stream_inject_data = StreamInjectData,
	.stream_inject_data_packet = StreamInjectDataPacket,
	.stream_discontinuity = StreamDiscontinuity,
	.stream_drain = StreamDrain,
	.stream_check_drained = StreamCheckDrained,
	.stream_enable = StreamEnable,
	.stream_set_id = StreamSetId,
	.stream_channel_select = StreamChannelSelect,
	.stream_set_option = StreamSetOption,
	.stream_get_option = StreamGetOption,
	.stream_step = StreamStep,
	.stream_set_alarm = StreamSetAlarm,
	.stream_get_play_info = StreamGetPlayInfo,
	.stream_switch = StreamSwitch,
	.stream_get_decode_buffer = StreamGetDecodeBuffer,
	.stream_return_decode_buffer = StreamReturnDecodeBuffer,
	.stream_set_output_window = StreamSetOutputWindow,
	.stream_get_output_window = StreamGetOutputWindow,
	.stream_set_input_window = StreamSetInputWindow,
	.stream_set_play_interval = StreamSetPlayInterval,
	.stream_get_decode_buffer_pool_status = StreamGetDecodeBufferPoolStatus,
	.stream_get_player_environment = StreamGetPlayerEnvironment,
	.stream_register_event_signal_callback = StreamRegisterEventSignalCallback,

	.display_create = DisplayCreate,
	.display_delete = DisplayDelete,
	.display_synchronize = DisplaySynchronize
#ifdef __TDT__
	, .is_display_created = isDisplayCreated
#endif

};

#ifndef __TDT__
static struct player_interface_operations PlayerInterfaceOps =
{
	.owner = THIS_MODULE,

	.component_get_attribute = ComponentGetAttribute,
	.component_set_attribute = ComponentSetAttribute,
	.player_register_event_signal_callback = PlayerRegisterEventSignalCallback
};
#endif

static struct alsa_backend_operations AlsaBackendOps =
{
	.owner = THIS_MODULE,

	.mixer_get_instance = MixerGetInstance,
	.mixer_set_module_parameters = ComponentSetModuleParameters,

	.mixer_alloc_substream = MixerAllocSubStream,
	.mixer_free_substream = MixerFreeSubStream,
	.mixer_setup_substream = MixerSetupSubStream,
	.mixer_prepare_substream = MixerPrepareSubStream,
	.mixer_start_substream = MixerStartSubStream,
	.mixer_stop_substream = MixerStopSubStream,
};

extern void SysfsInit(void);

#ifndef __TDT__
#define CONFIG_EXPORT_PLAYER_INTERFACE
#endif
static int __init PlayerLoadModule(void)
{
	report_init();
	report_restricted_severity_levels(severity_fatal, severity_interrupt * 10);
	DisplayInit();
	BackendInit();
	register_dvb_backend(MODULE_NAME, &DvbBackendOps);
	register_alsa_backend(MODULE_NAME, &AlsaBackendOps);
#if defined (CONFIG_EXPORT_PLAYER_INTERFACE)
	register_player_interface(MODULE_NAME, &PlayerInterfaceOps);
#endif
#if defined(QBOXHD) || defined(QBOXHD_MINI)
	{
		/* create the /proc file */
		disable_time_ctrl_proc = create_proc_entry(PROCFS_NAME, 0644, NULL);

		if (disable_time_ctrl_proc == NULL) {
			remove_proc_entry(PROCFS_NAME, &proc_root);
			printk(KERN_ALERT "Error: Could not initialize /proc/%s\n",
				   PROCFS_NAME);
			return -ENOMEM;
		}

		disable_time_ctrl_proc->read_proc  = procfile_read;
		disable_time_ctrl_proc->write_proc = procfile_write;
		disable_time_ctrl_proc->owner 	  = THIS_MODULE;
		disable_time_ctrl_proc->mode 	  = S_IFREG | S_IRUGO | S_IWUGO;
		disable_time_ctrl_proc->uid 	  = 0;
		disable_time_ctrl_proc->gid 	  = 0;
		disable_time_ctrl_proc->size 	  = 37;

		printk(KERN_INFO "/proc/%s created\n", PROCFS_NAME);
		return 0;	/* everything is ok */
	}

#endif
	PLAYER_DEBUG("Player2 backend loaded\n");
	return 0;
}

static void __exit PlayerUnloadModule(void)
{
	BackendDelete();
#if defined(QBOXHD) || defined(QBOXHD_MINI)
	remove_proc_entry(PROCFS_NAME, &proc_root);
#endif
	PLAYER_DEBUG("Player2 backend unloaded\n");
	return;
}

