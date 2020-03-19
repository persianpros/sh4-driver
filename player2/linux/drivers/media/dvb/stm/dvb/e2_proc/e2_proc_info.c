/*
 * e2_proc_info.c
 */

#include <linux/proc_fs.h> /* proc fs */
#include <asm/uaccess.h> /* copy_from_user */

int proc_info_model_read(char *page, char **start, off_t off, int count,
						 int *eof, void *data_unused)
{
	int len = 0;
	printk("%s\n", __FUNCTION__);
#if defined(CUBEREVO)
	len = sprintf(page, "cuberevo\n");
#elif defined(CUBEREVO_MINI)
	len = sprintf(page, "cuberevo_mini\n");
#elif defined(CUBEREVO_MINI2)
	len = sprintf(page, "cuberevo_mini2\n");
#elif defined(CUBEREVO_250HD)
	len = sprintf(page, "cuberevo_250hd\n");
#elif defined(CUBEREVO_MINI_FTA)
	len = sprintf(page, "cuberevo_mini_fta\n");
#elif defined(CUBEREVO_2000HD)
	len = sprintf(page, "cuberevo_2000hd\n");
#elif defined(CUBEREVO_9500HD)
	len = sprintf(page, "cuberevo_9500hd\n");
#elif defined(CUBEREVO_3000HD)
	len = sprintf(page, "cuberevo_3000hd\n");
#elif defined(TF7700)
	len = sprintf(page, "tf7700\n");
#elif defined(HL101)
	len = sprintf(page, "hl101\n");
#elif defined(VIP1_V2)
	len = sprintf(page, "vip1_v2\n");
#elif defined(VIP2_V1)
	len = sprintf(page, "vip2_v1\n");
#elif defined(UFS922)
	len = sprintf(page, "ufs922\n");
#elif defined(UFC960)
	len = sprintf(page, "ufc960\n");
#elif defined(UFS912)
	len = sprintf(page, "ufs912\n");
#elif defined(UFS913)
	len = sprintf(page, "ufs913\n");
#elif defined(SPARK)
	len = sprintf(page, "spark\n");
#elif defined(SPARK7162)
	len = sprintf(page, "spark7162\n");
#elif defined(FORTIS_HDBOX)
	len = sprintf(page, "fortis_hdbox\n");
#elif defined(QBOXHD)
	len = sprintf(page, "qboxhd\n");
#elif defined(QBOXHD_MINI)
	len = sprintf(page, "qboxhd_mini\n");
#elif defined(HOMECAST5101)
	len = sprintf(page, "hs5101\n");
#elif defined(OCTAGON1008)
	len = sprintf(page, "octagon1008\n");
#elif defined(ATEVIO7500)
	len = sprintf(page, "atevio7500\n");
#elif defined(HS7110)
	len = sprintf(page, "hs7110\n");
#elif defined(HS7119)
	len = sprintf(page, "hs7119\n");
#elif defined(HS7420)
	len = sprintf(page, "hs7420\n");
#elif defined(HS7429)
	len = sprintf(page, "hs7429\n");
#elif defined(HS7810A)
	len = sprintf(page, "hs7810a\n");
#elif defined(HS7819)
	len = sprintf(page, "hs7819\n");
#elif defined(FOREVER_NANOSMART)
	len = sprintf(page, "forever_nanosmart\n");
#elif defined(FOREVER_9898HD)
	len = sprintf(page, "forever_9898hd\n");
#elif defined(DP7001)
	len = sprintf(page, "dp7001\n");
#elif defined(FOREVER_3434HD)
	len = sprintf(page, "forever_3434hd\n");
#elif defined(FOREVER_2424HD)
	len = sprintf(page, "forever_2424hd\n");
#elif defined(GPV8000)
	len = sprintf(page, "gpv8000\n");
#elif defined(EP8000)
	len = sprintf(page, "ep8000\n");
#elif defined(EPP8000)
	len = sprintf(page, "epp8000\n");
#elif defined(ATEMIO520)
	len = sprintf(page, "atemio520\n");
#elif defined(ATEMIO530)
	len = sprintf(page, "atemio530\n");
#elif defined(IPBOX9900)
	len = sprintf(page, "ipbox9900\n");
#elif defined(IPBOX99)
	len = sprintf(page, "ipbox99\n");
#elif defined(IPBOX55)
	len = sprintf(page, "ipbox55\n");
#elif defined(ADB_BOX)
	len = sprintf(page, "adb_box\n");
#elif defined(VITAMIN_HD5000)
	len = sprintf(page, "vitamin_hd5000\n");
#elif defined(SAGEMCOM88)
	len = sprintf(page, "sagemcom88\n");
#elif defined(UFS910)
	len = sprintf(page, "ufs910\n");
#elif defined(ARIVALINK200)
	len = sprintf(page, "arivalink200\n");
#elif defined(PACE7241)
	len = sprintf(page, "pace7241\n");
#else
	len = sprintf(page, "unknown\n");
#endif
	return len;
}

