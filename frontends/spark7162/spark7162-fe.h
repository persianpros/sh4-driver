/****************************************************************************
 *
 * Spark7162 frontend driver
 * 
 * Original code:
 * Copyright (C), 2011-2016, AV Frontier Tech. Co., Ltd.
 *
 */
#ifndef __SPARK7162_H
#define __SPARK7162_H

#ifdef __cplusplus
extern "C" {
#endif

#define dprintk(level, x...) do \
{ \
	if ((paramDebug) && ((paramDebug >= level) || level == 0)) \
	{ \
		printk(TAGDEBUG x); \
	} \
} while (0)

#ifdef __cplusplus
}
#endif

#endif // __SPARK7162_H
// vim:ts=4
