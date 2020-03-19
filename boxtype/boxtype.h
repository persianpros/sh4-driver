/*
 */
#define DEBUG 1

#if defined(DEBUG)
#define dprintk(fmt, args...) \
	printk(fmt, ##args)
#else
#define dprintk(fmt, args...)
#endif
// vim:ts=4
