#ifndef ts_makros_123
#define ts_makros_123

/* ********************************************* */
/* Helper Functions to access ts header fields   */
/* ********************************************* */

static inline u16 ts_pid ( const u8 * buf )
{
  return ( ( buf[1] & 0x1f ) << 8 ) + buf[2];
}

static inline u8 ts_cc ( const u8 * buf )
{
  return ( buf[3] & 0x0f );
}

static inline u8 ts_scrambled ( const u8 * buf )
{
  return ( buf[3] & 0xc0 );
}

static inline u8 ts_error ( const u8 * buf )
{
  return ( buf[1] & 0x80 );
}

static inline u8 ts_payload_unit ( const u8 * buf )
{
  return ( buf[1] & 0x40 );
}

static inline u8 ts_priority ( const u8 * buf )
{
  return ( buf[1] & 0x20 );
}

static inline u8 ts_adaptation ( const u8 * buf )
{
  return ( buf[3] & 0x30 );
}

static inline u8 ts_sync ( const u8 * buf )
{
  return ( buf[0]);
}

/* ********************************************* */
/* TS Packet Checker functions                   */
/* ********************************************* */

static inline int getOutOfSync(u8* data, int num)
{
	int count = 0;
	int vLoop;
	
  	for (vLoop = 0; vLoop < num; vLoop++)
  	{
		if (data[0] != 0x47)
			count++;	
		data += 188;
	}
	
	return count;
}

static inline int isPacketValid(u8* data)
{
	return ((ts_sync(data) == 0x47) && (ts_error(data) == 0) );
}

#endif
