#ifndef __fulan_names__
#define __fulan_names__

/***********************************************************
 *
 * This file uses the STB-ID issued by Fulan for a receiver
 * and provides information regarding the brand name and
 * the model name matching the STB-ID.
 *
 * STB-ID format:
 * AA:BB:CC:DD:EE:FF:GG
 * in which are:
 * AA          : generic basic model, e.g. 0C is a Spark7162
 * BB          : usually zero
 * CC          : brand name code
 * DD:EE:FF:GG : last four bytes of the MAC (may be serial number)
 *
 * Information used below was retrieved from the net:
 * https://wiki.tuxbox-neutrino.org/wiki/Hardware:Spark
 * https://wiki.tuxbox-neutrino.org/wiki/Hardware:Spark7162
 * 
 */
unsigned char *brand_name[70] =  // index is first digit
{
	"Fulan",               // 0x00
	"Unknown",             // 0x01
	"Unknown",             // 0x02
	"Truman",              // 0x03
	"Delta",               // 0x04
	"Startrack",           // 0x05
	"Unknown",             // 0x06
	"Golden Media",        // 0x07
	"Edision",             // 0x08
	"Unknown",             // 0x09
	"Amiko",               // 0x0A
	"Galaxy Innovations",  // 0x0B
	"Unknown",             // 0x0C
	"Dynavision",          // 0x0D
	"SAB",                 // 0x0E
	"Unknown",             // 0x0F
	"Unknown",             // 0x10
	"Unknown",             // 0x11
	"Unknown",             // 0x12
	"Unknown",             // 0x13
	"Unknown",             // 0x14
	"Superbox",            // 0x15
	"Unknown",             // 0x16
	"Unknown",             // 0x17
	"Unknown",             // 0x18
	"Unknown",             // 0x19
	"Unknown",             // 0x1A
	"Unknown",             // 0x1B
	"Unknown",             // 0x1C
	"Satcom / Fulan",      // 0x1D
	"Unknown",             // 0x1E
	"Unknown",             // 0x1F
	"Samsat"               // 0x20
	"Visionnet",           // 0x21
	"Unknown",             // 0x22
	"Unknown",             // 0x23
	"Unknown",             // 0x24
	"Unknown",             // 0x25
	"Unknown",             // 0x26
	"Unknown",             // 0x27
	"Unknown",             // 0x28
	"Unknown",             // 0x29
	"Unknown",             // 0x2A
	"Icecrypt",            // 0x2B
	"Unknown",             // 0x2C
	"Unknown",             // 0x2D
	"Unknown",             // 0x2E
	"Unknown",             // 0x2F
	"Unknown",             // 0x30
	"Unknown",             // 0x31
	"Unknown",             // 0x32
	"Unknown",             // 0x33
	"Unknown",             // 0x34
	"Unknown",             // 0x35
	"Unknown",             // 0x36
	"Unknown",             // 0x37
	"Unknown",             // 0x38
	"Unknown",             // 0x39
	"Unknown",             // 0x3A
	"Icecrypt",            // 0x3B
	"Unknown",             // 0x3C
	"Unknown",             // 0x3D
	"Unknown",             // 0x3E
	"Unknown",             // 0x3F
	"Unknown",             // 0x40
	"Unknown",             // 0x41
	"Unknown",             // 0x42
	"Sogno",               // 0x43
	"Unknown",             // 0x44
	"Interstar",           // 0x45 - 69
};

// TODO: HL-101 / Argus VIP models

// spark models
unsigned char *model_name_09[70] =  // index is third digit
{
	"Unknown",                                                     // 0x00 Fulan
	"Unknown",                                                     // 0x01
	"Unknown",                                                     // 0x02
	"Premier 1 Plus",                                              // 0x03 Truman
	"Unknown",                                                     // 0x04 Delta
	"SRT 2020 HD / SRT 2020 HD Plus",                              // 0x05 Startrack
	"Unknown",                                                     // 0x06
	"990 CR HD PVR Spark LX / Spark Reloaded",                     // 0x07 Golden Media
	"Argus Pingulux / Argus Pingulux Plus / Argus Pingulux Mini",  // 0x08 Edision
	"Unknown",                                                     // 0x09
	"Alien SHD8900",                                               // 0x0A Amiko
	"S8120E",                                                      // 0x0B Galaxy Innovations
	"Unknown",                                                     // 0x0C
	"Spark / Spark PLUS",                                          // 0x0D Dynavision
	"Unix F+ Solo",                                                // 0x0E SAB
	"Unknown",                                                     // 0x0F
	"Unknown",                                                     // 0x10
	"Unknown",                                                     // 0x11
	"Unknown",                                                     // 0x12
	"Unknown",                                                     // 0x13
	"Unknown",                                                     // 0x14
	"S750",                                                        // 0x15 Superbox
	"Unknown",                                                     // 0x16
	"Unknown",                                                     // 0x17
	"Unknown",                                                     // 0x18
	"Unknown",                                                     // 0x19
	"Unknown",                                                     // 0x1A
	"Unknown",                                                     // 0x1B
	"Unknown",                                                     // 0x1C
	"7162",                                                        // 0x1D Satcom / Fulan
	"Unknown",                                                     // 0x1E
	"Unknown",                                                     // 0x1F
	"Linux 1",                                                     // 0x20 Samsat
	"Fireball 101 / Hammer 5400",                                  // 0x21 Visionnet
	"Unknown",                                                     // 0x22
	"Unknown",                                                     // 0x23
	"Unknown",                                                     // 0x24
	"Unknown",                                                     // 0x25
	"Unknown",                                                     // 0x26
	"Unknown",                                                     // 0x27
	"Unknown",                                                     // 0x28
	"Unknown",                                                     // 0x29
	"Unknown",                                                     // 0x2A
	"S3700 CHD",                                                   // 0x2B Icecrypt
	"Unknown",                                                     // 0x2C
	"Unknown",                                                     // 0x2D
	"Unknown",                                                     // 0x2E
	"Unknown",                                                     // 0x2F
	"Unknown",                                                     // 0x30
	"Unknown",                                                     // 0x31
	"Unknown",                                                     // 0x32
	"Unknown",                                                     // 0x33
	"Unknown",                                                     // 0x34
	"Unknown",                                                     // 0x35
	"Unknown",                                                     // 0x36
	"Unknown",                                                     // 0x37
	"Unknown",                                                     // 0x38
	"Unknown",                                                     // 0x39
	"Unknown",                                                     // 0x3A
	"Unknown",                                                     // 0x3B
	"Unknown",                                                     // 0x3C
	"Unknown",                                                     // 0x3D
	"Unknown",                                                     // 0x3E
	"Unknown",                                                     // 0x3F
	"Unknown",                                                     // 0x40
	"Unknown",                                                     // 0x41
	"Unknown",                                                     // 0x42
	"Spark Revolution",                                            // 0x43 Sogno
	"Unknown",                                                     // 0x44
	"Unknown"                                                      // 0x45 Interstar
};

// spark7162 models
unsigned char *model_name_0c[70] =  // index is third digit
{
	"Unknown",              // 0x00
	"Unknown",              // 0x01
	"Unknown",              // 0x02
	"Top Box 2",            // 0x03 Truman
	"Unknown",              // 0x04 Delta
	"Unknown",              // 0x05 Startrack
	"Unknown",              // 0x06
	"Spark TripleX",        // 0x07 Golden Media
	"Unknown",              // 0x08 Edision
	"Unknown",              // 0x09
	"Alien2/2+",            // 0x0A Amiko
	"Avatar 3 / Avatar 3",  // 0x0B Galaxy Innovations
	"Unknown",              // 0x0C
	"7162",                 // 0x0D Dynavision
	"Unix Triple HD",       // 0x0E SAB
	"Unknown",              // 0x0F
	"Unknown",              // 0x10
	"Unknown",              // 0x11
	"Unknown",              // 0x12
	"Unknown",              // 0x13
	"Unknown",              // 0x14
	"Z500",                 // 0x15  Superbox
	"Unknown",              // 0x16
	"Unknown",              // 0x17
	"Unknown",              // 0x18
	"Unknown",              // 0x19
	"Unknown",              // 0x1A
	"Unknown",              // 0x1B
	"Unknown",              // 0x1C
	"7162",                 // 0x1D Satcom
	"Unknown",              // 0x1E
	"Unknown",              // 0x1F
	"7162",                 // 0x20 Samsat
	"Falcon",               // 0x21 Visionnet
	"Unknown",              // 0x22
	"Unknown",              // 0x23
	"Unknown",              // 0x24
	"Unknown",              // 0x25
	"Unknown",              // 0x26
	"Unknown",              // 0x27
	"Unknown",              // 0x28
	"Unknown",              // 0x29
	"Unknown",              // 0x2A
	"S3700 CHD",            // 0x2B Icecrypt
	"Unknown",              // 0x2C
	"Unknown",              // 0x2D
	"Unknown",              // 0x2E
	"Unknown",              // 0x2F
	"Unknown",              // 0x30
	"Unknown",              // 0x31
	"Unknown",              // 0x32
	"Unknown",              // 0x33
	"Unknown",              // 0x34
	"Unknown",              // 0x35
	"Unknown",              // 0x36
	"Unknown",              // 0x37
	"Unknown",              // 0x38
	"Unknown",              // 0x39
	"Unknown",              // 0x3A
	"Unknown",              // 0x3B
	"Unknown",              // 0x3C
	"Unknown",              // 0x3D
	"Unknown",              // 0x3E
	"Unknown",              // 0x3F
	"Unknown",              // 0x40
	"Unknown",              // 0x41
	"Unknown",              // 0x42
	"Spark Triple",         // 0x43 Sogno
	"Unknown",              // 0x44
	"Unknown",              // 0x45 Interstar
};

#endif  // __fulan_names__
// vim:ts=4
