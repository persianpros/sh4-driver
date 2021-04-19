#ifndef __MXL301_H
#define __MXL301_H

struct MxL301_config
{
	char name[128];

	u8             addr;
	fe_bandwidth_t bandwidth;
	u32            Frequency;
	u32            IF;
	u32            TunerStep;
};
extern struct dvb_frontend *mxlx0x_attach(struct dvb_frontend *fe, const struct MxL301_config *config, struct i2c_adapter *i2c);

#endif  // __MXL301_H
// vim:ts=4
