/*****************************************************************************/
/* COPYRIGHT (C) 2009 STMicroelectronics - All Rights Reserved               */
/* ST makes no warranty express or implied including but not limited to,     */
/* any warranty of                                                           */
/*                                                                           */
/*   (i)  merchantability or fitness for a particular purpose and/or         */
/*   (ii) requirements, for a particular purpose in relation to the LICENSED */
/*        MATERIALS, which is provided “AS IS”, WITH ALL FAULTS. ST does not */
/*        represent or warrant that the LICENSED MATERIALS provided here     */
/*        under is free of infringement of any third party patents,          */
/*        copyrights,trade secrets or other intellectual property rights.    */
/*        ALL WARRANTIES, CONDITIONS OR OTHER TERMS IMPLIED BY LAW ARE       */
/*        EXCLUDED TO THE FULLEST EXTENT PERMITTED BY LAW                    */
/*                                                                           */
/*****************************************************************************/
/**
 @File   fe_sat_tuner.h
 @brief



*/
#ifndef H_FE_SAT_TUNER
	#define H_FE_SAT_TUNER

	#include "chip.h"
	#include "stfe_utilities.h"


	/* structures -------------------------------------------------------------- */




    typedef struct
    {

		TUNER_Model_t		Model;    	/* Tuner model */
    	FE_DEMOD_Model_t    DemodModel;
    	ST_String_t 		TunerName;	/* Tuner name */
		U8 					TunerI2cAddress;			/* Tuner I2C address */
        U8              	*DefVal;   /* pointer to table of default values */
		TUNER_Band_t		Band_Select;
        U32            		Reference;	/* reference frequency (Hz) */
        U32            		IF;       	/* Intermediate frequency (KHz) */
        TUNER_IQ_t    		IQ_Wiring; 	/* hardware I/Q invertion */
		TUNER_IQ_t			SpectrInvert;/* tuner invert spectrum*/
        U32 		  		FirstIF ;   /* First IF frequency for Microtune tuner */
		void                *additionalParams;	/* pointer to any specific data of a specific tuner */
		U32			    	LNAConfig;	/* Setting of the LNA in Microtune tuners */
	    U32             	UHFSens;    /* Sensitivity Setting in Microtune tuners */
        U32			    	Fxtal;		/* tuner xtal freq value*/
		U32            	 	StepSize;
		U32             	BandWidth;
		void*	      		mt2266_handle;	/* Handle for the Microtune tuner MT2066 API data*/
		U8			  		CalValue;      /*calibration value for Silicon tuners*/
		U8			  		CLPF_CalValue;      /*calibration value for Silicon tuners*/
		/*BOOL		  		ShiftNominal; */ /*value for Silicon tuners*/
		TUNER_Lock_t  		Lock;
	    STCHIP_Handle_t 	RepeaterHost;
		STCHIP_RepeaterFn_t RepeaterFn;
		BOOL                Repeater;         /* Is repeater enabled or not ? */
		#ifdef CHIP_STAPI
			U32 			TopLevelHandle; /*Used in STTUNER : Change ///AG*/
		#endif
        TUNER_Dual_t TunerPath;
		U32			   BandBorder;					/* Band border Frequency in KHz (limit frequency between high band and low band for wide band tuners) default is 950000*/
		TUNER_WIDEBandS_t	BandSelect;	/* for wide band tuner (6130 like)hardware band wiring */
		TUNER_WIDEBandS_t	BandSelect2;		/* for dual wide band tuner (6120 like)hardware band wiring */
		TUNER_DualInput_t	InputSelect;		/* dual tuner input select*/
		S32 OutputClockDivider; /*output clock divider : 0 to stop the output clock*/
		TUNER_RFSource_t RF_Source ;
		U8 Lna_agc_mode; /* Applicable only for 6111 tuner */
		U8 Lna_agc_ref;  /* Applicable only for 6111 tuner */

    }
    SAT_TUNER_InitParams_t;

    typedef SAT_TUNER_InitParams_t *SAT_TUNER_Params_t;

	/* functions --------------------------------------------------------------- */

	/* create instance of tuner register mappings */

	TUNER_Error_t FE_Sat_TunerInit(void *pTunerInit,STCHIP_Handle_t *TunerHandle);
	TUNER_Error_t FE_Sat_TunerSetStandby(STCHIP_Handle_t hTuner, U8 StandbyOn);

	TUNER_Error_t FE_Sat_TunerSetFrequency(STCHIP_Handle_t hTuner,U32 Frequency);
	U32 FE_Sat_TunerGetFrequency(STCHIP_Handle_t hTuner);
	TUNER_Error_t FE_Sat_TunerSetBandwidth(STCHIP_Handle_t hTuner,U32 Bandwidth);
	U32 FE_Sat_TunerGetBandwidth(STCHIP_Handle_t hTuner);
	BOOL FE_Sat_TunerGetStatus(STCHIP_Handle_t hTuner);
	TUNER_Error_t FE_Sat_TunerWrite(STCHIP_Handle_t hTuner);
	TUNER_Error_t FE_Sat_TunerRead(STCHIP_Handle_t hTuner);
	TUNER_Error_t FE_Sat_TunerSetGain(STCHIP_Handle_t hTuner,S32 Gain);
	S32 FE_Sat_TunerGetGain(STCHIP_Handle_t hTuner);
	TUNER_Error_t FE_Sat_TunerSetOutputClock(STCHIP_Handle_t hTuner, S32 Divider);

	TUNER_Error_t FE_Sat_TunerSetAttenuator(STCHIP_Handle_t hTuner, BOOL AttenuatorOn);
	BOOL FE_Sat_TunerGetAttenuator(STCHIP_Handle_t hTuner);

	TUNER_Model_t	FE_Sat_TunerGetModel(STCHIP_Handle_t hTuner);
	void			FE_Sat_TunerSetIQ_Wiring(STCHIP_Handle_t hTuner, S8 IQ_Wiring);/*changed to retain common function prototypes ///AG*/
	S8		FE_Sat_TunerGetIQ_Wiring(STCHIP_Handle_t hTuner);
	TUNER_Error_t	FE_Sat_Tuner_RFSelect(STCHIP_Handle_t htuner );
	void			FE_Sat_TunerSetReferenceFreq(STCHIP_Handle_t hTuner, U32 Reference);
	U32				FE_Sat_TunerGetReferenceFreq(STCHIP_Handle_t hTuner);
	void			FE_Sat_TunerSetIF_Freq(STCHIP_Handle_t hTuner, U32 IF);
	U32				FE_Sat_TunerGetIF_Freq(STCHIP_Handle_t hTuner);
	void FE_Sat_TunerSetBandSelect(STCHIP_Handle_t hChip, U8 BandSelect);
	U8 FE_Sat_TunerGetBandSelect(STCHIP_Handle_t hChip);
	S32 FE_Sat_TunerGetRFGain(STCHIP_Handle_t hTuner,U32 AGCIntegrator,S32 BBGain) ;
	
TUNER_Error_t FE_Sat_TunerSetHMR_Filter(STCHIP_Handle_t hTuner,S32 filterValue) ;
	TUNER_Error_t FE_Sat_TunerTerm(STCHIP_Handle_t hTuner);

BOOL VGLNA_Detection(STCHIP_Info_t * hChip ,STFRONTEND_InitParams_t *InitParams);
#endif
