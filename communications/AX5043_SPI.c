//
//  AX5043_SPI.c
//  UHF_Transceiver
//
//  Created by Filipe Pereira on 5/26/16.
//
//

#include "AX5043_SPI.h"

struct radio_settings psk125_reg_settings = {
    .MODULATION         = { .address=AX_REG_MODULATION, 		.data=0x04},//PSK
    .ENCODING           = { .address=AX_REG_ENCODING,           .data=0x03},//Diff and INV (NRZI Enconding)
    .FRAMING            = { .address=AX_REG_FRAMING,            .data=0x06},//Raw, pattern match ,no CRC-32
    .PINFUNCSYSCLK      = { .address=AX_REG_PINFUNCSYSCLK,      .data=0x01},//Output 1
    .PINFUNCDCLK        = { .address=AX_REG_PINFUNCDCLK,        .data=0x01},//Output 1
    .PINFUNCDATA        = { .address=AX_REG_PINFUNCDATA,        .data=0x01},//Output 1
    .PINFUNCANTSEL      = { .address=AX_REG_PINFUNCANTSEL,      .data=0x01},//Output 1
    .PINFUNCPWRAMP      = { .address=AX_REG_PINFUNCPWRAMP,      .data=0x07},//Output Power Amp control
    .WAKEUPXOEARLY      = { .address=AX_REG_WAKEUPXOEARLY,      .data=0x01},//Nb of LPOSC cycles by which the XTAL Osc is woken up before the receiver
    .IFFREQ1            = { .address=AX_REG_IFFREQ1,            .data=0x01},//IF Freq (Computed by RadioLab)
    .IFFREQ0            = { .address=AX_REG_IFFREQ0,            .data=0x11},//IF Freq (Computed by RadioLab)
    .DECIMATION         = { .address=AX_REG_DECIMATION,	        .data=0x1E},//Filter decimation factor
    .RXDATARATE2        = { .address=AX_REG_RXDATARATE2,        .data=0x00},//Receiver data rate
    .RXDATARATE1        = { .address=AX_REG_RXDATARATE1,        .data=0x50},//Receiver data rate
    .RXDATARATE0        = { .address=AX_REG_RXDATARATE0,        .data=0x00},//Receiver data rate
    .MAXDROFFSET2       = { .address=AX_REG_MAXDROFFSET2,       .data=0x00},//maximum possible data rate offset
    .MAXDROFFSET1       = { .address=AX_REG_MAXDROFFSET1,       .data=0x00},//maximum possible data rate offset
    .MAXDROFFSET0       = { .address=AX_REG_MAXDROFFSET0,       .data=0x00},//maximum possible data rate offset
    .MAXRFOFFSET2       = { .address=AX_REG_MAXRFOFFSET2,       .data=0x80},//maximum frequency offset
    .MAXRFOFFSET1       = { .address=AX_REG_MAXRFOFFSET1,       .data=0x00},//maximum frequency offset
    .MAXRFOFFSET0       = { .address=AX_REG_MAXRFOFFSET0,       .data=0x00},//maximum frequency offset
    .AMPLFILTER         = { .address=AX_REG_AMPLFILTER,         .data=0x00},//3dB corner frequency of the Amplitude (Magnitude) Lowpass Filter
    .RXPARAMSETS        = { .address=AX_REG_RXPARAMSETS,        .data=0xF4},//Receiver Parameter Set Indirection
    .AGCGAIN0           = { .address=AX_REG_AGCGAIN0,           .data=0xD6},//AGC Speed
    .AGCTARGET0         = { .address=AX_REG_AGCTARGET0,         .data=0x84},//AGC Target
    .TIMEGAIN0          = { .address=AX_REG_TIMEGAIN0,          .data=0xA9},//Timing Gain
    .DRGAIN0            = { .address=AX_REG_DRGAIN0,            .data=0xA3},//Data Rate Gain
    .PHASEGAIN0         = { .address=AX_REG_PHASEGAIN0,         .data=0xC3},//Filter Index, Phase Gain
    .FREQUENCYGAINA0    = { .address=AX_REG_FREQUENCYGAINA0,	.data=0x46},//Frequency Gain A
    .FREQUENCYGAINB0    = { .address=AX_REG_FREQUENCYGAINB0,	.data=0x0A},//Frequency Gain B
    .FREQUENCYGAINC0    = { .address=AX_REG_FREQUENCYGAINC0,	.data=0x1F},//Frequency Gain C
    .FREQUENCYGAIND0    = { .address=AX_REG_FREQUENCYGAIND0,	.data=0x1F},//Frequency Gain D
    .AMPLITUDEGAIN0     = { .address=AX_REG_AMPLITUDEGAIN0,     .data=0x06},//Amplitude Gain
    .FREQDEV10          = { .address=AX_REG_FREQDEV10,          .data=0x00},//Receiver Frequency Deviation
    .FREQDEV00          = { .address=AX_REG_FREQDEV00,          .data=0x00},//Receiver Frequency Deviation
    .BBOFFSRES0         = { .address=AX_REG_BBOFFSRES0,         .data=0x00},//Baseband Offset Compensation Resistors
    .AGCGAIN1           = { .address=AX_REG_AGCGAIN1,           .data=0xD6},//AGC Speed
    .AGCTARGET1         = { .address=AX_REG_AGCTARGET1,         .data=0x84},//AGC Target
    .AGCAHYST1          = { .address=AX_REG_AGCAHYST1,          .data=0x00},//AGC Digital Threshold Range
    .AGCMINMAX1         = { .address=AX_REG_AGCMINMAX1,         .data=0x00},//AGC Digital Minimum/Maximum Set Points
    .TIMEGAIN1          = { .address=AX_REG_TIMEGAIN1,          .data=0xA7},//Timing Gain
    .DRGAIN1            = { .address=AX_REG_DRGAIN1,            .data=0xA2},//Data Rate Gain
    .PHASEGAIN1         = { .address=AX_REG_PHASEGAIN1,         .data=0xC3},//Filter Index, Phase Gain
    .FREQUENCYGAINA1    = { .address=AX_REG_FREQUENCYGAINA1,    .data=0x46},//Frequency Gain A
    .FREQUENCYGAINB1    = { .address=AX_REG_FREQUENCYGAINB1,	.data=0x0A},//Frequency Gain B
    .FREQUENCYGAINC1    = { .address=AX_REG_FREQUENCYGAINC1,	.data=0x1F},//Frequency Gain C
    .FREQUENCYGAIND1    = { .address=AX_REG_FREQUENCYGAIND1,	.data=0x1F},//Frequency Gain D
    .AMPLITUDEGAIN1     = { .address=AX_REG_AMPLITUDEGAIN1,	    .data=0x06},//Amplitude Gain
    .FREQDEV11          = { .address=AX_REG_FREQDEV11,	        .data=0x00},//Rx freq deviation
    .FREQDEV01          = { .address=AX_REG_FREQDEV01,	        .data=0x00},//Rx freq deviation
    .FOURFSK1           = { .address=AX_REG_FOURFSK1,	        .data=0x16},//Four FSK control
    .BBOFFSRES1         = { .address=AX_REG_BBOFFSRES1,	        .data=0x00},//BB offset compensation resistors
    .AGCGAIN3           = { .address=AX_REG_AGCGAIN3,           .data=0xFF},//AGC Speed
    .AGCTARGET3         = { .address=AX_REG_AGCTARGET3,         .data=0x84},//AGC Target
    .AGCAHYST3          = { .address=AX_REG_AGCAHYST3,          .data=0x00},//AGC Digital Threshold Range
    .AGCMINMAX3         = { .address=AX_REG_AGCMINMAX3,         .data=0x00},//AGC Digital Minimum/Maximum Set Points
    .TIMEGAIN3          = { .address=AX_REG_TIMEGAIN3,          .data=0xA6},//Timing Gain
    .DRGAIN3            = { .address=AX_REG_DRGAIN3,            .data=0xA1},//Data Rate Gain
    .PHASEGAIN3         = { .address=AX_REG_PHASEGAIN3,         .data=0xC3},//Filter Index, Phase Gain
    .FREQUENCYGAINA3    = { .address=AX_REG_FREQUENCYGAINA3,    .data=0x46},//Frequency Gain A
    .FREQUENCYGAINB3    = { .address=AX_REG_FREQUENCYGAINB3,	.data=0x0A},//Frequency Gain B
    .FREQUENCYGAINC3    = { .address=AX_REG_FREQUENCYGAINC3,	.data=0x1F},//Frequency Gain C
    .FREQUENCYGAIND3    = { .address=AX_REG_FREQUENCYGAIND3,	.data=0x1F},//Frequency Gain D
    .AMPLITUDEGAIN3     = { .address=AX_REG_AMPLITUDEGAIN3,	    .data=0x06},//Amplitude Gain
    .FREQDEV13          = { .address=AX_REG_FREQDEV13,	        .data=0x00},//Rx freq deviation
    .FREQDEV03          = { .address=AX_REG_FREQDEV03,	        .data=0x00},//Rx freq deviation
    .FOURFSK3           = { .address=AX_REG_FOURFSK3,	        .data=0x16},//Four FSK control
    .BBOFFSRES3         = { .address=AX_REG_BBOFFSRES3,	        .data=0x00},//BB offset compensation resistors
    .FSKDEV2            = { .address=AX_REG_FSKDEV2,	        .data=0x00},//FSK freq deviation
    .FSKDEV1            = { .address=AX_REG_FSKDEV1,	        .data=0x00},//FSK freq deviation
    .FSKDEV0            = { .address=AX_REG_FSKDEV0,	        .data=0x00},//FSK freq deviation
    .MODCFGA            = { .address=AX_REG_MODCFGA,	        .data=0x06},//Amplitude shaping mode
    .TXRATE2            = { .address=AX_REG_TXRATE2,	        .data=0x00},//TX bitrate
    .TXRATE1            = { .address=AX_REG_TXRATE1,	        .data=0x06},//TX bitrate
    .TXRATE0            = { .address=AX_REG_TXRATE0,	        .data=0xD4},//TX bitrate
//    .TXPWRCOEFFB1       = { .address=AX_REG_TXPWRCOEFFB1,	    .data=0x04},//TX predistortion ... 6.5dBm
//    .TXPWRCOEFFB0       = { .address=AX_REG_TXPWRCOEFFB0,	    .data=0x54},//TX predistortion
    .TXPWRCOEFFB1       = { .address=AX_REG_TXPWRCOEFFB1,	    .data=0x02},//TX predistortion ... 0dBm
    .TXPWRCOEFFB0       = { .address=AX_REG_TXPWRCOEFFB0,	    .data=0x07},//TX predistortion
//    .TXPWRCOEFFB1       = { .address=AX_REG_TXPWRCOEFFB1,	    .data=0x0F},//TX predistortion ... 15dBm
//    .TXPWRCOEFFB0       = { .address=AX_REG_TXPWRCOEFFB0,	    .data=0xFF},//TX predistortion
    .PLLVCOI            = { .address=AX_REG_PLLVCOI,	        .data=0x98},//PLL parameters
    .PLLRNGCLK          = { .address=AX_REG_PLLRNGCLK,	        .data=0x05},//PLL parameters
    .BBTUNE             = { .address=AX_REG_BBTUNE,	            .data=0x0F},//Baseband tuning
    .BBOFFSCAP          = { .address=AX_REG_BBOFFSCAP,	        .data=0x77},//Baseband gain offset compensation
    .PKTADDRCFG         = { .address=AX_REG_PKTADDRCFG,	        .data=0x01},//Packet addr config
    .PKTLENCFG          = { .address=AX_REG_PKTLENCFG,	        .data=0x80},//Packet length config
    .PKTLENOFFSET       = { .address=AX_REG_PKTLENOFFSET,	    .data=0x00},//Packet length offset
    .PKTMAXLEN          = { .address=AX_REG_PKTMAXLEN,	        .data=0xC8},//Packet max length
    .MATCH0PAT3         = { .address=AX_REG_MATCH0PAT3,	        .data=0xAA},//Pattern match
    .MATCH0PAT2         = { .address=AX_REG_MATCH0PAT2,	        .data=0xCC},//Pattern match
    .MATCH0PAT1         = { .address=AX_REG_MATCH0PAT1,	        .data=0xAA},//Pattern match
    .MATCH0PAT0         = { .address=AX_REG_MATCH0PAT0,	        .data=0xCC},//Pattern match
    .MATCH0LEN          = { .address=AX_REG_MATCH0LEN,	        .data=0x1F},//Pattern length
    .MATCH0MAX          = { .address=AX_REG_MATCH0MAX,	        .data=0x1F},//Max match
    .MATCH1PAT1         = { .address=AX_REG_MATCH1PAT1,	        .data=0x55},//Pattern match
    .MATCH1PAT0         = { .address=AX_REG_MATCH1PAT0,	        .data=0x55},//Pattern match
    .MATCH1LEN          = { .address=AX_REG_MATCH1LEN,	        .data=0x8A},//Pattern length
    .MATCH1MAX          = { .address=AX_REG_MATCH1MAX,	        .data=0x0A},//Max match
    .TMGTXBOOST         = { .address=AX_REG_TMGTXBOOST,         .data=0x5B},//Boost time
    .TMGTXSETTLE        = { .address=AX_REG_TMGTXSETTLE,	    .data=0x3E},//Settling time
    .TMGRXBOOST         = { .address=AX_REG_TMGRXBOOST,         .data=0x5B},//Boost time
    .TMGRXSETTLE        = { .address=AX_REG_TMGRXSETTLE,	    .data=0x3E},//Settling time
    .TMGRXOFFSACQ       = { .address=AX_REG_TMGRXOFFSACQ,	    .data=0x00},//Baseband offset acq
    .TMGRXCOARSEAGC     = { .address=AX_REG_TMGRXCOARSEAGC,	    .data=0x9C},//Coarse AGC time
    .TMGRXRSSI          = { .address=AX_REG_TMGRXRSSI,	        .data=0x03},//RSSI settling time
    .TMGRXPREAMBLE2     = { .address=AX_REG_TMGRXPREAMBLE2,	    .data=0x35},//Preamble 2 timeout
    .RSSIABSTHR         = { .address=AX_REG_RSSIABSTHR,	        .data=0xE0},//RSSI abs. threshold
    .BGNDRSSITHR        = { .address=AX_REG_BGNDRSSITHR,        .data=0x00},//Background RSSI rel. threshold
    .PKTCHUNKSIZE       = { .address=AX_REG_PKTCHUNKSIZE,	    .data=0x0D},//Packet chunk size
    .PKTACCEPTFLAGS     = { .address=AX_REG_PKTACCEPTFLAGS,	    .data=0x1C},//Packet ctrl accept flags
    .DACVALUE1          = { .address=AX_REG_DACVALUE1,	        .data=0x00},//DAC value
    .DACVALUE0          = { .address=AX_REG_DACVALUE0,	        .data=0x00},//DAC value
    .DACCONFIG          = { .address=AX_REG_DACCONFIG,	        .data=0x00},//DAC config
    .REF                = { .address=AX_REG_REF,	            .data=0x03},
    .XTALOSC            = { .address=AX_REG_XTALOSC,	        .data=0x04},
    .XTALAMPL           = { .address=AX_REG_XTALAMPL,	        .data=0x00},
    .TUNE_F1C           = { .address=AX_REG_TUNE_F1C,	        .data=0x07},//Tuning registers
    .TUNE_F21           = { .address=AX_REG_TUNE_F21,	        .data=0x68},//Tuning registers
    .TUNE_F22           = { .address=AX_REG_TUNE_F22,	        .data=0xFF},//Tuning registers
    .TUNE_F23           = { .address=AX_REG_TUNE_F23,	        .data=0x84},//Tuning registers
    .TUNE_F26           = { .address=AX_REG_TUNE_F26,	        .data=0x98},//Tuning registers
    .TUNE_F34           = { .address=AX_REG_TUNE_F34,	        .data=0x28},//Tuning registers
    .TUNE_F35           = { .address=AX_REG_TUNE_F35,	        .data=0x11},//Tuning registers
    .TUNE_F44           = { .address=AX_REG_TUNE_F44,	        .data=0x25},//Tuning registers
    .MODCFGP            = { .address=AX_REG_MODCFGP,	        .data=0xE1},
// Register TX/RX
	.PLLLOOP            = { .address=AX_REG_PLLLOOP,           .data=0x0B},
	.PLLCPI             = { .address=AX_REG_PLLCPI,            .data=0x10},
	.PLLVCODIV          = { .address=AX_REG_PLLVCODIV,         .data=0x24},
	.XTALCAP            = { .address=AX_REG_XTALCAP,           .data=0x00},
	.TUNE_F00           = { .address=AX_REG_TUNE_F00,          .data=0x0F},
	.TUNE_F18           = { .address=AX_REG_TUNE_F18,          .data=0x06},
// RXcont settings
    .TMGRXAGC           = { .address=AX_REG_TMGRXAGC,           .data=0x00},//Receiver AGC settling time
    .TMGRXPREAMBLE1     = { .address=AX_REG_TMGRXPREAMBLE1,     .data=0x00},//Receiver preamble 1 timeout
    .PKTMISCFLAGS       = { .address=AX_REG_PKTMISCFLAGS,       .data=0x00},//Packet misc flags
// Set Frequency (437.5 MHz)
    .FREQA0             = { .address=AX_REG_FREQA0,	            .data=0x55},//Output Frequency
    .FREQA1             = { .address=AX_REG_FREQA1,	            .data=0x55},//Output Frequency
    .FREQA2             = { .address=AX_REG_FREQA2,	            .data=0x1D},//Output Frequency
    .FREQA3             = { .address=AX_REG_FREQA3,	            .data=0x09},//Output Frequency
//
    .RSSIREFERENCE      = { .address=AX_REG_RSSIREFERENCE,      .data=0x00},//RSSI offset
//
    .PKTSTOREFLAGS      = { .address=AX_REG_PKTSTOREFLAGS,      .data=0x54},//Packet store flags
// Additional registers
    .MODCFGF            = { .address=AX_REG_MODCFGF,	        .data=0x00},//Freq shaping mode
};

FILE *fp;

void ax5043_readAllReg(){
    
    int i,reg_size;
    char data;
    struct reg_pair *SettingsPtr;
    
    SettingsPtr = &psk125_reg_settings.MODULATION;
    
    reg_size = sizeof(psk125_reg_settings)/sizeof(reg_pair);
    
    for (i=0; i < reg_size; i++) {
        data = ax5043_readReg(SettingsPtr->address);
        SettingsPtr ++;
    }
}

void ax5043_writeAllReg(){
    
    int i,reg_size;
    struct reg_pair *SettingsPtr;
    
    SettingsPtr = &psk125_reg_settings.MODULATION;
    
    // write register settings
    reg_size = sizeof(psk125_reg_settings)/sizeof(reg_pair);
    
    for (i=0; i < reg_size; i++) {
        ax5043_writeReg(SettingsPtr->address, SettingsPtr->data);
        SettingsPtr ++;
    }
}

void ax5043_writeReg(uint16_t addr, unsigned char value) {
	unsigned char data[2];
	unsigned char longData[3];
	
	if (addr < 0x70) {

		data[0] = addr | 0x80;
		data[1] = value;
		wiringPiSPIDataRW(SPI_DEVICE, data, 2);
		usleep(5);
	
	} else {
		
		longData[0] = (addr >> 8) | 0xF0;
		longData[1] = addr & 0xFF;
		longData[2] = value;
		
		wiringPiSPIDataRW(SPI_DEVICE, longData, 3);
	}
}

char ax5043_readReg(uint16_t addr) {
	unsigned char data[2];
	unsigned char longData[3];
	
	if (addr < 0x70) {
		data[0] = addr & 0x7F;
		data[1] = 0;
		wiringPiSPIDataRW(SPI_DEVICE, data, 2);
		//fprintf(fp,"0x%02x : 0x%02x 0x%02x\n", addr,data[0],data[1]);
		
	    usleep(5);
	    return data[1]; 
	} else {
		
		longData[0] = (addr >> 8) | 0x70;
		longData[1] = addr & 0xFF;
		longData[2] = 0;
		
		wiringPiSPIDataRW(SPI_DEVICE, longData, 3);
		//fprintf(fp,"0x%03x : 0x%02x 0x%02x 0x%02x\n", addr,longData[0],longData[1],longData[2]);
		usleep(5);
	    return longData[2]; 
	}
	
}

void ax5043_writePreamble() {
	
	ax5043_writeReg(AX_REG_FIFODATA,0x62);//Repeat command
	ax5043_writeReg(AX_REG_FIFODATA,0x38);//Flag byte...unenconded / raw / NoCRC
	ax5043_writeReg(AX_REG_FIFODATA,0x21);//8 bit pattern (+ 33 repetitions)= 272 bit preamble
	ax5043_writeReg(AX_REG_FIFODATA,0xAA);//Preamble byte...equivalent to 0xFF in differential (Ax-RadioLab input)
}

void ax5043_writePacket() {
    
    uint8_t k;
    
    //Long Sync 
    //SYNC command = FIFOCMD_DATA (0x01) | (sizeof(SYNCWORD) + 1) << 5)
    ax5043_writeReg(AX_REG_FIFODATA,0xA1);
    ax5043_writeReg(AX_REG_FIFODATA,0x18);
	ax5043_writeReg(AX_REG_FIFODATA,0xCC);
	ax5043_writeReg(AX_REG_FIFODATA,0xAA);
	ax5043_writeReg(AX_REG_FIFODATA,0xCC);
	ax5043_writeReg(AX_REG_FIFODATA,0xAA);
    
    ax5043_writeReg(AX_REG_FIFODATA,0xE1);//DATA command
    ax5043_writeReg(AX_REG_FIFODATA,0xC9);//127 = 0x80
    ax5043_writeReg(AX_REG_FIFODATA,0x13);//Flag byte : Packet start & end
    // 0  0  UNENC  RAW   NOCRC  RESIDUE   PKTEND  PKTSTART

    ax5043_writeReg(AX_REG_FIFODATA,0xC8);//Data
    
    for (k = 0; k < 19; k++)
    {
		ax5043_writeReg(AX_REG_FIFODATA,0x01);//Data
		ax5043_writeReg(AX_REG_FIFODATA,0x23);
		ax5043_writeReg(AX_REG_FIFODATA,0x45);
		ax5043_writeReg(AX_REG_FIFODATA,0x67);
		ax5043_writeReg(AX_REG_FIFODATA,0x89);
		ax5043_writeReg(AX_REG_FIFODATA,0x01);    
		ax5043_writeReg(AX_REG_FIFODATA,0x23);
		ax5043_writeReg(AX_REG_FIFODATA,0x45);    
		ax5043_writeReg(AX_REG_FIFODATA,0x67);  
		ax5043_writeReg(AX_REG_FIFODATA,0x89);
    }
        ax5043_writeReg(AX_REG_FIFODATA,0x01);    
		ax5043_writeReg(AX_REG_FIFODATA,0x23);
		ax5043_writeReg(AX_REG_FIFODATA,0x45);    
		ax5043_writeReg(AX_REG_FIFODATA,0x67);  
		ax5043_writeReg(AX_REG_FIFODATA,0x89);
		ax5043_writeReg(AX_REG_FIFODATA,0x01);
		ax5043_writeReg(AX_REG_FIFODATA,0x23);
	    ax5043_writeReg(AX_REG_FIFODATA,0x45);
		ax5043_writeReg(AX_REG_FIFODATA,0x67);
    
}  

void ax5043_writePacket2() {
    
    uint8_t k;
    
    //Long Sync 
    //SYNC command = FIFOCMD_DATA (0x01) | (sizeof(SYNCWORD) + 1) << 5)
    ax5043_writeReg(AX_REG_FIFODATA,0xA1);
    ax5043_writeReg(AX_REG_FIFODATA,0x18);
	ax5043_writeReg(AX_REG_FIFODATA,0xCC);
	ax5043_writeReg(AX_REG_FIFODATA,0xAA);
	ax5043_writeReg(AX_REG_FIFODATA,0xCC);
	ax5043_writeReg(AX_REG_FIFODATA,0xAA);
     
    ax5043_writeReg(AX_REG_FIFODATA,0xE1);//DATA command
    ax5043_writeReg(AX_REG_FIFODATA,0xC9);//127 = 0x80
    ax5043_writeReg(AX_REG_FIFODATA,0x13);//Flag byte : Packet start & end
    // 0  0  UNENC  RAW   NOCRC  RESIDUE   PKTEND  PKTSTART

    ax5043_writeReg(AX_REG_FIFODATA,0xC8);//Data
    
    for (k = 0; k < 19; k++)
    {
		ax5043_writeReg(AX_REG_FIFODATA,0x98);//Data
		ax5043_writeReg(AX_REG_FIFODATA,0x76);
		ax5043_writeReg(AX_REG_FIFODATA,0x54);
		ax5043_writeReg(AX_REG_FIFODATA,0x32);
		ax5043_writeReg(AX_REG_FIFODATA,0x10);
		ax5043_writeReg(AX_REG_FIFODATA,0x98);    
		ax5043_writeReg(AX_REG_FIFODATA,0x76);
		ax5043_writeReg(AX_REG_FIFODATA,0x54);    
		ax5043_writeReg(AX_REG_FIFODATA,0x32);  
		ax5043_writeReg(AX_REG_FIFODATA,0x10);
    }
        ax5043_writeReg(AX_REG_FIFODATA,0x98);    
		ax5043_writeReg(AX_REG_FIFODATA,0x76);
		ax5043_writeReg(AX_REG_FIFODATA,0x54);    
		ax5043_writeReg(AX_REG_FIFODATA,0x32);  
		ax5043_writeReg(AX_REG_FIFODATA,0x10);
		ax5043_writeReg(AX_REG_FIFODATA,0x98);
		ax5043_writeReg(AX_REG_FIFODATA,0x76);
	    ax5043_writeReg(AX_REG_FIFODATA,0x54);
		ax5043_writeReg(AX_REG_FIFODATA,0x32);
    
}  

void ax5043_writePacket3() {
    
    uint8_t k;
    
    //Long Sync 
    //SYNC command = FIFOCMD_DATA (0x01) | (sizeof(SYNCWORD) + 1) << 5)
    ax5043_writeReg(AX_REG_FIFODATA,0xA1);
    ax5043_writeReg(AX_REG_FIFODATA,0x18);
	ax5043_writeReg(AX_REG_FIFODATA,0xCC);
	ax5043_writeReg(AX_REG_FIFODATA,0xAA);
	ax5043_writeReg(AX_REG_FIFODATA,0xCC);
	ax5043_writeReg(AX_REG_FIFODATA,0xAA);
    
    ax5043_writeReg(AX_REG_FIFODATA,0xE1);//DATA command
    ax5043_writeReg(AX_REG_FIFODATA,0xC9);//127 = 0x80
    ax5043_writeReg(AX_REG_FIFODATA,0x13);//Flag byte : Packet start & end
    // 0  0  UNENC  RAW   NOCRC  RESIDUE   PKTEND  PKTSTART

    ax5043_writeReg(AX_REG_FIFODATA,0xC8);//Data
    
    for (k = 0; k < 19; k++)
    {
		ax5043_writeReg(AX_REG_FIFODATA,0xAA);//Data
		ax5043_writeReg(AX_REG_FIFODATA,0xBB);
		ax5043_writeReg(AX_REG_FIFODATA,0xCC);
		ax5043_writeReg(AX_REG_FIFODATA,0xDD);
		ax5043_writeReg(AX_REG_FIFODATA,0xEE);
		ax5043_writeReg(AX_REG_FIFODATA,0xFF);    
		ax5043_writeReg(AX_REG_FIFODATA,0x00);
		ax5043_writeReg(AX_REG_FIFODATA,0x11);    
		ax5043_writeReg(AX_REG_FIFODATA,0x22);  
		ax5043_writeReg(AX_REG_FIFODATA,0x33);
    }
		ax5043_writeReg(AX_REG_FIFODATA,0xAA);//Data
		ax5043_writeReg(AX_REG_FIFODATA,0xBB);
		ax5043_writeReg(AX_REG_FIFODATA,0xCC);
		ax5043_writeReg(AX_REG_FIFODATA,0xDD);
		ax5043_writeReg(AX_REG_FIFODATA,0xEE);
		ax5043_writeReg(AX_REG_FIFODATA,0xFF);    
		ax5043_writeReg(AX_REG_FIFODATA,0x00);
		ax5043_writeReg(AX_REG_FIFODATA,0x11);    
		ax5043_writeReg(AX_REG_FIFODATA,0x22);  

    
}  

void ax5043_writePacket4() {
    
    uint8_t k;
    
    //Long Sync 
    //SYNC command = FIFOCMD_DATA (0x01) | (sizeof(SYNCWORD) + 1) << 5)
    ax5043_writeReg(AX_REG_FIFODATA,0xA1);
    ax5043_writeReg(AX_REG_FIFODATA,0x18);
	ax5043_writeReg(AX_REG_FIFODATA,0xCC);
	ax5043_writeReg(AX_REG_FIFODATA,0xAA);
	ax5043_writeReg(AX_REG_FIFODATA,0xCC);
	ax5043_writeReg(AX_REG_FIFODATA,0xAA);
    
    ax5043_writeReg(AX_REG_FIFODATA,0xE1);//DATA command
    ax5043_writeReg(AX_REG_FIFODATA,0xC9);//127 = 0x80
    ax5043_writeReg(AX_REG_FIFODATA,0x13);//Flag byte : Packet start & end
    // 0  0  UNENC  RAW   NOCRC  RESIDUE   PKTEND  PKTSTART

    ax5043_writeReg(AX_REG_FIFODATA,0xC8);//Data
    
    for (k = 0; k < 19; k++)
    {
		ax5043_writeReg(AX_REG_FIFODATA,0xFF);//Data
		ax5043_writeReg(AX_REG_FIFODATA,0xEE);
		ax5043_writeReg(AX_REG_FIFODATA,0xDD);
		ax5043_writeReg(AX_REG_FIFODATA,0xCC);
		ax5043_writeReg(AX_REG_FIFODATA,0xBB);
		ax5043_writeReg(AX_REG_FIFODATA,0xAA);    
		ax5043_writeReg(AX_REG_FIFODATA,0x00);
		ax5043_writeReg(AX_REG_FIFODATA,0x11);    
		ax5043_writeReg(AX_REG_FIFODATA,0x22);  
		ax5043_writeReg(AX_REG_FIFODATA,0x33);
    }
		ax5043_writeReg(AX_REG_FIFODATA,0xFF);//Data
		ax5043_writeReg(AX_REG_FIFODATA,0xEE);
		ax5043_writeReg(AX_REG_FIFODATA,0xDD);
		ax5043_writeReg(AX_REG_FIFODATA,0xCC);
		ax5043_writeReg(AX_REG_FIFODATA,0xBB);
		ax5043_writeReg(AX_REG_FIFODATA,0xAA);    
		ax5043_writeReg(AX_REG_FIFODATA,0x00);
		ax5043_writeReg(AX_REG_FIFODATA,0x11);    
		ax5043_writeReg(AX_REG_FIFODATA,0x22); 
    
}  

void ax5043_set_datarate() {
	//F_XTAL = 48MHz
	//Set to 125 bps...TXRATE = 0x2C = 44 = [BITRATE / F_XTAL].2^24 + 0.5
    //Set to 500bps ...TXRATE = 0x0000AF
    ax5043_writeReg(AX_REG_DECIMATION,0x7F);
    ax5043_writeReg(AX_REG_RXDATARATE2,0x00); 
    ax5043_writeReg(AX_REG_RXDATARATE1,0x5E);
    ax5043_writeReg(AX_REG_RXDATARATE0,0x7C);
    ax5043_writeReg(AX_REG_AGCGAIN0,0xE8);
    ax5043_writeReg(AX_REG_TIMEGAIN0,0xB9);
    ax5043_writeReg(AX_REG_DRGAIN0,0xB3);
    ax5043_writeReg(AX_REG_AGCGAIN1,0xE8);
    ax5043_writeReg(AX_REG_TIMEGAIN1,0xB7);
    ax5043_writeReg(AX_REG_DRGAIN1,0xB2);
    ax5043_writeReg(AX_REG_TIMEGAIN3,0xB6);
    ax5043_writeReg(AX_REG_DRGAIN3,0xB1);
    ax5043_writeReg(AX_REG_TXRATE1,0x00);
    ax5043_writeReg(AX_REG_TXRATE0,0xAF);
    ax5043_writeReg(AX_REG_TUNE_F35,0x12);
     
    ////Set to 5kbps ...TXRATE = 0x0006D4
    //ax5043_writeReg(AX_REG_DECIMATION,0x1E);
    //ax5043_writeReg(AX_REG_RXDATARATE2,0x00); 
    //ax5043_writeReg(AX_REG_RXDATARATE1,0x50);
    //ax5043_writeReg(AX_REG_RXDATARATE0,0x00);
    //ax5043_writeReg(AX_REG_AGCGAIN0,0xD6);
    //ax5043_writeReg(AX_REG_TIMEGAIN0,0xA9);
    //ax5043_writeReg(AX_REG_DRGAIN0,0xA3);
    //ax5043_writeReg(AX_REG_AGCGAIN1,0xD6);
    //ax5043_writeReg(AX_REG_TIMEGAIN1,0xA7);
    //ax5043_writeReg(AX_REG_DRGAIN1,0xA2);   
    //ax5043_writeReg(AX_REG_TIMEGAIN3,0xA6);
    //ax5043_writeReg(AX_REG_DRGAIN3,0xA1);
    //ax5043_writeReg(AX_REG_TXRATE1,0x06);
    //ax5043_writeReg(AX_REG_TXRATE0,0xD4);
    //ax5043_writeReg(AX_REG_TUNE_F35,0x11);
    
    //Barker 
    //ax5043_writeReg(AX_REG_MATCH0PAT3,0x53);
    //ax5043_writeReg(AX_REG_MATCH0PAT2,0xE0);
    //ax5043_writeReg(AX_REG_MATCH0PAT1,0x00);
    //ax5043_writeReg(AX_REG_MATCH0PAT0,0x00);
    //ax5043_writeReg(AX_REG_MATCH0LEN,0x0A);
    //ax5043_writeReg(AX_REG_MATCH0MAX,0x0A);
    //ax5043_writeReg(AX_REG_TMGRXPREAMBLE2,0x15);
  
}

void ax5043_set_reg_tx() {

    //ax5043_writeReg(AX_REG_PLLVCOI,0x98);
	ax5043_writeReg(AX_REG_PLLLOOP,0x0B);//500kHz & Use FreqA...
	ax5043_writeReg(AX_REG_PLLCPI,0x10);//0x08
	ax5043_writeReg(AX_REG_PLLVCODIV,0x24);
	ax5043_writeReg(AX_REG_XTALCAP,0x00);
	ax5043_writeReg(AX_REG_TUNE_F00,0x0F);
	ax5043_writeReg(AX_REG_TUNE_F18,0x06);
}

void ax5043_set_reg_rx() {
	
	//ax5043_writeReg(AX_REG_PLLVCOI,0x99);
	ax5043_writeReg(AX_REG_PLLLOOP,0x0B);//500kHz; 0x09...100kHz BW for ranging
	ax5043_writeReg(AX_REG_PLLCPI,0x10);//0x08
	ax5043_writeReg(AX_REG_PLLVCODIV,0x25);//fPD = fXTAL/2
	ax5043_writeReg(AX_REG_XTALCAP,0x00);//0 - TCXO
	ax5043_writeReg(AX_REG_TUNE_F00,0x0F);
	ax5043_writeReg(AX_REG_TUNE_F18,0x02);	
}


void ax5043_init() {
	
	char reg;
	
	//set_registers
	ax5043_writeAllReg();
    
    //Set freqA and tune for TX
    ax5043_set_reg_tx();
    
	//set datarate...
	ax5043_set_datarate();

    //Perform autoranging
    ax5043_autoranging();
}

void ax5043_autoranging() {
	
	char reg, reg2;
	
	ax5043_writeReg(AX_REG_PWRMODE,PWRMODE_POWERDOWN);
	usleep(10);
	//turn XTAL ON...power in stand-by
	ax5043_writeReg(AX_REG_PWRMODE,PWRMODE_STANDBY);
	
	//Wait till Xtal is running
	do {
		reg = ax5043_readReg(AX_REG_XTALSTATUS);
		fprintf(fp,"XTAL STATUS: %02x\n",reg);
		usleep(10);
	} while (reg != 0x01);
	
	//Start autoranging (set RNGSTART) with VCO ranging at 8 (to be tuned)
	fprintf(fp,"set RNGSTART...\n");
	ax5043_writeReg(AX_REG_PLLRANGINGA,0x18);
	
    do {
		reg = ax5043_readReg(AX_REG_PLLRANGINGA);
		fprintf(fp,"waiting for Autoranging to terminate...\n");
		usleep(10);
	} while (reg & 0x10);
		
	//Powerdown
	ax5043_writeReg(AX_REG_PWRMODE,PWRMODE_POWERDOWN);
	
}

void ax5043_TX() {
	
    char reg;
    uint8_t transmitted;

    transmitted = 0;

    while (transmitted < 1800)
	{
		//According to ERRATA for Silicon v51 
		//turn off receiver
		ax5043_writeReg(AX_REG_PWRMODE,PWRMODE_STANDBY);
		usleep(100);
		//release FIFO ports
		ax5043_writeReg(AX_REG_PWRMODE,PWRMODE_FIFOON);
		usleep(100);
		
		//Set freqA and tune for TX
	    ax5043_set_reg_tx();
		
		//Clear FIFO data and flags
	    ax5043_writeReg(AX_REG_FIFOSTAT,0x03);
		
		//FULL TX Mode
		ax5043_writeReg(AX_REG_PWRMODE,PWRMODE_FULLTX);
		usleep(100);
		
		//write Preamble
		ax5043_writePreamble();
		
		if (transmitted < 50) {
			//write Packet
		    ax5043_writePacket();	
		} else if (transmitted < 100) {
			ax5043_writePacket2();
		} else if (transmitted < 150) {
			ax5043_writePacket3();
		} else {
			ax5043_writePacket4();
		}
		

	
		
		//Wait till Xtal is running
		do {
			//fprintf(fp,"waiting for Xtal...\n");
			reg = ax5043_readReg(AX_REG_XTALSTATUS);
			usleep(10);
		} while (reg != AX_REG_XTALSTATUS_MASK);
		
	    //Commit FIFO
		ax5043_writeReg(AX_REG_FIFOSTAT,0x04);
		
		printf("Transmiting...");
		transmitted++;
		usleep(10);
		
	    //Wait till TX is done
		do {
			reg = ax5043_readReg(AX_REG_RADIOSTATE);
			//fprintf(fp,"RADIOSTATE: %02x\n",reg);
	        //fprintf(fp,"waiting for TX to finish...\n");
	        usleep(10);
		} while (reg != 0x00);
		printf("TX done...Packet No : %u\n",transmitted);
		//Powerdown
		ax5043_writeReg(AX_REG_PWRMODE,PWRMODE_POWERDOWN);
		
		usleep(5000000);
    }
}

void ax5043_RX() {

    uint8_t received,i;
    uint16_t FIFObytes;
    char reg, rstat, fif0, fif1;
    
    ax5043_set_reg_rx();

    received = 0;
    
    while (received < 100)
	{
	    //Clear FIFO data and flags
	    ax5043_writeReg(AX_REG_FIFOSTAT,0x03);
	
		//Set power mode to FULLRX
	    ax5043_writeReg(AX_REG_PWRMODE,PWRMODE_FULLRX);
	
	    printf("Receiving...\n");
	    while (rstat != AX_REG_RADIOSTATE_RX_MASK) {
		    rstat = ax5043_readReg(AX_REG_RADIOSTATE);
		    //if (rstat == AX_REG_RADIOSTATE_RX_PREAMBLE_2_MASK) {
				//printf("Received %02x :\n",rstat);
				//usleep(1000);
			//} else if (rstat == AX_REG_RADIOSTATE_RX_PREAMBLE_3_MASK) {
				//printf("Received %02x :\n",rstat);
				//usleep(1000);
				
			//}
	    }
	    while (rstat == AX_REG_RADIOSTATE_RX_MASK) {
		    rstat = ax5043_readReg(AX_REG_RADIOSTATE);
		    usleep(100);
	    }
	    
	    fif0 = ax5043_readReg(AX_REG_FIFOCOUNT0);
		fif1 = ax5043_readReg(AX_REG_FIFOCOUNT1); 
	    FIFObytes = (fif1 << 8) | fif0;
	
	    received++;
	    printf("Received DATA :");
		for (i = 0; i < FIFObytes; i++)
		{
			reg = ax5043_readReg(AX_REG_FIFODATA);	
			printf(" %02x",reg);
		}
	    printf(" ... Packet No : %u\n",received);
	    //Set power mode to POWERDOWN
	    ax5043_writeReg(AX_REG_PWRMODE,PWRMODE_POWERDOWN);
	}
}
