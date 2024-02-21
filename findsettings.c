/*----------------------------------------------------------------------
 * Copyright (c) 2017, 2024 XIA LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, 
 * with or without modification, are permitted provided 
 * that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above 
 *     copyright notice, this list of conditions and the 
 *     following disclaimer.
 *   * Redistributions in binary form must reproduce the 
 *     above copyright notice, this list of conditions and the 
 *     following disclaimer in the documentation and/or other 
 *     materials provided with the distribution.
 *   * Neither the name of XIA LLC
 *     nor the names of its contributors may be used to endorse 
 *     or promote products derived from this software without 
 *     specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
 * SUCH DAMAGE.
 *----------------------------------------------------------------------*/
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <sys/mman.h>
#include <math.h>
// need to compile with -lm option

#include "PixieNetDefs.h"
#include "PixieNetCommon.h"
#include "PixieNetConfig.h"



// global variables
unsigned int Random_Set[NTRACE_SAMPLES];			// Random indices used by TauFinder and BLCut

// subroutines
int Tau_Finder (
				volatile unsigned int *mapped,
            unsigned int ch,		       // Pixie channel number       
            unsigned FL, 
            unsigned FG,
            double   xdt,
				double *Tau );		          // Tau value

double Tau_Fit (
				unsigned int  *Trace,	    // ADC trace data
				unsigned int  kmin,		    // lower end of fitting range
				unsigned int  kmax,		    // uuper end of fitting range
				double dt );		          // sampling interval of ADC trace data

double Phi_Value (
				unsigned int  *ydat,	       // source data for search
				double qq,		             // search parameter
				unsigned int  kmin,	       // search lower limit
				unsigned int  kmax );	    // search upper limit

double Thresh_Finder (
				unsigned int *Trace,	       // ADC trace data
				double *Tau,		          // Tau value
				double *FF,		             // return values for fast filter
				double *FF2,		          // return values for fast filter
				unsigned int  FL,	          // fast length
				unsigned int  FG,	          // fast gap
				double dt );		          // xdt

int RandomSwap(void);

unsigned int RoundOff(double x);

#ifndef MAX
   #define MAX(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
   #define MIN(a,b)            (((a) < (b)) ? (a) : (b))
#endif

int main(void) {

  int fd;
  void *map_addr;
  int size = 4096;
  volatile unsigned int *mapped;
  int k, addr, ch, adc, dac;
  unsigned int mval;

  unsigned int mins[NCHANNELS] = {4192,4192,4192,4192};
  unsigned int mint[NCHANNELS] = {4192,4192,4192,4192};
  unsigned int readdr[NCHANNELS] = {AADC0,AADC1,AADC2,AADC3};
  unsigned int targetdac[NCHANNELS] = {0,0,0,0};
  unsigned int targetBL[NCHANNELS] = {400,400,400,400};     // defaults will be overwritten with values from settings file
  double dacadj;
  unsigned int oldadc, adcchanged[NCHANNELS], saveaux;
  unsigned int OBsave, csr;
  unsigned int revsn, rev;
  unsigned int ADCmax=4000;
  int verbose=1;

  unsigned int GOOD_CH[NCHANNELS], FL[NCHANNELS], FG[NCHANNELS];
  double Tau, xdt[NCHANNELS];




  // *************** PS/PL IO initialization *********************
  // open the device for PD register I/O
  fd = open("/dev/uio0", O_RDWR);
  if (fd < 0) {
    perror("Failed to open devfile");
    return 1;
  }

  map_addr = mmap( NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

  if (map_addr == MAP_FAILED) {
    perror("Failed to mmap");
    return 1;
  }

  mapped = (unsigned int *) map_addr;


  // ******************* XIA code begins ********************


   // check if run is in progress
   OBsave = mapped[AOUTBLOCK];
   mapped[AOUTBLOCK] = OB_EVREG;
   csr = mapped[ACSROUT];
   if(csr & 0x1)          // test runenable bit
   {
      printf("This function can not be executed while a run is in progress. CSRout = 0x%x.",csr);
      mapped[AOUTBLOCK] = OBsave;
      return(-1);
   }

     // ******************* read ini file and fill struct with values ********************
  
  PixieNetFippiConfig fippiconfig;		// struct holding the input parameters
  const char *defaults_file = "defaults.ini";
  int rval;
  rval = init_PixieNetFippiConfig_from_file( defaults_file, 0, &fippiconfig );   // first load defaults, do not allow missing parameters
  if( rval != 0 )
  {
    printf( "Failed to parse FPGA settings from %s, rval=%d\n", defaults_file, rval );
    return rval;
  }
  const char *settings_file = "settings.ini";
  rval = init_PixieNetFippiConfig_from_file( settings_file, 2, &fippiconfig );   // second override with user settings, do allow missing, no warning
  if( rval != 0 )
  {
    printf( "Failed to parse FPGA settings from %s, rval=%d\n", settings_file, rval );
    return rval;
  }

 // ***** check HW info *********
  revsn = hwinfo(mapped);
  rev = (revsn>>16) & 0xFFFF;
   if ( (rev == PN_BOARD_VERSION_12_250_A)     ||
        (rev == PN_BOARD_VERSION_12_250_B)     ||
        (rev == PN_BOARD_VERSION_12_250_B_PTP) )
   { 
      printf("Using offset targets for 12 bit version\n");
   } 
   else 
   {
      printf("Using offset targets for 14 bit version\n");
      for( ch = 0; ch < NCHANNELS; ch ++ )
      {
         mins[ch] = mins[ch]*4;
         mint[ch] = mint[ch]*4;
         targetBL[ch] = targetBL[ch]*4;
      }
      ADCmax = ADCmax *4;
   }
 

  // shorthand a few parameters
  for( ch = 0; ch < NCHANNELS; ch ++ )
  {
      GOOD_CH[ch]  = ( fippiconfig.CHANNEL_CSRA[ch] & (1<<2) ) >0;               // CCSRA_GOOD is bit 2
      targetBL[ch] = (unsigned int)floor((double)ADCmax*(double)fippiconfig.BASELINE_PERCENT[ch]/100.0);
      xdt[ch]      = (double)fippiconfig.ADC_AVG[ch]/ADC_CLK_MHZ;                //    using triggered traces with known dt  (in us)   
      FL[ch]       = (int)floorf(fippiconfig.TRIGGER_RISETIME[ch] * FILTER_CLOCK_MHZ);
      FG[ch]       = (int)floorf(fippiconfig.TRIGGER_FLATTOP[ch] * FILTER_CLOCK_MHZ);
  }


  mapped[AOUTBLOCK] = OB_IOREG;	// read from IO block
  saveaux = mapped[AAUXCTRL];
  mapped[AAUXCTRL] = saveaux & 0x0001;        // turn off LED and any other stuff, but keep on pulser
  mapped[ACSRIN] = 0x0000;       // all off

  // ----------- swap channels 0<>1 and 2<>3 if necessary  -------------
  printf("Checking for swapped channels ...\n");
  


   if(1){ // new style using ADC control
   // SPI register write
   // bit 31:26: unused
   // bit 25:24: ADC chip select 
   // bits 23:16: upper 8 bits of serial data (R/nW WW A12..A8)
   // bits 15:0: lower 16 bits of serial data (A7... A0 D7 ...D0)
   // Note: pin sharing means that ADC SPI can only operate if red LED is off (AUXCTRL bit 1 =0)
   
      mapped[AADCCTRL] = 0;  // start from known swap status
      
      
      // program one ADC output to known value, check if that comes through
      // Note: this is now quick enough so that it could be done as part of booting or progfippi
      
      // set the test pattern      
      for( ch = 0; ch < NCHANNELS; ch = ch+2 )    // check every other channel
      {
      
         if(ch==0) addr =  0x01000000;  // write, chip 0
         if(ch==2) addr =  0x02000000;  // write, chip 1
         
         mval = addr + 0x00000502;  // address 0x0005 (channel select), value 0x02 (ch.B only)  (ADC "B" = PN 0 or 2)
         mapped[ASPI] = mval;	
         usleep(100);
         mval = addr + 0x00000D04;  // address 0x000D (test IO), value 0x04 (checkerboard)
         //mval = addr + 0x00000D0F;  // address 0x000D (test IO), value 0x0F (ramp)
         mapped[ASPI] = mval;
         //printf("SPI for test pattern, ch %d = 0x%06x\n", ch, mval);
         usleep(100);
         mval = addr + 0x0000FF01;  // write to the transfer register (0xFF) to apply (0x01)
         mapped[ASPI] = mval;	
         usleep(100);
      
      } // endfor  channels
      
      
      // read the ADC back
      mapped[AOUTBLOCK] = OB_EVREG;	      
      for( ch = 0; ch < NCHANNELS; ch = ch+2 )     // check every other channel
      {
         if(ch==0) addr = AADC0;
         if(ch==1) addr = AADC1;
         if(ch==2) addr = AADC2;
         if(ch==3) addr = AADC3;
         
         adc = mapped[addr] & 0xFFF;                           // read ADC data from MZ
         if(verbose) printf(" Channel %u: ADC value = %d (0x%x)\n",ch, adc, adc);
         if((adc==0xAAA) | (adc==0x555))  adcchanged[ch]++ ;     // check if it's one of the expected values
         adc = mapped[addr] & 0xFFF;                            // read ADC data from MZ
         if(verbose) printf(" Channel %u: ADC value = %d (0x%x)\n",ch, adc, adc);
         if((adc==0xAAA) | (adc==0x555))  adcchanged[ch]++ ;     // check if it's one of the expected values
         adc = mapped[addr] & 0xFFF;                            // read ADC data from MZ
         if(verbose) printf(" Channel %u: ADC value = %d (0x%x)\n",ch, adc, adc);
         if((adc==0xAAA) | (adc==0x555))  adcchanged[ch]++ ;     // check if it's one of the expected values
         adc = mapped[addr] & 0xFFF;                            // read ADC data from MZ
         if(verbose) printf(" Channel %u: ADC value = %d (0x%x)\n",ch, adc, adc);
         if((adc==0xAAA) | (adc==0x555))  adcchanged[ch]++ ;     // check if it's one of the expected values
      
         if (adcchanged[ch] < 3)  {             // sometimes the first read is old data
            if (ch<2)  {
               mval =  mapped[AADCCTRL];
               mval = mval ^ 0x0001;
               mapped[AADCCTRL] = mval;	  // swap 0/1
            } else {
               mval =  mapped[AADCCTRL];
               mval = mval ^ 0x0002;
               mapped[AADCCTRL] = mval;	  // swap 2/3
            }
            printf(" Channel %u: ADC values do not change with DAC. Swapped channel inputs\n",ch);
         }   // end unchanged     
      
      } // endfor  channels
      
      
      // undo the test pattern   
      for( ch = 0; ch < NCHANNELS; ch = ch+2 )    // check every other channel
      {
      
         if(ch==0) addr =  0x01000000;  // write, chip 0
         if(ch==2) addr =  0x02000000;  // write, chip 1
         
         mval = addr + 0x00000503;  // address 0x0005 (channel select), value 0x03 (both)
         mapped[ASPI] = mval;	
         usleep(100);
         mval = addr + 0x00000D00;  // address 0x000D (test IO), value 0x00 (normal)
         mapped[ASPI] = mval;
         //printf("SPI for test pattern, ch %d = 0x%06x\n", ch, mval);
         usleep(100);
         mval = addr + 0x0000FF01;  // write to the transfer register (0xFF) to apply (0x01)
         mapped[ASPI] = mval;	
         usleep(100);     
      
      } // endfor  channels
   
   } // end new style


  if(0)  // old style check with DAC ramping
  {
     mapped[AOUTBLOCK] = OB_IOREG;	  // read from IO block
     for( ch = 0; ch < NCHANNELS; ch ++ )
     {
         // find if change of DAC changes ADC
        adc = 0;
        oldadc = 0;
        adcchanged[0] = 0;
        addr = N_PL_IN_PAR+ch*N_PL_IN_PAR+CA_DAC;   // channel registers begin after NPLPAR system registers, NPLPAR each
        dac = 0; 
        k=0;
   
        // scan through DAC settings
        do {
            mapped[AOUTBLOCK] = OB_IOREG;	        // read from IO block
            mapped[addr] = dac;
            usleep(DACWAIT);
            mapped[addr] = dac; //;               // TODO: double write required?
            if(mapped[addr] != dac) printf("Error writing parameters to DAC register\n");
            usleep(DACSETTLE);		              // wait for DAC's RC filter
   
            mapped[AOUTBLOCK] = OB_EVREG;		     // switch reads to event data block of addresses
            adc = (mapped[readdr[ch]] & 0xFFFF);  // dummy read to refresh read register
            adc = (mapped[readdr[ch]] & 0xFFFF);
            if (k==0)   {
               oldadc = adc;
            } else {
               if ( abs(oldadc-adc)>200)  adcchanged[0] = 1;  // look for a change > 200 steps. Not foolproof with pulses!
            }
            k=k+1;
            dac = dac+4096;
         //   printf("Channel %u: DAC value %u, adc %u, adcdiff %d\n",ch,dac,adc,abs(oldadc-adc));
        } while ( (adcchanged[0]==0) & (k<16) );   //  dac loop
   
        // check if there was a change, if not, swap channels
        if (k==16)  {
            if (ch<2)  {
               mval =  mapped[AADCCTRL];
               mval = mval ^ 0x0001;
               mapped[AADCCTRL] = mval;	  // swap 0/1
            } else {
               mval =  mapped[AADCCTRL];
               mval = mval ^ 0x0002;
               mapped[AADCCTRL] = mval;	  // swap 2/3
            }
            printf(" Channel %u: ADC values does not change with DAC. Swapped channel inputs\n",ch);
        }
   
     } // endfor  channels

     // restore offset DAC for tau finder
     for( ch = 0; ch < NCHANNELS; ch ++ )
     {
        addr = N_PL_IN_PAR+ch*N_PL_IN_PAR+CA_DAC;   // channel registers begin after NPLPAR system registers, NPLPAR each
        
        dac = (int)floor( (1 - fippiconfig.VOFFSET[ch]/ V_OFFSET_MAX) * 32768);	
        mapped[addr] = dac;
        usleep(DACWAIT);		 // wait for programming
        mapped[addr] = dac;    // TODO: double write required?
        usleep(DACSETTLE);		 // wait for DAC's RC filter

     }

  } //end old style with ramp
  
  
  // ----------- need to have correct polarity  -------------

  // TODO!

  // ----------- tau finder  -------
   
  if(1)     // optionally skip this (for debug)
  {
      printf("\nDetermining decay time TAU (correct polarity required) ...\n"); 
      printf(" Tau finder depends on \n");
      printf("  - current TAU (for minimum fit range)\n");
      printf("  - ADC_AVG (for extend of ADC data)\n");
      printf("  - THRESH_ADC_AVG (absolute ADC level) to capture pulses \n");      
      printf(" Suggested TAU values for settings file (if 0, ignore)\n  ");
         
         for( ch = 0; ch < NCHANNELS; ch++ ) 
         {
       //  ch=3;  
            //printf("TauFinder: channel %d \n", ch);
            Tau = fippiconfig.TAU[ch];
            if(GOOD_CH[ch])
               Tau_Finder ( mapped, ch, FL[ch], FG[ch], xdt[ch], &Tau );
            else
               Tau = 0.0;  // report value that indicates not found
            printf("  %4.3f ", Tau);
         }   // end for

       printf(" \n");
   } // end debug switch
  

  // ----------- adjust offset: search for two DAC settings with valid ADC response, then extrapolate  -------
  printf("\nAdjusting DC offsets (correct polarity required) ...\n");
  printf(" target BL (ch.0) %d \n", targetBL[0]);  
  for( ch = 0; ch < NCHANNELS; ch ++ )
  {
     dac = 0;
     adc = 0;
     addr = N_PL_IN_PAR+ch*N_PL_IN_PAR+CA_DAC;   // channel registers begin after NPLPAR system registers, NPLPAR each
     k=0;

     // 1. find first DAC value with valid response
     do  {
         mapped[AOUTBLOCK] = OB_IOREG;	   // read from IO block
         mapped[addr] = dac; 
         usleep(DACWAIT);
         mapped[addr] = dac;              // TODO: double write required?
         if(mapped[addr] != dac) printf("Error writing parameters to DAC register\n");
         usleep(DACSETTLE);		         // wait for DAC's RC filter

         mapped[AOUTBLOCK] = OB_EVREG;		// switch reads to event data block of addresses
         adc = (mapped[readdr[ch]] & 0xFFFF);  // dummy read to refresh read register
         adc = (mapped[readdr[ch]] & 0xFFFF);

         //printf("Channel %u: addr 0x%x, DAC value %u, adc %u\n",ch,addr,dac,adc);
         dac = dac + 2048;
         k=k+1;
     } while ( ((adc>ADCmax) | (adc<100)) & (dac < 65536)  );    //& (k<33)
     //printf("Channel %u: DAC value %u, adc %u\n",ch,dac,adc);
     dac = dac - 2048;               // dac is now the lowest valid DAC value
  

     // 2. get min/max of many samples
     for( k = 0; k < NTRACE_SAMPLES; k ++ )   {
         adc = (mapped[readdr[ch]] & 0xFFFF);
         //avg[ch] = avg[ch]+ adc/NTRACE_SAMPLES;   // find average
         if (adc < mins[ch])  mins[ch] = adc;    // find min
         //if (adc > maxs[ch])  maxs[ch] = adc;    // find max
     }
    // printf("Channel %u: DAC value %u, min adc read %u\n",ch,dac,mins[ch]);

     // 3. change DAC settings
     mapped[AOUTBLOCK] = OB_IOREG;	 // read from IO block
     dac = dac + 1024;               // new, second dac
     mapped[addr] = dac;
     usleep(DACWAIT);		          // wait for programming
     mapped[addr] = dac;             // TODO: double write required?
     usleep(DACSETTLE);		          // wait for DAC's RC filter

     // 4. get min/max of many samples
     mapped[AOUTBLOCK] = OB_EVREG;		      // switch reads to event data block of addresses
     adc = (mapped[readdr[ch]] & 0xFFFF);    // dummy read to refresh read register
      for( k = 0; k < NTRACE_SAMPLES; k ++ )   {
         adc = (mapped[readdr[ch]] & 0xFFFF);
        // avg[ch] = avg[ch]+ adc/NTRACE_SAMPLES;   // find average
         if (adc < mint[ch])  mint[ch] = adc;    // find min
        // if (adc > maxs[ch])  maxs[ch] = adc;    // find max
     }
     //printf("Channel %u: DAC value %u, min adc read %u\n",ch,dac,mint[ch]);

     // 5. compute target dac from 2 points
     dacadj =  1024.0 * ((double)targetBL[ch] - (double)mint[ch]) / ((double)mint[ch] - (double)mins[ch]);
     dac = dac + (int)floor(dacadj);
     if( (dac>0) & (dac<65536) )
     {
        //printf("Channel %u: DAC adjustment %f\n",ch,dacadj  );
        targetdac[ch]  = dac;
     } else {
          printf(" Channel %u: could not find target DAC value\n",ch);
     }
      
   }    // endfor

   // 6. set all channels to target and report voltages
   for( ch = 0; ch < NCHANNELS; ch ++ )
   {
        addr = N_PL_IN_PAR+ch*N_PL_IN_PAR+CA_DAC;   // channel registers begin after NPLPAR system registers, NPLPAR each
        
        mapped[AOUTBLOCK] = OB_IOREG;	  // read from IO block
        dac = targetdac[ch];	
        mapped[addr] = dac;
        usleep(DACWAIT);		          // wait for programming
        mapped[addr] = dac;             // TODO: double write required?
        usleep(DACSETTLE);		          // wait for DAC's RC filter

        mapped[AOUTBLOCK] = OB_EVREG;		      // switch reads to event data block of addresses
        adc = (mapped[readdr[ch]] & 0xFFFF);    // dummy read to refresh read register
        adc = (mapped[readdr[ch]] & 0xFFFF);  
        printf(" Channel %u: DAC value %u, offset %fV, ADC %u\n",ch,dac,V_OFFSET_MAX*(1.0-(double)dac/32678.0), adc);
   }

   mapped[AOUTBLOCK] = OB_IOREG;	  // read from IO block
   mapped[AAUXCTRL] = saveaux;     // turn on pulser TODO: read from file!
    
   printf(" Suggested VOFFSET values for settings file \n  ");
   for( ch = 0; ch < NCHANNELS; ch ++ )
   {
      printf(" %6.3f",V_OFFSET_MAX*(1.0-(double)targetdac[ch]/32678.0));
   }
    printf("\n");

 // clean up  
 munmap(map_addr, size);
 close(fd);
 return 0;
}






// ----------------------------------------------------------------------------------------
// tau finder subroutines from Pixie-4 
// ----------------------------------------------------------------------------------------

/****************************************************************
*	Tau_Finder function:
*		Find the exponential decay constant of the detector/preamplifier
*		signal connected to one channel of a Pixie module.
*			
*		Tau is both an input and output parameter: 
*     it is used as the initial guess of Tau enforcing a min. fit region of 3*tau
*     then used for returning the new Tau value (average of up to 10 successful fits).
*
*		Return Value:
*			 0 - success
*			-1 - failure to acquire ADC traces
*
****************************************************************/

int Tau_Finder (
				volatile unsigned int *mapped,
            unsigned int ch,			// Pixie channel number calling function must loop over all channels, ignore not "GOOD" channels
            unsigned FL, 
            unsigned FG,
            double  xdt,      // in us
				double *Tau )		// Tau value in us
{

	unsigned int  Trace[NTRACE_SAMPLES];
	unsigned int  TFcount;                
	unsigned int  ndat, k, kmin, kmax, n, tcount, MaxTimeIndex, Ntaufound;
	unsigned int  Trig[NTRACE_SAMPLES];
	double threshold, t0, t1, TriggerLevelShift, avg, MaxTimeDiff, fitted_tau;
	double FF[NTRACE_SAMPLES], FF2[NTRACE_SAMPLES], TimeStamp[NTRACE_SAMPLES/4];
	double input_tau, Tau_avg, dt;
   unsigned int dn, addr;  
	int mval;
   double scale=1;
   FILE * fil;     // for debug
   int debug=0;
   int maxwait = 20;

	/* Save input Tau value */
	input_tau=*Tau * 1e-6;      // in seconds

	/* Generate random indices */
	RandomSwap();




   ndat=NAVG_TRACE_SAMPLES;
   dn = (int)floor( xdt*FILTER_CLOCK_MHZ);   // in samples
   dt = xdt * (1e-6);                        // in seconds
   Ntaufound = 0;

   scale = (double)dn; // start with number of samples averaged into one value  // (double)fippiconfig.ADC_AVG[k];
   if(dn > 2048)
      scale = scale/16384.0;               // rescale for larger averages
   else if(dn > 64)
      scale = scale/128.0;                 // rescale for medium averages


   maxwait = dn;
   maxwait = maxwait * NAVG_TRACE_SAMPLES * 50;            // 50 times  max total sampling time   (in clock cycles)
   maxwait = (int)floor(maxwait * 8 / 100000);             // scale in 100 us wait cycles

   //printf("TauFinder: channel %d, FL %d, FG %d, xdt %6.4f, dt %e, dn %d, initial Tau %f, scale %f\n", ch, FL, FG, xdt, dt, dn, *Tau, scale);
   


   TFcount=0;  /* Initialize TFcount */
   Ntaufound = 0;
   Tau_avg = 0;
   do
   {
      /* get ADC trace -- buffered FIFO read */
            
      mapped[AOUTBLOCK] = OB_IOREG;	
      mapped[ACOUNTER_CLR] = 1;	      // any write to COUNTER_CLR arms the trigger for capturing averaged samples

      // poll for capture to be finished
      //maxwait=3;     //debug
      mapped[AOUTBLOCK] = OB_EVREG;		// switch reads to event data block of addresses
      k=0; 
      do {          
         usleep(1000);                    
         k=k+1;
         mval = mapped[AADCTRIG];
         //printf("ADCTRIG = 0x%x \n", mval);
      }  
      while ( ((mval & (0x0010<<ch)) == 0) & (k< maxwait) );      // channel triggered and is done
      if(k>=maxwait)
      {
         //printf("Error: Ch. %d: Waiting for trigger timed out \n",ch);
         TFcount ++;
         continue;
         //return -1;
      }

     
      // read samples
      if(ch==0) addr = AAVGADC0;
      if(ch==1) addr = AAVGADC1;
      if(ch==2) addr = AAVGADC2;
      if(ch==3) addr = AAVGADC3;

      // dummy read for sampling update
      k = mapped[addr] & 0xFFFF;
      
      for( k = 0; k < NAVG_TRACE_SAMPLES; k ++ )
      {
            Trace[k] = (int)floor((double)mapped[addr]/scale);
      }     
      //printf("\nTauFinder: Trace %d %d %d %d %d %d %d %d\n", Trace[4], Trace[5],Trace[6],Trace[7],Trace[8],Trace[9],Trace[10],Trace[11]);
      
      // debug: save the trace
      if(debug)
      {
         // open the output file
         fil = fopen("ADC.csv","w");
         fprintf(fil,"sample, adc0\n");
         
         //  write to file
         for( k = 0; k < NAVG_TRACE_SAMPLES; k ++ )
         {
            //fprintf(fil,"%d",k);                  // sample number
            fprintf(fil,"%f",k*xdt );           // time in us
            fprintf(fil,",%d",Trace[k]);    // print channel data
            fprintf(fil,"\n");
         }
      } // end debug

		/* Find threshold */          
		threshold=Thresh_Finder(Trace, Tau, FF, FF2, FL, FG, dt);
      if(debug) printf("TauFinder:  threshold %f\n", threshold);

      /* find triggers (rising edges) */   

      // initialize
		kmin=2*FL+FG;
		for(k=0;k<kmin;k+=1) Trig[k]= 0;

		// Find average FF shift 
		avg=0.0;
		n=0;
		for(k=kmin;k<(ndat-1);k+=1)
		{
			if((FF[k+1]-FF[k])<threshold)
			{
				avg+=FF[k];
				n+=1;
			}
		}

		avg/=n;
		for(k=kmin;k<(ndat-1);k+=1)
		{
			FF[k]-=avg;
		}

		for(k=kmin;k<(ndat-1);k+=1)  /* look for rising edges */
		{
			Trig[k]= (FF[k]>threshold)?1:0;
		}

		tcount=0;
		for(k=kmin;k<(ndat-1);k+=1)  /* record trigger times */
		{
			if((Trig[k+1]-Trig[k])==1)
			{
				TimeStamp[tcount++]=k+2;  /* there are tcount triggers */
			}
		}
		if(debug) printf("*INFO* (Tau_Finder): found  %d triggers\n", tcount);

      /* select the best pulse to fit */
		if(tcount>2)
		{
			TriggerLevelShift=0.0;
			for(n=0; n<(tcount-1); n+=1)
			{
				avg=0.0;
				kmin=(unsigned int )(TimeStamp[n]+2*FL+FG);
				kmax=(unsigned int )(TimeStamp[n+1]-1);
				if((kmax-kmin)>0)
				{
					for(k=kmin;k<kmax;k+=1)
					{
						avg+=FF2[k];
					}
				}
				TriggerLevelShift+=avg/(kmax-kmin);
			}
			TriggerLevelShift/=tcount;
		}

		switch(tcount)
		{
   		case 0:
   			// Increment TFcount 
   			TFcount ++;
   			continue;
   		case 1:
   			t0=TimeStamp[0]+2*FL+FG;
   			t1=ndat-2;
   			break;
   		default:
   			MaxTimeDiff=0.0;
   			for(k=0;k<(tcount-1);k+=1)
   			{
   				if((TimeStamp[k+1]-TimeStamp[k])>MaxTimeDiff)
   				{
   					MaxTimeDiff=TimeStamp[k+1]-TimeStamp[k];
   					MaxTimeIndex=k;
   				}
   			}
   
   			if((ndat-TimeStamp[tcount-1])<MaxTimeDiff)
   			{
   				t0=TimeStamp[MaxTimeIndex]+2*FL+FG;
   				t1=TimeStamp[MaxTimeIndex+1]-1;
   			}
   			else
   			{
   				t0=TimeStamp[tcount-1]+2*FL+FG;
   				t1=ndat-2;
   			}


   			break;
		}  // end switch

		if(debug) printf("*INFO* (Tau_Finder): boundaries at points %6.2f  and %6.2f\n   (time %6.2f and %6.2f)\n", t0,t1, t0*xdt, t1*xdt);
		if(((t1-t0)*dt)<3*(input_tau))
		{
         if(debug) printf("*INFO* (Tau_Finder): interval too small, try again \n");
			// Increment TFcount 
			TFcount ++;
			continue;
		}

      /* fit the trace segment */
   	fitted_tau=Tau_Fit(Trace, (unsigned int )t0, (unsigned int )t1, dt);
		if(fitted_tau > 0)	// Check if returned Tau value is valid 
		{
			//*Tau=fitted_tau;
         Tau_avg=Tau_avg+fitted_tau;
         Ntaufound ++;
         if(debug) printf("*INFO* (Tau_Finder):found tau = %f (us)\n",fitted_tau*1e6);

		}


		TFcount ++;

	//} while((*Tau == input_tau) && (TFcount < 10)); /* Try 10 times at most to get a valid Tau value */
   } while(TFcount < 20); /* Try 20 times no matter what */
   //} while(Ntaufound <1 && TFcount < 10);   // stop at first valid

         if(Ntaufound>0)
            *Tau=Tau_avg/Ntaufound*1e6;       // then use average
         else
            *Tau = 0.0;
         if(debug) printf("*INFO* (Tau_Finder):found average tau = %f (us) from %d traces\n",Tau_avg/Ntaufound*1e6, Ntaufound);

	return(0);

}




/****************************************************************
*	Tau_Fit function:
*		Exponential fit of the ADC trace.
*
*		Return Value:
*			Tau value if successful
*			-1 - Geometric search did not find an enclosing interval
*			-2 - Binary search could not find small enough interval
*
****************************************************************/

double Tau_Fit (
				unsigned int  *Trace,		// ADC trace data
				unsigned int  kmin,		// lower end of fitting range
				unsigned int  kmax,		// uuper end of fitting range
				double dt )		// sampling interval in seconds
{
	double mutop,mubot,valbot,eps,dmu,mumid,valmid; // valtop
	unsigned int  count;
  // double dt;

   //dt = xdt*1e-6;      // xdt is in us
	eps=1e-3;
	mutop=10e6; /* begin the search at tau=100ns (=1/10e6) */
	//valtop=Phi_Value(Trace,exp(-mutop*dt),kmin,kmax);
	mubot=mutop;
	count=0;

   //printf( "*INFO* (Tau_Fit): kmin %d, kmax %d\n",kmin, kmax);

	do  /* geometric progression search */
	{
	//	printf( "*INFO* (Tau_Fit): mubot %e, mutop %e, valbot %e\n",mubot, mutop, valbot);
      mubot=mubot/2.0;
		valbot=Phi_Value(Trace,exp(-mubot*dt),kmin,kmax);
		count+=1;
		if(count>20)
		{
			//printf( "*ERROR* (Tau_Fit): geometric search did not find an enclosing interval\n");
			return(-1);
		}	/* Geometric search did not find an enclosing interval */
      //printf( "*INFO* (Tau_Fit): mubot %e, mutop %e, valbot %e\n",mubot, mutop, valbot);
	} while(valbot>0);	/* tau exceeded 100ms */

   //printf( "*INFO* (Tau_Fit): after geo: mubot %e, mutop %e, valbot %e\n",mubot, mutop, valbot);
	mutop=mubot*2.0;
	//valtop=Phi_Value(Trace,exp(-mutop*dt),kmin,kmax);
	count=0;
	do  /* binary search */
	{
		
      mumid=(mutop+mubot)/2.0;
		valmid=Phi_Value(Trace,exp(-mumid*dt),kmin,kmax);
		if(valmid>0)
		{
			mutop=mumid;
		}
		else
		{
			mubot=mumid;
		}

		dmu=mutop-mubot;
		count+=1;
		if(count>20)
		{
			//printf("*ERROR* (Tau_Fit): Binary search could not find small enough interval\n");
			return(-2);  /* Binary search could not find small enough interval */
		}
      //printf( "*INFO* (Tau_Fit): mumid %e, valmid %e\n", mumid, valmid);
	} while(fabs(dmu/mubot) > eps);

	return(1/mutop);  /* success */
}


/****************************************************************
*	Phi_Value function:
*		geometric progression search.
*
*		Return Value:
*			search result
*
****************************************************************/

double Phi_Value (
				  unsigned int  *ydat,		// source data for search
				  double qq,		// search parameter
				  unsigned int  kmin,		// search lower limit
				  unsigned int  kmax )		// search upper limit
{
	int ndat;
	double s0,s1,s2,qp;
	double A,B,Fk,F2k,Dk,Ek,val;
	unsigned int  k;

	ndat=kmax-kmin+1;
	s0=0; s1=0; s2=0;
	qp=1;

	for(k=kmin;k<=kmax;k+=1)
	{
		s0+=ydat[k];
		s1+=qp*ydat[k];
		s2+=qp*ydat[k]*(k-kmin)/qq;
		qp*=qq;
	}

	Fk=(1-pow(qq,ndat))/(1-qq);
	F2k=(1-pow(qq,(2*ndat)))/(1-qq*qq);
	Dk=-(ndat-1)*pow(qq,(2*ndat-1))/(1-qq*qq)+qq*(1-pow(qq,(2*ndat-2)))/pow((1-qq*qq),2);
	Ek=-(ndat-1)*pow(qq,(ndat-1))/(1-qq)+(1-pow(qq,(ndat-1)))/pow((1-qq),2);
	A=(ndat*s1-Fk*s0)/(ndat*F2k-Fk*Fk) ;
	B=(s0-A*Fk)/ndat;

	val=s2-A*Dk-B*Ek;

	return(val);

} 

/****************************************************************
*	Thresh_Finder function:
*		Threshold finder used for Tau Finder function.
*
*		Return Value:
*			Threshold
*
****************************************************************/

double Thresh_Finder (
					  unsigned int  *Trace,		// ADC trace data
					  double *Tau,		// Tau value
					  double *FF,		// return values for fast filter
					  double *FF2,		// return values for fast filter
					  unsigned int  FL,			// fast length
					  unsigned int  FG,			// fast gap
					  double dt )		// samoming interval in seconds
{

	unsigned int  ndat,kmin,k,ndev,n,m;
	double xx,c0,sum0,sum1,deviation,threshold;


	ndev=8;		/* threshold will be 8 times sigma */
	ndat=NTRACE_SAMPLES;

	// sprintf(str,"XWAIT%d",ChanNum);
	// idx=Find_Xact_Match(str, DSP_Parameter_Names, N_DSP_PAR);
	// Xwait=(double)Pixie_Devices[ModNum].DSP_Parameter_Values[idx];
   // dt=Xwait/SYSTEM_CLOCK_MHZ*1e-6;
	xx=dt/(*Tau);
	c0=exp(-xx*(FL+FG));

	kmin=2*FL+FG;
	/* zero out the initial part,where the true filter values are unknown */
	for(k=0;k<kmin;k+=1)
	{
		FF[k]=0;
	}

	for(k=kmin;k<ndat;k+=1)
	{
		sum0=0;	sum1=0;
		for(n=0;n<FL;n++)
		{
			sum0+=Trace[k-kmin+n];
			sum1+=Trace[k-kmin+FL+FG+n];
		}
		FF[k]=sum1-sum0*c0;
	}

	/* zero out the initial part,where the true filter values are unknown */
	for(k=0;k<kmin;k+=1)
	{
		FF2[k]=0;
	}

	for(k=kmin;k<ndat;k+=1)
	{
		sum0=0;	sum1=0;
		for(n=0;n<FL;n++)
		{
			sum0+=Trace[k-kmin+n];
			sum1+=Trace[k-kmin+FL+FG+n];
		}
		FF2[k]=(sum0-sum1)/FL;
	}

	deviation=0;
	for(k=0;k<ndat;k+=2)
	{
		deviation+=fabs(FF[Random_Set[k]]-FF[Random_Set[k+1]]);
	}

	deviation/=(ndat/2);
	threshold=ndev/2*deviation/2;

	m=0; deviation=0;
	for(k=0;k<ndat;k+=2)
	{
		if(fabs(FF[Random_Set[k]]-FF[Random_Set[k+1]])<threshold)
		{
			m+=1;
			deviation+=fabs(FF[Random_Set[k]]-FF[Random_Set[k+1]]);
		}
	}
	deviation/=m;
	deviation*=sqrt(PI)/2;
	threshold=ndev*deviation;

	m=0; deviation=0;
	for(k=0;k<ndat;k+=2)
	{
		if(fabs(FF[Random_Set[k]]-FF[Random_Set[k+1]])<threshold)
		{
			m+=1;
			deviation+=fabs(FF[Random_Set[k]]-FF[Random_Set[k+1]]);
		}
	}

	deviation/=m;
	deviation*=sqrt(PI)/2;
	threshold=ndev*deviation;

	m=0; deviation=0;
	for(k=0;k<ndat;k+=2)
	{
		if(fabs(FF[Random_Set[k]]-FF[Random_Set[k+1]])<threshold)
		{
			m+=1;
			deviation+=fabs(FF[Random_Set[k]]-FF[Random_Set[k+1]]);
		}
	}
	deviation/=m;
	deviation*=sqrt(PI)/2;
	threshold=ndev*deviation;

	return(threshold);
}


/****************************************************************
*	RandomSwap function:
*		Generate a random set. The size of the set is NTRACE_SAMPLES.
*
****************************************************************/

int RandomSwap(void)
{

	unsigned int  rshift,Ncards;
	unsigned int  k,MixLevel,imin,imax;
	unsigned int  a;

	for(k=0; k<NTRACE_SAMPLES; k++) Random_Set[k]=(unsigned int )k;

	Ncards=NTRACE_SAMPLES;
	rshift= (unsigned int )(log(((double)RAND_MAX+1.0)/(double)NTRACE_SAMPLES)/log(2.0));
	MixLevel=5;

	for(k=0; k<MixLevel*Ncards; k++)
	{
		imin=(rand()>>rshift); 
		imax=(rand()>>rshift);
		a=Random_Set[imax];
		Random_Set[imax]=Random_Set[imin];
		Random_Set[imin]=a;
	}

	return(0);

}




/****************************************************************
*	RoundOff function:
*		Round a floating point number to the nearest integer.
*
*		Return Value:
*			rounded 32-bit integer
*
****************************************************************/

unsigned int  RoundOff(double x) { return((unsigned int )floor(x+0.5)); }




