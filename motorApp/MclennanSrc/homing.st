program homing("MOTPV=xxx,MODE,AXIS,DEBUG")

#include "seqPVmacros.h"

%% #include "string.h"
%% #include "errlog.h"

char* SNLtaskName;
int jog_forward_value, jog_reverse_value;
assign jog_forward_value to "{MOTPV}.JOGF";
assign jog_reverse_value to "{MOTPV}.JOGR";

int set, position, position_d;
double offset;
assign set to "{MOTPV}.SET";
assign position to "{MOTPV}.VAL";
assign position_d to "{MOTPV}.DVAL";
assign offset to "{MOTPV}.OFF";

double offsetval;

int debug_flag;
int debug;
int mode;
int axis;

double cached_soft_high_limit = 0.0;
double cached_soft_low_limit = 0.0;

/* Turn on run-time debug messages */
option +d;

/* Make code reentrant. This is needed to run more than one instance of this program. */
option +r;

/* PV definitions */
PV(int, home_forward_pv, "{MOTPV}.HOMF", Monitor);
PV(int, home_reverse_pv, "{MOTPV}.HOMR", Monitor);
PV(int, movable, "{MOTPV}.DMOV", Monitor);

// Soft limits
PV(int, hlm, "{MOTPV}.HLM", Monitor);
PV(int, llm, "{MOTPV}.LLM", Monitor);

// Physical limit switches
PV(int, hls, "{MOTPV}.HLS", Monitor);
PV(int, lls, "{MOTPV}.LLS", Monitor);

ss motor
{
  state init
  {
    when ()
    {
      /* 0: Use motor home, 1: Constant velocity move, 2: Reverse home and s position to 0, 3: Constant veloctiy move and 0 */
      mode = atoi(macValueGet("MODE"));
      axis = atoi(macValueGet("AXIS"));
      debug = atoi(macValueGet("DEBUG"));
      errlogSevPrintf(errlogInfo, "Sequencer: Homing mode for axis %i set to %i\n", axis, mode);
      
      if (debug) {
        errlogSevPrintf(errlogInfo, "Sequencer: Debug mode ON\n");
      }
    } state ready
  }

  state ready
  { 
    when (home_reverse_pv==1)
    {
      if (debug) {
        errlogSevPrintf(errlogInfo, "Sequencer: FROM ready TO reverse_home_requested\n");
      }
    } state reverse_home_requested
    
    /* Mode 3 always reverse homes */
    when ((home_forward_pv==1) && (mode==3))
    {
      if (debug) {
        errlogSevPrintf(errlogInfo, "Sequencer: FROM ready TO reverse_home_requested\n");
      }
    } state reverse_home_requested
    
    when ((home_forward_pv==1) && (mode!=3))
    {
      if (debug) {
        errlogSevPrintf(errlogInfo, "Sequencer: FROM ready TO forward_home_requested\n");
      }
    } state forward_home_requested
    
    exit {
      /* Cache soft limits and then remove them */
      cached_soft_high_limit = hlm;
      cached_soft_low_limit = llm;
      
      PVPUT(hlm, 999999999);
      PVPUT(llm, -999999999);
    }
  }
  
  state forward_home_requested
  {
    /* In modes 1 and 3 we need to wait for the home to be cancelled before requesting a jog */
    when (home_forward_pv==0 && (mode==1 || mode==3)) {

      jog_forward_value = 1;
      pvPut(jog_forward_value);
      if (debug) {
        errlogSevPrintf(errlogInfo, "Sequencer: FROM forward_home_requested TO processing_move_request (constant velocity move)\n");
      }
    } state processing_move_request
    
    /* No delay needed for modes other than 1 */
    when (!(mode==1 || mode==3)) {
      if (debug) {
        errlogSevPrintf(errlogInfo, "Sequencer: FROM forward_home_requested TO processing_move_request\n");
      }
    } state processing_move_request
  }
  
  state reverse_home_requested
  {
    /* In modes 1 and 3 we need to wait for the home to be cancelled before requesting a jog */
    when (home_reverse_pv==0 && (mode==1 || mode==3 && home_forward_pv==0)) {
    
      jog_reverse_value = 1;
      pvPut(jog_reverse_value);
      if (debug) {
        errlogSevPrintf(errlogInfo, "Sequencer: FROM reverse_home_requested TO processing_move_request (constant velocity move)\n");
      }
    } state processing_move_request
    
    /* No delay needed for modes other than 1 and 3 */
    when (!(mode==1 || mode==3)) {
      if (debug) {
        errlogSevPrintf(errlogInfo, "Sequencer: FROM reverse_home_requested TO processing_move_request\n");
      }
    } state processing_move_request
  }
  
  state processing_move_request
  {
    when (movable==0) {
      if (debug) {
        errlogSevPrintf(errlogInfo, "Sequencer: FROM processing_move_request TO moving\n");
      }
    } state moving
  }

  state moving
  { 
    when (movable==1)
    { 
      if (mode==2 || mode==3) {
        if (debug) {
            errlogSevPrintf(errlogInfo, "Sequencer: Setting position to zero (modes 2 and 3)\n");
        }

        if (mode==3 && (!hls) && (!lls)) {
            errlogSevPrintf(errlogMajor, "Sequencer: Home mode 3 move complete but hardware limit not engaged. Not defining motor position.\n");
        } else {
            pvGet(offset);
            offsetval = offset;

            PVPUT(set, 1);

            PVPUT(position, 0.0);
            PVPUT(position_d, 0.0);

            /* Reapply the offset */
            PVPUT(offset, offsetval);
            
            PVPUT(set, 0);
        }

      }
      if (debug) {
        errlogSevPrintf(errlogInfo, "Sequencer: FROM moving TO done\n");
      }
    } state done
  }
  
  state done {
    when () {
      if (mode==1 || mode==3) {
        jog_reverse_value = 0;
        pvPut(jog_reverse_value);
        jog_forward_value = 0;
        pvPut(jog_forward_value);
      }
      
      /* Re-apply cached soft limits */
      PVPUT(hlm, cached_soft_high_limit);
      PVPUT(llm, cached_soft_low_limit);
      
      if (debug) {
        errlogSevPrintf(errlogInfo, "Sequencer: FROM done TO ready\n");
      }
    } state ready
  } 
}
