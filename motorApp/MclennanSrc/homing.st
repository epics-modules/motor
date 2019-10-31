program homing("MOTPV=xxx,MODE,AXIS,DEBUG")

#include "seqPVmacros.h"

%% #include "string.h"
%% #include "errlog.h"
%% #include "float.h"

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

double cached_soft_high_limit;
double cached_soft_low_limit;

/* Turn on run-time debug messages */
option +d;

/* Make code reentrant. This is needed to run more than one instance of this program. */
option +r;

/* PV definitions */
PV(int, home_forward_pv, "{MOTPV}.HOMF", Monitor);
PV(int, home_reverse_pv, "{MOTPV}.HOMR", Monitor);
PV(int, movable, "{MOTPV}.DMOV", Monitor);
PV(int, user_stop, "{MOTPV}.STOP", Monitor);

/* Need to use SNL queue synchronization to get all events here. */
syncq user_stop 100;

// Soft limits
PV(double, hlm, "{MOTPV}.HLM", Monitor);
PV(double, llm, "{MOTPV}.LLM", Monitor);

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
        errlogSevPrintf(errlogInfo, "Sequencer: axis %i: FROM ready TO reverse_home_requested\n", axis);
      }
    } state reverse_home_requested
    
    /* Mode 3 always reverse homes */
    when ((home_forward_pv==1) && (mode==3))
    {
      if (debug) {
        errlogSevPrintf(errlogInfo, "Sequencer: axis %i: FROM ready TO reverse_home_requested\n", axis);
      }
    } state reverse_home_requested
    
    when ((home_forward_pv==1) && (mode!=3))
    {
      if (debug) {
        errlogSevPrintf(errlogInfo, "Sequencer: axis %i: FROM ready TO forward_home_requested\n", axis);
      }
    } state forward_home_requested
    
    exit {
      /* Cache soft limits and then remove them */
      cached_soft_high_limit = hlm;
      cached_soft_low_limit = llm;
      
      if (debug) {
          errlogSevPrintf(errlogInfo, "Sequencer: axis %i: Caching limits (high=%f, low=%f)\n", axis, cached_soft_high_limit, cached_soft_low_limit);
      }
      PVPUT(hlm, DBL_MAX);
      PVPUT(llm, -DBL_MAX);
    }
  }
  
  state forward_home_requested
  {
    /* In modes 1 and 3 we need to wait for the home to be cancelled before requesting a jog */
    when (home_forward_pv==0 && (mode==1 || mode==3)) {

      jog_forward_value = 1;
      pvPut(jog_forward_value);
      if (debug) {
        errlogSevPrintf(errlogInfo, "Sequencer: axis %i: FROM forward_home_requested TO processing_move_request (constant velocity move)\n", axis);
      }
    } state processing_move_request
    
    /* No delay needed for modes other than 1 */
    when (!(mode==1 || mode==3)) {
      if (debug) {
        errlogSevPrintf(errlogInfo, "Sequencer: axis %i: FROM forward_home_requested TO processing_move_request\n", axis);
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
        errlogSevPrintf(errlogInfo, "Sequencer: axis %i: FROM reverse_home_requested TO processing_move_request (constant velocity move)\n", axis);
      }
    } state processing_move_request
    
    /* No delay needed for modes other than 1 and 3 */
    when (!(mode==1 || mode==3)) {
      if (debug) {
        errlogSevPrintf(errlogInfo, "Sequencer: axis %i: FROM reverse_home_requested TO processing_move_request\n", axis);
      }
    } state processing_move_request
  }
  
  state processing_move_request
  {
    when (movable==0) {
      if (debug) {
        errlogSevPrintf(errlogInfo, "Sequencer: axis %i: FROM processing_move_request TO moving\n", axis);
      }
    } state moving
    
    /* If move doesn't start within 10 seconds, cancel home (we may be in a state where we can't move) */
    when(delay(10)){
      errlogSevPrintf(errlogMajor, "Sequencer: axis %i: Unable to start homing move. Cancelling home.\n", axis);
      errlogSevPrintf(errlogInfo, "Sequencer: axis %i: FROM processing_move_request TO done\n", axis);
    } state done
  }

  state moving
  { 
    /* Slightly funny syntax - check if variable was updated, if it was, 
       update user_stop with it's latest value and then check stop condition. */
    when (pvGetQ(user_stop) && user_stop == 1)
    {
      errlogSevPrintf(errlogMajor, "Sequencer: axis %i: User stop detected during home - will not define position.\n", axis);
      if (debug) {
        errlogSevPrintf(errlogInfo, "Sequencer: axis %i: FROM moving TO done\n", axis);
      }
    } state done
  
    when (movable==1)
    { 
      if (mode==2 || mode==3) {
        if (debug) {
            errlogSevPrintf(errlogInfo, "Sequencer: axis %i: Setting position to zero (modes 2 and 3)\n", axis);
        }

        pvGet(offset);
        offsetval = offset;

        PVPUT(set, 1);

        PVPUT(position, 0.0);
        PVPUT(position_d, 0.0);

        /* Reapply the offset */
        PVPUT(offset, offsetval);
        
        PVPUT(set, 0);
      }
      if (debug) {
        errlogSevPrintf(errlogInfo, "Sequencer: axis %i: FROM moving TO done\n", axis);
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
      
      if (debug) {
          errlogSevPrintf(errlogInfo, "Sequencer: axis %i: Reapplying cached limits (high=%f, low=%f)\n", axis, cached_soft_high_limit, cached_soft_low_limit);
      }
      /* Re-apply cached soft limits */
      PVPUT(hlm, cached_soft_high_limit);
      PVPUT(llm, cached_soft_low_limit);
      
      if (debug) {
        errlogSevPrintf(errlogInfo, "Sequencer: axis %i: FROM done TO ready\n", axis);
      }
    } state ready
  } 
}
