program homing("MOTPV=xxx,MODE,AXIS,DEBUG")

#include "seqPVmacros.h"

char* SNLtaskName;
int jog_forward_value, jog_reverse_value;
assign jog_forward_value to "{MOTPV}.JOGF";
assign jog_reverse_value to "{MOTPV}.JOGR";

int set, position, position_d;
double offset, velocity;
assign set to "{MOTPV}.SET";
assign position to "{MOTPV}.VAL";
assign position_d to "{MOTPV}.DVAL";
assign offset to "{MOTPV}.OFF";
assign velocity to "{MOTPV}.VELO";

double offsetval, jogvelocity;

int debug_flag;
int debug;
int mode;
int axis;

/* Turn on run-time debug messages */
option +d;

/* PV definitions */
PV(int, home_forward_pv, "{MOTPV}.HOMF", Monitor);
PV(int, home_reverse_pv, "{MOTPV}.HOMR", Monitor);
PV(int, movable, "{MOTPV}.DMOV", Monitor);

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
      printf("Sequencer: Homing mode for axis %i set to %i\n", axis, mode);
      if (debug) {
        printf("Sequencer: Debug mode ON");
      }
    } state ready
  }

  state ready
  { 
    when (home_reverse_pv==1)
    {
      if (debug) {
        printf("Sequencer: FROM ready TO reverse_home_requested\n");
      }
    } state reverse_home_requested
    
    /* Mode 3 always reverse homes */
    when ((home_forward_pv==1) && (mode==3))
    {
      if (debug) {
        printf("Sequencer: FROM ready TO reverse_home_requested\n");
      }
    } state reverse_home_requested
    
    when ((home_forward_pv==1) && (mode!=3))
    {
      if (debug) {
        printf("Sequencer: FROM ready TO forward_home_requested\n");
      }
    } state forward_home_requested
  }
  
  state forward_home_requested
  {
    /* In modes 1 and 3 we need to wait for the home to be cancelled before requesting a jog */
    when (home_forward_pv==0 && (mode==1 || mode==3)) {
      jog_forward_value = 1;
      pvPut(jog_forward_value);
      if (debug) {
        printf("Sequencer: FROM forward_home_requested TO processing_move_request (constant velocity move)\n");
      }
    } state processing_move_request
    
    /* No delay needed for modes other than 1 */
    when (!(mode==1 || mode==3)) {
      if (debug) {
        printf("Sequencer: FROM forward_home_requested TO processing_move_request\n");
      }
    } state processing_move_request
  }
  
  state reverse_home_requested
  {
    /* In modes 1 and 3 we need to wait for the home to be cancelled before requesting a jog */
    when ((mode==1 || mode==3) && home_reverse_pv==0) {
      jog_reverse_value = 1;
      pvPut(jog_reverse_value);
      if (debug) {
        printf("Sequencer: FROM reverse_home_requested TO processing_move_request (constant velocity move)\n");
      }
    } state processing_move_request
    
    /* No delay needed for modes other than 1 and 3 */
    when (!(mode==1 || mode==3)) {
      if (debug) {
        printf("Sequencer: FROM reverse_home_requested TO processing_move_request\n");
      }
    } state processing_move_request
  }
  
  state processing_move_request
  {
    when (movable==0) {
      if (debug) {
        printf("Sequencer: FROM processing_move_request TO moving\n");
      }
    } state moving
  }

  state moving
  { 
    when (movable==1)
    { 
      if (mode==2 || mode==3) {
        if (debug) {
            printf("Sequencer: Setting position to zero (modes 2 and 3)\n");
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
        printf("Sequencer: FROM moving TO done\n");
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
        printf("Sequencer: FROM done TO ready\n");
      }
    } state ready
  } 
}
