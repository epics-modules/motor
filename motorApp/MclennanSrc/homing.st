program homing("MOTPV=xxx,MODE=0")

#include "seqPVmacros.h"

char* SNLtaskName;
int mode;
int jog_forward, jog_reverse;
assign jog_forward to "{MOTPV}.JOGF";
assign jog_reverse to "{MOTPV}.JOGR";
int debug_flag;

/* Turn on run-time debug messages */
option +d;

/* PV definitions */
PV(int, homing_forward_pv, "{MOTPV}.HOMF", Monitor);
PV(int, homing_reverse_pv, "{MOTPV}.HOMR", Monitor);
PV(int, movable, "{MOTPV}.DMOV", Monitor);

ss motor
{
  state init
  {
	when ()
	{
	  /* Currently we only have one mode so it isn't used */
	  mode = "{MODE}";
	} state ready
  }

  state ready
  {
    when (homing_reverse_pv==1)
    {
    } state reverse_home_requested
	
    when (homing_forward_pv==1)
    {
    } state forward_home_requested
  }
  
  state forward_home_requested
  {
	when (homing_forward_pv==0) {
	  jog_forward = 1;
	  pvPut(jog_forward);
	} state moving
  }
  
  state reverse_home_requested
  {
	when (homing_reverse_pv==0) {
	  jog_reverse = 1;
	  pvPut(jog_reverse);	  
	} state moving
  }

  state moving
  {	
    when (movable==1)
    { 
    } state ready
  }
}
