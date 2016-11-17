program homing("MOTPV=xxx,MODE=Constant_velocity")

#include "seqPVmacros.h"

char* SNLtaskName;
char* mode;
int jog_forward_value, jog_reverse_value;
assign jog_forward_value to "{MOTPV}.JOGF";
assign jog_reverse_value to "{MOTPV}.JOGR";
int debug_flag;

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
	  /* Currently we only have one mode so it isn't used. */
	  mode = "{MODE}";
	} state ready
  }

  state ready
  {
    when (home_reverse_pv==1)
    {
    } state reverse_home_requested
	
    when (home_forward_pv==1)
    {
    } state forward_home_requested
  }
  
  state forward_home_requested
  {
	when (home_forward_pv==0) {
	  jog_forward_value = 1;
	  pvPut(jog_forward_value);
	} state moving
  }
  
  state reverse_home_requested
  {
	when (home_reverse_pv==0) {
	  jog_reverse_value = 1;
	  pvPut(jog_reverse_value);	  
	} state moving
  }

  state moving
  {	
    when (movable==1)
    { 
    } state ready
  }
}
