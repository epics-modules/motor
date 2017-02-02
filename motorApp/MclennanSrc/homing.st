program homing("MOTPV=xxx,MODE")

#include "seqPVmacros.h"

char* SNLtaskName;
int jog_forward_value, jog_reverse_value;
char* macros;
assign jog_forward_value to "{MOTPV}.JOGF";
assign jog_reverse_value to "{MOTPV}.JOGR";

int set, position, position_d;
assign set to "{MOTPV}.SET";
assign position to "{MOTPV}.VAL";
assign position_d to "{MOTPV}.DVAL";

int debug_flag;
int mode;

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
	  /* 0: Use motor home, 1: Constant velocity move, 2: Reverse home and s position to 0 */
	  mode = atoi(macValueGet("MODE"));
	  printf("Homing sequence move mode set to %i\n", mode);
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
	  if ( mode==1 ) {
	    jog_forward_value = 1;
	    pvPut(jog_forward_value);
	  }
	} state moving
  }
  
  state reverse_home_requested
  {
	when (home_reverse_pv==0) {
	  if ( mode==1 ) {
	    jog_reverse_value = 1;
	    pvPut(jog_reverse_value);
	  }
	} state moving
  }

  state moving
  {	
    when (movable==1)
    { 
	  if ( mode==2 ) {
		set = 1;
		position = 0;
		position_d = 0;
		
		pvPut(set);
		pvPut(position);
		pvPut(position_d);
		
		set = 0;
		pvPut(set);
	  }
    } state ready
  }
}
