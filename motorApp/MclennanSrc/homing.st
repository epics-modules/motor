program homing("MOTPV=xxx")

#include "seqPVmacros.h"

char* SNLtaskName;
int debug_flag;

/* Turn on run-time debug messages */
option +d;

/* PV definitions */
PV(int, homing_forward_pv, "{MOTPV}.HOMF", Monitor);
PV(int, homing_reverse_pv, "{MOTPV}.HOMR", Monitor);

ss motor
{
  state ready
  {
    when (homing_reverse_pv==1)
    {
	  /* Trigger reverse move */
      
      DEBUG_PRINT(2, "init -> moving", 0, 0, 0, 0);
    
    } state moving
	
    when (homing_forward_pv==1)
    {
	  /* Trigger forward move */
      
      DEBUG_PRINT(2, "init -> moving", 0, 0, 0, 0);
    
    } state moving
  }

  state moving
  {
    when (homing_forward_pv==0 && homing_reverse_pv==0)
    { 
	
      DEBUG_PRINT(2, "moving -> ready", 0, 0, 0, 0);
	  
    } state ready
  }
}
