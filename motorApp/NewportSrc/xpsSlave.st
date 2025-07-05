program xpsSlave("name=xpsSlave,P=xxx:,R=xps:slave,IP=192.168.1.11,M=GROUP1.POSITIONER,S=GROUP2")

/* Turn on run-time debug messages */
option +d;
/* Make the generated code reentrant */
option +r;
/* Suppress SNC warnings */
/*option -w;*/

#include "seqPVmacros.h"
%%#define epicsExportSharedSymbols
%%#include <shareLib.h>
%%#include "XPS_C8_drivers.h"

#define PORT 5001
#define TIMEOUT 100.0

char* SNLtaskName;
char* ip;
char* master;
char* slave;
%%int xps_socket;
int val;
char* masterRdbk;
double ratioRdbk;

/* PV definitions */

PV(int, debug_flag,   "{P}{R}:debug.VAL", Monitor);
/* the ok PV is an indicator that slave mode has been configured. it should only be false after a power-cycle before initializing */
PV(int, ok_pv,      "{P}{R}:ok.VAL",    NoMon);
PV(int, on_pv,      "{P}{R}:on.VAL",    EvFlag);
PV(int, init_pv,    "{P}{R}:init.VAL",  EvFlag);
PV(double, ratio_pv,   "{P}{R}:ratio.VAL", Monitor);
PV(double, ratioRdbk_pv, "{P}{R}:ratioRdbk.VAL", NoMon);

ss slave_main
{
  state init
  {
    when ()
    {
      /* get macros */
      ip = macValueGet("IP");
      master = macValueGet("M");
      slave = macValueGet("S");
      SNLtaskName = macValueGet("name");
      
      /* Connect to the XPS controller (save the xps_socket ID) */
      %%xps_socket = TCP_ConnectToServer(pVar->ip, PORT, TIMEOUT);
      
      DEBUG_PRINT(2, "init -> update", 0, 0, 0, 0);
      
      efClear(init_pv_mon);
      efClear(on_pv_mon);
    
    } state update
  }

  state update
  {
    when ()
    {
      /* Check if slave mode is on and set the on_pv appropriately */
      %%pVar->val = SingleAxisSlaveParametersGet(xps_socket, pVar->slave, pVar->masterRdbk, &pVar->ratioRdbk);
      DEBUG_PRINT(2, "update(): val = %i, masterRdbk = %s, ratioRdbk = %f", val, masterRdbk, ratioRdbk, 0);
      
      /* Set the ok indictor to 1 if slave parameters exist */
      if (val == 0)
      {
        PVPUT(ok_pv, 1);
      }
      else
      {
        PVPUT(ok_pv, 0);
      }
      
      /* what is the initial value if the first poll fails? */
      /* display the ratio */
      PVPUT(ratioRdbk_pv, ratioRdbk);
    
      DEBUG_PRINT(2, "update -> idle", 0, 0, 0, 0);
    
    } state idle
  
  }
  
  state modeChange
  {
    when ()
    {
      if (on_pv == 1)
      {
        %%pVar->val = SingleAxisSlaveModeEnable(xps_socket, pVar->slave);
        DEBUG_PRINT(2, "on_pv == 1", 0, 0, 0, 0);
      }
      else
      {
        %%pVar->val = SingleAxisSlaveModeDisable(xps_socket, pVar->slave);
        DEBUG_PRINT(2, "on_pv != 1", 0, 0, 0, 0);
      }
      DEBUG_PRINT(2, "modeChange -> idle", 0, 0, 0, 0);
   } state idle
    
  }
  
  /* Should reinit wait until a move is done before changing the gear ratio? */
  state reinit
  {
    when ()
    {
      %%pVar->val = SingleAxisSlaveParametersSet(xps_socket, pVar->slave, pVar->master, pVar->ratio_pv);
      DEBUG_PRINT(2, "reinit(): val = %i", val, 0, 0, 0);

      /* reset the init PV */
      PVPUT(init_pv, 0);
      efClear(init_pv_mon);
      DEBUG_PRINT(2, "reinit -> update", 0, 0, 0, 0);
    
    } state update
  }
  
  state idle
  {
    when ( efTestAndClear(on_pv_mon) && ok_pv == 1 )
    {
      /* A mode toggle has been requested */
      DEBUG_PRINT(2, "idle -> modeChange", 0, 0, 0, 0);
    } state modeChange
    
    when ( efTest(init_pv_mon) )
    {
      /* A reinit has been requested */
      DEBUG_PRINT(2, "idle -> reinit", 0, 0, 0, 0);
    } state reinit
  
  }
}
