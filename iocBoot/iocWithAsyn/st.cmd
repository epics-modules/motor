# The is the MPF example for communication to either a Newport MM4000/5 or an
# IMS483 controller.  The examples must be configured by including or omitting
# comment characters (i.e., #'s) from this file.

# The Newport example can be configured for either serial or GPIB communication
# by omitting/including comments (i.e., remove "# !SERIAL! #" or "# !GPIB!   #"
# for serial or GPIB communication, respectively.  The IMS example is serial
# communication only, in "single mode".

# The MPF option is either single or double CPU board configuration and is
# selected by deleting either the "# !MPF-1-CPU! #" for the "# !MPF-2-CPU! #"
# comments.


# The following must be added for many board support packages
#cd "... IOC st.cmd complete directory path ... "

< cdCommands
 
#< ../nfsCommands

cd appbin

# If the VxWorks kernel was built using the project facility, the following must
# be added before any C++ code is loaded (see SPR #28980).
sysCplusEnable=1

ld < WithMPFLib

# !MPF-1-CPU! #ld < GpibHideosLocal.o
# !MPF-2-CPU! #ld < GpibHideosRemote.o

cd startup
dbLoadDatabase("../../dbd/WithMPFApp.dbd")
dbLoadRecords("../../db/WithMPF.db")

routerInit
MPF_Server_Location = 1
# !MPF-1-CPU! #localMessageRouterStart(MPF_Server_Location)
# !MPF-2-CPU! #tcpMessageRouterClientStart(MPF_Server_Location,9900,"164.54.53.78",1500,40)

# Configure the MPF server code. This MUST be configured too!
# !MPF-1-CPU! #< st_mpfserver.cmd

# Newport MM4000 driver setup parameters: 
#     (1) max. controllers, (2)Unused, (3)polling rate (min=1Hz,max=60Hz) 
MM4000Setup(1, 0, 10)

# Newport MM4000 driver configuration parameters: 
#     (1)controller# being configured,
#     (2)port type: 0-GPIB_PORT or 1-RS232_PORT,
#     (3)GPIB link or MPF server location
#     (4)GPIB address or hideos_task name
# !SERIAL! #MM4000Config(0, 1, MPF_Server_Location, "a-Serial[0]")
# !GPIB!   #GPIB_Link = 10
# !GPIB!   #GPIB_Addr = 1
# !GPIB!   #MM4000Config(0, 0, GPIB_Link, GPIB_Addr)

# IMS IM483 driver setup parameters:
#     (1) maximum number of controllers in system
#     (2) N/A
#     (3) motor task polling rate (min=1Hz,max=60Hz)
#IM483SMSetup(1, 0, 1)

# IMS IM483 configuration parameters:
#     (1) card being configured
#     (2) port type (1-RS232_PORT)
#     (3) MPF server location
#     (4) GPIB address or hideos_task
#IM483SMConfig(0, 1, MPF_Server_Location, "a-Serial[0]")

# !MPF-1-CPU! #Server_Mod_Name = GPIB_Module_Name
# !MPF-2-CPU! #Server_Mod_Name = "GPIB0"
# !GPIB!   #HiDEOSGpibLinkConfig(GPIB_Link, GPIB_Addr, Server_Mod_Name)

iocInit
