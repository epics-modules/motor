# This example if for OMS VME8/44 controllers.  Since OMS controllers
# communicate across the VME backplace, this example does not require
# MPF.

#The following must be added for many board support packages
#cd "... IOC st.cmd complete directory path ... "

< cdCommands
 
#< ../nfsCommands

cd appbin

# If the VxWorks kernel was built using the project facility, the following must
# be added before any C++ code is loaded (see SPR #28980).
sysCplusEnable=1

ld < NoMPFLib

cd startup
dbLoadDatabase("../../dbd/NoMPFApp.dbd")
dbLoadRecords("../../db/NoMPF.db")
#dbLoadRecords("../../motorExApp/Db/SoftMotorEx.db","user=rls,motor=m1", startup)

# OMS VME driver setup parameters: 
#     (1)cards, (2)axes per card, (3)base address(short, 16-byte boundary), 
#     (4)interrupt vector (0=disable or  64 - 255), (5)interrupt level (1 - 6),
#     (6)motor task polling rate (min=1Hz,max=60Hz)
#omsSetup(1, 8, 0xFC00, 180, 5, 60)

# OMS VME58 driver setup parameters: 
#     (1)cards, (2)axes per card, (3)base address(short, 4k boundary), 
#     (4)interrupt vector (0=disable or  64 - 255), (5)interrupt level (1 - 6),
#     (6)motor task polling rate (min=1Hz,max=60Hz)
oms58Setup(6, 8, 0x1000, 190, 4, 10)

iocInit
