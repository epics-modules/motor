# This example if for OMS VME8/44 controllers.  Since OMS controllers
# communicate across the VME backplace, this example does not require
# MPF.

#The following must be added for many board support packages
#cd "... IOC st.cmd complete directory path ... "

< cdCommands
 
#< ../nfsCommands

cd appbin
ld < NoMPFLib

cd startup
dbLoadDatabase("../../dbd/NoMPFApp.dbd")
dbLoadRecords("../../db/NoMPF.db")

# OMS VME driver setup parameters: 
#     (1)cards, (2)axes per card, (3)base address(short, 16-byte boundary), 
#     (4)interrupt vector (0=disable or  64 - 255), (5)interrupt level (1 - 6),
#     (6)motor task polling rate (min=1Hz,max=60Hz)
omsSetup(1, 8, 0xFC00, 180, 5, 60)

iocInit
