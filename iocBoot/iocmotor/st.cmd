# Example vxWorks startup file
#Following must be added for many board support packages
#cd <full path to target bin directory>

< cdCommands
 
#< ../nfsCommands

cd appbin
ld < iocCore
ld < seq
#ld < Lib

cd startup
#dbLoadDatabase("../../dbd/.dbd")
#dbLoadRecords("../../db/.db")
#dbLoadTemplate("../../db/.substitutions")

iocInit
#seq &<some snc program>
