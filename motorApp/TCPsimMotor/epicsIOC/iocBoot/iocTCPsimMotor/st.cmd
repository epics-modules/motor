#!../../bin/__EPICS_HOST_ARCH/TCPsimMotor

 # This file is not ment to be executed directly
 # It is preprocessed by doit.sh into e.g. st.cmd.linux-x86_64

< envPaths.__EPICS_HOST_ARCH

cd ${TOP}

## Register all support components
dbLoadDatabase "dbd/TCPsimMotor.dbd"
TCPsimMotor_registerRecordDeviceDriver pdbbase

## Load record instances
#dbLoadRecords("db/xxx.db","user=torstenbogershausenHost")
dbLoadRecords("db/TCPsimMotor.db", "P=IOC:,PORT=MCU1,M1=m1,M2=m2,M3=m3,M4=m4")

## Configure devices
drvAsynIPPortConfigure("MC_CPU1","127.0.0.1:5024",0,0,0)
asynOctetSetOutputEos("MC_CPU1", -1, "\n")
asynOctetSetInputEos("MC_CPU1", -1, "\n")
TCPsimMotorCreateController("MCU1", "MC_CPU1", "32", "200", "1000")
  #define ASYN_TRACE_ERROR     0x0001
  #define ASYN_TRACEIO_DEVICE  0x0002
  #define ASYN_TRACEIO_FILTER  0x0004
  #define ASYN_TRACEIO_DRIVER  0x0008
  #define ASYN_TRACE_FLOW      0x0010
  #define ASYN_TRACE_WARNING   0x0020
  #define ASYN_TRACE_INFO      0x0040
asynSetTraceMask("MC_CPU1", -1, 0x41)
##asynSetTraceMask("MC_CPU1", -1, 0x48)

  #define ASYN_TRACEIO_NODATA 0x0000
  #define ASYN_TRACEIO_ASCII  0x0001
  #define ASYN_TRACEIO_ESCAPE 0x0002
  #define ASYN_TRACEIO_HEX    0x0004
asynSetTraceIOMask("MC_CPU1", -1, 2)
##asynSetTraceIOMask("MC_CPU1", -1, 6)

  #define ASYN_TRACEINFO_TIME 0x0001
  #define ASYN_TRACEINFO_PORT 0x0002
  #define ASYN_TRACEINFO_SOURCE 0x0004
  #define ASYN_TRACEINFO_THREAD 0x0008
asynSetTraceInfoMask("MC_CPU1", -1, 15)

#Parameter 3 TCPsimMotorCreateAxis
#define AMPLIFIER_ON_FLAG_CREATE_AXIS  (1)
#define AMPLIFIER_ON_FLAG_WHEN_HOMING  (1<<1)
#define AMPLIFIER_ON_FLAG_USING_CNEN   (1<<2)

##Encoder steps via Ethercat
TCPsimMotorCreateAxis("MCU1", "1", "2", "encoder=MEASURE")

#TCPsimMotorCreateAxis("MCU1", "1", "6", "")
##TCPsimMotorCreateAxis("MCU1", "2")
##TCPsimMotorCreateAxis("MCU1", "3")
##TCPsimMotorCreateAxis("MCU1", "4")

cd ${TOP}/iocBoot/${IOC}
iocInit

## Start any sequence programs
#seq sncxxx,"user=torstenbogershausenHost"
