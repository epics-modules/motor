# This configures the MPF server stuff.  GPIB support is commented out, but can
# be added by simply deleting the "# !GPIB! #" comments.

#< ../nfsCommands

#The following must be added for many board support packages
#cd "... IOC st.cmd complete directory path ... "

< cdCommands
 
cd appbin
ld < ipServerLib

cd startup

#start message router
routerInit
tcpMessageRouterServerStart(1,9900,"164.54.53.78",1500,40)

###############################################################################
# Initialize IP carrier
# ipacAddCarrier(ipac_carrier_t *pcarrier, char *cardParams)
#   pcarrier   - pointer to carrier driver structure
#   cardParams - carrier-specific init parameters

carrier = "ipac"
ipacAddCarrier(&ipmv162, "A:l=3,3 m=0xe0000000,64;B:l=3,3 m=0xe0010000,64;C:l=3,3 m=0xe0020000,64;D:l=3,3 m=0xe0030000,64")
initIpacCarrier(carrier, 0)

###############################################################################
# Initialize GPIB module

# !GPIB! #initGpibGsTi9914("GS-IP488-0",carrier,"IP_d",104)
# !GPIB! #initGpibServer("GPIB0","GS-IP488-0",1024,1000)

###############################################################################
# Initialize Octal UART module
#initOctalUART("moduleName","carrierName","carrierSite",nports,intVec)
initOctalUART("octalUart0",carrier,"IP_a",8,100)

# initOctalUARTPort(char* portName,char* moduleName,int port,int baud,
#                   char* parity,int stop_bits,int bits_char,char* flow_control)
# 'baud' is the baud rate. 1200, 2400, 4800, 9600, 19200, 38400
# 'parity' is "E" for even, "O" for odd, "N" for none.
# 'bits_per_character' = {5,6,7,8}
# 'stop_bits' = {1,2}
# 'flow_control' is "N" for none, "H" for hardware
# Port 0 is Generic Serial Record

initOctalUARTPort("UART[0]","octalUart0",0, 9600,"N",1,8,"N")
initOctalUARTPort("UART[1]","octalUart0",1, 9600,"N",1,8,"N")
initOctalUARTPort("UART[2]","octalUart0",2, 9600,"N",1,8,"N")
initOctalUARTPort("UART[3]","octalUart0",3, 9600,"N",1,8,"N")
initOctalUARTPort("UART[4]","octalUart0",4, 9600,"N",1,8,"N")
initOctalUARTPort("UART[5]","octalUart0",5, 9600,"N",1,8,"N")
initOctalUARTPort("UART[6]","octalUart0",6, 9600,"N",1,8,"N")
initOctalUARTPort("UART[7]","octalUart0",7, 9600,"N",1,8,"N")

initSerialServer("a-Serial[0]","UART[0]",1000,20,"\r",1)
initSerialServer("a-Serial[1]","UART[1]",1000,20,"\r",1)
initSerialServer("a-Serial[2]","UART[2]",1000,20,"\r",1)
initSerialServer("a-Serial[3]","UART[3]",1000,20,"\r",1)
initSerialServer("a-Serial[4]","UART[4]",1000,20,"\r",1)
initSerialServer("a-Serial[5]","UART[5]",1000,20,"\r",1)
initSerialServer("a-Serial[6]","UART[6]",1000,20,"\r",1)
initSerialServer("a-Serial[7]","UART[7]",1000,20,"\r",1)
