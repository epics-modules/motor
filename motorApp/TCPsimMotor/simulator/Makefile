#
#
uname_S   := $(shell sh -c 'uname -s 2>/dev/null || echo unknown')
uname_M   := $(shell sh -c 'uname -m 2>/dev/null || echo unknown')
uname_R   := $(shell sh -c 'uname -r 2>/dev/null | sed -e "s/[()/]/-/g"|| echo unknown')

# Default for the binaries:
BIN            = $(uname_S)_$(uname_M)_$(uname_R)

CFLAGS         = -g -O0
CC             = Unkown_OS
CFLAGS         += -I.

ifeq ($(uname_S),Darwin)
CC             = gcc
CFLAGS         += -Wall -Werror
CFLAGS         += -O0 -g

endif

ifeq ($(uname_S),Linux)
CC             = gcc
CFLAGS         += -Wall -Werror
endif

ifneq (,$(findstring CYGWIN,$(uname_S)))
  ifneq (,$(findstring x86_64,$(uname_M)))
    CFLAGS        += -DSTART_WINSOCK2
    WINOBJS       += $(BIN)/startWinSock.o
  endif
  CC             = gcc
  CFLAGS         += -Wall -Werror
  #CFLAGS         += -DUSE_WINSOCK2
  EXE            = .exe
  LINKWINSOCK    = -lws2_32
endif

ifneq (,$(findstring MINGW,$(uname_S)))
  ifneq (,$(findstring MINGW32_NT-6.1-WOW,$(uname_S)))
    CC             = gcc
    CFLAGS         += -Wall -Werror
    CFLAGS         += -DUSE_WINSOCK2
    CFLAGS         += -DSTART_WINSOCK2
    #CFLAGS         += -O0 -g3 -Wall -fmessage-length=0
    EXE            = .exe
    WINOBJS       += $(BIN)/startWinSock.o
    LINKWINSOCK    = -lws2_32
  else
    CC             = gcc -mno-cygwin
    CFLAGS         += -Wall -Werror
    CFLAGS         += -DUSE_WINSOCK2
    CFLAGS         += -DSTART_WINSOCK2
    #CFLAGS         += -O0 -g3 -Wall -fmessage-length=0
    EXE            = .exe
    WINOBJS       += $(BIN)/startWinSock.o
    LINKWINSOCK    = -lws2_32
  endif

endif


#First target, done when we run "make" (and CC is unknown)
ifeq ($(CC),Unkown_OS)
unkown: 
	echo "Unsupported OS" $(uname_S)
	exit 1
endif

MOTOROBJS=\
 $(BIN)/cmd_TCPsim.o \
 $(BIN)/hw_motor.o

TELOBJS=\
 $(BIN)/main.o \
 $(BIN)/sock-util.o \
 $(BIN)/cmd.o \
 $(BIN)/cmd_buf.o


#First target, done when we run "make" (and CC is known)
all: checkfortabs mdbin $(BIN)/simMotor$(EXE)

checkfortabs:
	./checkfortabs.sh
	./today.sh today.h
	./version.sh version.h

mdbin:
	mkdir -p $(BIN)

clean:
	rm -f $(BIN)/*.o $(BIN)/*.exe

ALLOBJS=$(MOTOROBJS) $(TELOBJS) $(WINOBJS)

$(BIN)/simMotor$(EXE): $(ALLOBJS)
	$(CC) $(ALLOBJS) $(LINKWINSOCK) -o $@

$(BIN)/main.o: \
 Makefile \
 logerr_info.h \
 sock-util.h \
 main.c
	$(CC) -c $(CFLAGS) main.c -o $@

$(BIN)/sock-util.o: \
 Makefile \
 logerr_info.h \
 sock-util.h \
 sock-util.c
	$(CC) -c $(CFLAGS) sock-util.c -o $@

$(BIN)/cmd.o: \
 Makefile \
 sock-util.h \
 hw_motor.h \
 cmd.c
	$(CC) -c $(CFLAGS) cmd.c -o $@

$(BIN)/cmd_buf.o: \
 Makefile \
 cmd_buf.h \
 cmd_buf.c
	$(CC) -c $(CFLAGS) cmd_buf.c -o $@

$(BIN)/cmd_TCPsim.o: \
 Makefile \
 cmd_TCPsim.c \
 hw_motor.h \
 cmd_TCPsim.h
	$(CC) -c $(CFLAGS) cmd_TCPsim.c -o $@

$(BIN)/hw_motor.o: \
 Makefile \
 hw_motor.h \
 hw_motor.c
	$(CC) -c $(CFLAGS) hw_motor.c -o $@


$(BIN)/startWinSock.o: \
 Makefile \
 startWinSock.h \
 startWinSock.c
	$(CC) -c $(CFLAGS)  startWinSock.c -o $@


