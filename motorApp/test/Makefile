#*************************************************************************
# Copyright (c) 2006 The University of Chicago, as Operator of Argonne
#     National Laboratory.
# Copyright (c) 2002 The Regents of the University of California, as
#     Operator of Los Alamos National Laboratory.
# EPICS BASE is distributed subject to a Software License Agreement found
# in file LICENSE that is included with this distribution.
#*************************************************************************
TOP=../..

include $(TOP)/configure/CONFIG

PROD_LIBS += Com
PROD_LIBS += asyn
PROD_LIBS += motor


TESTPROD_HOST += asynMotorControllerTest
asynMotorControllerTest_SRCS += asynMotorControllerTest.cpp
TESTS += asynMotorControllerTest

TESTPROD_HOST += asynMotorControllerTest2
asynMotorControllerTest2_SRCS += asynMotorControllerTest2.cpp
TESTS += asynMotorControllerTest2

TESTSCRIPTS_HOST += $(TESTS:%=%.t)

include $(TOP)/configure/RULES
