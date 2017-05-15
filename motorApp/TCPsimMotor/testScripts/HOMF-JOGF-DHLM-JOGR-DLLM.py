#!/usr/bin/env python

# EPICS Single Motion application test script
#
# http://cars9.uchicago.edu/software/python/pyepics3/
# http://cars9.uchicago.edu/software/python/pyepics3/devices.html#module-motor
#
# m-epics-singlemotion/src/main/test/singlemotion_test.py
# https://nose.readthedocs.org/en/latest/
# https://nose.readthedocs.org/en/latest/testing.html

import epics
import unittest
import os
import sys
import math
import time
###

polltime = 0.2

def waitForStartAndDone(motor, tc_no, wait_for_done):
    wait_for_start = 2
    while wait_for_start > 0:
        wait_for_start -= polltime
        dmov = int(motor.get('DMOV'))
        movn = int(motor.get('MOVN'))
        print '%s: wait_for_start=%f dmov=%d movn=%d dpos=%f' % (tc_no, wait_for_start, dmov, movn, motor.get_position(readback=True,dial=True))
        if movn and not dmov:
           break
        time.sleep(polltime)

    wait_for_done = math.fabs(wait_for_done) #negative becomes positive
    wait_for_done += 1 # One extra second for rounding
    while wait_for_done > 0:
        dmov = int(motor.get('DMOV'))
        movn = int(motor.get('MOVN'))
        print '%s: wait_for_done=%f dmov=%d movn=%d dpos=%f' % (tc_no, wait_for_done, dmov, movn, motor.get_position(readback=True,dial=True))
        if dmov and not movn:
            return True
        time.sleep(polltime)
        wait_for_done -= polltime
    return False
    
def jogDirection(motor, tc_no, direction, jogging_velocity, acceleration):
    if direction > 0:
        destination = motor.DHLM
        motor.put('JOGF', 1)
    else:
        destination = motor.DLLM
        motor.put('JOGR', 1)
    
    time_to_wait = 30
    if jogging_velocity > 0:
        if motor.DHLM != motor.DLLM:
            distance = math.fabs(motor.get_position(readback=True,dial=True) - destination)
            time_to_wait += distance / jogging_velocity + 2 * acceleration
    done = waitForStartAndDone(motor, tc_no, time_to_wait)

    if direction > 0:
        motor.put('JOGF', 0)
    else:
        motor.put('JOGR', 0)

def calcAlmostEqual(motor, tc_no, expected, actual, maxdelta):
    delta = math.fabs(expected - actual)
    inrange = delta < maxdelta
    print '%s: assertAlmostEqual expected=%f actual=%f delta=%f maxdelta=%f inrange=%d' % (tc_no, expected, actual, delta, maxdelta, inrange)
    return inrange

def moveDialPosition(motor, tc_no, destination, velocity, acceleration):
    time_to_wait = 30
    if velocity > 0:
        distance = math.fabs(motor.get_position(readback=True,dial=True) - destination)
        time_to_wait += distance / velocity + 2 * acceleration
    motor.put('DVAL', destination)
    done = waitForStartAndDone(motor, tc_no, time_to_wait)


class Test(unittest.TestCase):
    MSTA_BIT_HOMED    = 1 << (15 -1)
    MSTA_BIT_MINUS_LS = 1 << (14 -1)
    MSTA_BIT_PLUS_LS  = 1 << (3 -1)

    m1 = epics.Motor(os.getenv("TESTEDMOTORAXIS"))

    middle_dialPosition  = round((m1.DLLM + m1.DHLM) / 2)
    range_postion    = m1.DHLM - m1.DLLM
    homing_velocity  = m1.HVEL
    jogging_velocity = m1.JVEL
    moving_velocity  = m1.VELO
    acceleration     = m1.ACCL

    saved_high_limit = m1.get('DHLM')
    saved_low_limit = m1.get('DLLM')
    
    print "m1.DLLM=%d m1.DHLM=%d middle_dialPosition=%d" % (m1.DLLM, m1.DHLM, middle_dialPosition)

    def setUp(self):
        print 'set up'
        mymiddle_dialPosition  = int((self.m1.DLLM + self.m1.DHLM) / 2)
        print 'self.m1.DLLM=%f self.m1.DHLM=%f self.middle_dialPosition=%f' % (self.m1.DLLM, self.m1.DHLM, self.middle_dialPosition)
        print 'self.m1.DLLM=%f self.m1.DHLM=%f mymiddle_dialPosition=%f' % (self.m1.get('DLLM'), self.m1.get('DHLM'), mymiddle_dialPosition)

    def tearDown(self):
        print 'clean up'
        self.m1.put('STOP', 1)
        if self.saved_high_limit > self.saved_low_limit:
            self.m1.put('DHLM', self.saved_high_limit)
            self.m1.put('DLLM', self.saved_low_limit)


    
    # Home the motor
    def test_TC_01(self):
        tc_no = "TC-01"
        print '%s Home the motor forward' % tc_no
        self.m1.put('HOMF', 1)
        time_to_wait = 30
        if self.range_postion > 0 and self.homing_velocity > 0:
            time_to_wait = 1 + self.range_postion / self.homing_velocity + 2 * self.acceleration
        done = waitForStartAndDone(self.m1, tc_no, time_to_wait)

        msta = int(self.m1.get('MSTA'))
        self.assertEqual(True, done, 'done = True')
        self.assertNotEqual(0, msta & self.MSTA_BIT_HOMED, 'MSTA.homed (Axis has been homed)')


    # High soft limit
    def test_TC_020(self):
        tc_no = "TC-020-high-soft-limit"
        print '%s' % tc_no
        jogDirection(self.m1, tc_no, 1, self.jogging_velocity, self.acceleration)
        lvio = int(self.m1.get('LVIO'))
        msta = int(self.m1.get('MSTA'))

        self.assertEqual(0, msta & self.MSTA_BIT_PLUS_LS, 'Plus hard limit not reached')
        self.assertEqual(1, lvio, 'LVIO == 1')
        self.m1.put('JOGF', 0)

    # Tweak away from high limit
    def test_TC_021(self):
        tc_no = "TC-021-tweak-away-high-soft-limit"
        print '%s' % tc_no
        self.m1.tweak(direction='reverse', wait=True)
        lvio = int(self.m1.get('LVIO'))
        #self.assertEqual(0, lvio, 'LVIO == 0')

    # high limit switch
    def test_TC_022(self):
        tc_no = "TC-022-high-limit-switch"
        print '%s' % tc_no
        old_high_limit = self.m1.get('DHLM')
        old_low_limit = self.m1.get('DLLM')
        #switch off the soft limits. Depending on the postion
        # low or high must be set to 0 first
        if self.m1.get_position(readback=True,dial=True) > 0:
            self.m1.put('DLLM', 0.0)
            self.m1.put('DHLM', 0.0)
        else:
            self.m1.put('DHLM', 0.0)
            self.m1.put('DLLM', 0.0)
        
        jogDirection(self.m1, tc_no, 1, self.jogging_velocity, self.acceleration)
        # Get values, check them later
        lvio = int(self.m1.get('LVIO'))
        msta = int(self.m1.get('MSTA'))
        #Go away from limit switch
        self.m1.move(val=old_high_limit, wait=True)
        self.m1.put('DLLM', old_low_limit)
        self.m1.put('DHLM', old_high_limit)

        self.assertEqual(0, lvio, 'LVIO == 0')
        self.assertNotEqual(0, msta & self.MSTA_BIT_PLUS_LS, 'hard limit switch')

        
    # 10%  dialPosition
    def test_TC_023(self):
        tc_no = "TC-023-10-percent-dialPosition"
        print '%s' % tc_no
        destination =  (self.saved_high_limit + 9 * self.saved_low_limit) / 10
        moveDialPosition(self.m1, tc_no, destination,
                         self.moving_velocity, self.acceleration)

        dialPosition = self.m1.get_position(readback=True,dial=True)
        print '%s postion=%f middle_dialPosition=%f' % (
            tc_no, dialPosition, self.middle_dialPosition)
        assert calcAlmostEqual(self.m1, tc_no, destination, dialPosition, 2)
        
    # Low soft limit
    def test_TC_030(self):
        tc_no = "TC-030-low-soft-limit"
        print '%s' % tc_no
        jogDirection(self.m1, tc_no, 0, self.jogging_velocity, self.acceleration)
        lvio = int(self.m1.get('LVIO'))
        msta = int(self.m1.get('MSTA'))

        self.assertEqual(0, msta & self.MSTA_BIT_MINUS_LS, 'Minus hard limit not reached')
        self.assertEqual(1, lvio, 'LVIO == 1')
        self.m1.put('JOGF', 0)

    # Tweak away from low limit
    def test_TC_031(self):
        tc_no = "TC-031-tweak-away-low-soft-limit"
        print '%s' % tc_no
        self.m1.tweak(direction='forward', wait=True)


    # low limit switch
    def test_TC_032(self):
        tc_no = "TC-032-low-limit-switch"
        print '%s' % tc_no
        old_high_limit = self.m1.get('DHLM')
        old_low_limit = self.m1.get('DLLM')
        #switch off the soft limits. Depending on the postion
        # low or high must be set to 0 first
        if self.m1.get_position(readback=True,dial=True) > 0:
            self.m1.put('DLLM', 0.0)
            self.m1.put('DHLM', 0.0)
        else:
            self.m1.put('DHLM', 0.0)
            self.m1.put('DLLM', 0.0)
        
        jogDirection(self.m1, tc_no, 0, self.jogging_velocity, self.acceleration)
        lvio = int(self.m1.get('LVIO'))
        msta = int(self.m1.get('MSTA'))
        self.m1.move(val=old_low_limit, wait=True)

        self.m1.put('DLLM', old_low_limit)
        self.m1.put('DHLM', old_high_limit)

        self.assertEqual(0, lvio, 'LVIO == 0')
        self.assertNotEqual(0, msta & self.MSTA_BIT_MINUS_LS, 'hard limit switch')


    # middle dialPosition
    def test_TC_040(self):
        tc_no = "TC-040-middle-dialPosition"
        print '%s' % tc_no
        moveDialPosition(self.m1, tc_no, self.middle_dialPosition,
                         self.moving_velocity, self.acceleration)

        dialPosition = self.m1.get_position(readback=True,dial=True)
        print '%s postion=%f middle_dialPosition=%f' % (
            tc_no, dialPosition, self.middle_dialPosition)
        assert calcAlmostEqual(self.m1, tc_no, self.middle_dialPosition, dialPosition, 2)
