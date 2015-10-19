#!/usr/bin/env python

# EPICS Single Motion application test script
#
import epics




parser = argparse.ArgumentParser(description='Tests Single motion module specifications')
parser.add_argument('--deviceName', default='IOC')
parser.add_argument('--motor', default='m1')
parser.add_argument('--deadband', default=30)
parser.add_argument('--prompt', default='no')
parser.add_argument('unittest_args', nargs='*')    
args = parser.parse_args()
sys.argv[1:] = args.unittest_args

###

class Test(unittest.TestCase):
# Default device and motor name
    device = args.deviceName
    motor = args.motor
# wait deadband when the script moves the motor to a new location
    sleep_deadband = int(args.deadband)
    if args.prompt == 'yes':
        prompt = 'y'
    else:
        prompt = 'n'
        
    pv_spmg = CaChannel()
    pv_spmg.searchw(device + ':' + motor + '_.SPMG')
    
    pv_motor_resolution = CaChannel()
    pv_motor_resolution.searchw(device + ':' + motor + '_.MRES')
    
    pv_set_position = CaChannel()
    pv_set_position.searchw(device + ':' + motor + '_.VAL')
    
    pv_position = CaChannel()
    pv_position.searchw(device + ':' + motor + '_.RBV')
    
    pv_raw_position = CaChannel()
    pv_raw_position.searchw(device + ':' + motor + '_.RMP')
        
    pv_low_limit = CaChannel()
    pv_low_limit.searchw(device + ':' + motor + '_.LLM')
    
    pv_high_limit = CaChannel()
    pv_high_limit.searchw(device + ':' + motor + '_.HLM')
  
    pv_velocity = CaChannel()
    pv_velocity.searchw(device + ':' + motor + '_.VELO')
    
    pv_low_velocity_limit = CaChannel()
    pv_low_velocity_limit.searchw(device + ':' + motor + '_.VBAS')
    
    pv_high_velocity_limit = CaChannel()
    pv_high_velocity_limit.searchw(device + ':' + motor + '_.VMAX')
    
    pv_actual_velocity = CaChannel()
    pv_actual_velocity.searchw(device + ':' + motor + '_ACT_VL_MON')
    
    pv_tweak_step = CaChannel()
    pv_tweak_step.searchw(device + ':' + motor + '_.TWV')
    
    pv_tweak_forward = CaChannel()
    pv_tweak_forward.searchw(device + ':' + motor + '_.TWF')
    
    pv_tweak_backward = CaChannel()
    pv_tweak_backward.searchw(device + ':' + motor + '_.TWR')
    
    pv_stop = CaChannel()
    pv_stop.searchw(device + ':' + motor + '_.STOP')
    
    pv_kill = CaChannel()
    pv_kill.searchw(device + ':' + motor + '_KILL_MOTOR_CMD.PROC')
    
    pv_activate = CaChannel()
    pv_activate.searchw(device + ':' + motor + '_ACTIVATE_CMD.PROC')
    
    pv_home = CaChannel()
    pv_home.searchw(device + ':' + motor + '_HOME_CMD')

    pv_homing_velocity = CaChannel()
    pv_homing_velocity.searchw(device + ':' + motor + '_.HVEL')
    
    pv_acceleration_time = CaChannel()
    pv_acceleration_time.searchw(device + ':' + motor + '_.ACCL')
 
    pv_offset = CaChannel()
    pv_offset.searchw(device + ':' + motor + '_.OFF')

    pv_backlash = CaChannel()
    pv_backlash.searchw(device + ':' + motor + '_.BDST')
    
    pv_direction = CaChannel()
    pv_direction.searchw(device + ':' + motor + '_.DIR')
    
    pv_motor_status = CaChannel()
    pv_motor_status.searchw(device + ':' + motor + '_AXIS_STS')
    
    pv_activated_status = CaChannel()
    pv_activated_status.searchw(device + ':' + motor + '_MTACT_STS')
    
    pv_amplifier_enabled = CaChannel()
    pv_amplifier_enabled.searchw(device + ':' + motor + '_AMPEN_STS')
    
    pv_loop_mode = CaChannel()
    pv_loop_mode.searchw(device + ':' + motor + '_LOOPM_STS')
    
    pv_amplifier_fault = CaChannel()
    pv_amplifier_fault.searchw(device + ':' + motor + '_AMFE_STS')
    
    pv_negative_end_limit = CaChannel()
    pv_negative_end_limit.searchw(device + ':' + motor + '_NENDL_STS')
    
    pv_positive_end_limit = CaChannel()
    pv_positive_end_limit.searchw(device + ':' + motor + '_PENDL_STS')
    
    pv_stopped_on_des_limit = CaChannel()
    pv_stopped_on_des_limit.searchw(device + ':' + motor + '_SODPL_STS')
    
    pv_stopped_on_limit = CaChannel()
    pv_stopped_on_limit.searchw(device + ':' + motor + '_SOPL_STS')
    
    pv_fatal_following_err = CaChannel()
    pv_fatal_following_err.searchw(device + ':' + motor + '_FAFOE_STS')
    
    pv_I2T_amplifier_fault = CaChannel()
    pv_I2T_amplifier_fault.searchw(device + ':' + motor + '_AMFAE_STS')
    
    pv_phasing_active = CaChannel()
    pv_phasing_active.searchw(device + ':' + motor + '_PHSRA_STS')
    
    pv_phasing_reference_err = CaChannel()
    pv_phasing_reference_err.searchw(device + ':' + motor + '_PREFE_STS')
    
    pv_homed = CaChannel()
    pv_homed.searchw(device + ':' + motor + '_HOCPL_STS')
    
# We read the motor resolution and check the number of decimal places (without trailing zeroes)
    places = len(str(pv_motor_resolution.getw()).split('.')[1].rstrip('0'))
    
    default_acceleration_time = pv_acceleration_time.getw()
    default_low_limit = pv_low_limit.getw()
    default_high_limit = pv_high_limit.getw()
    default_velocity = pv_velocity.getw()
    default_low_velocity_limit = pv_low_velocity_limit.getw()
    default_high_velocity_limit = pv_high_velocity_limit.getw()
    default_homing_velocity = pv_homing_velocity.getw()
    default_step = pv_tweak_step.getw()
    default_offset = pv_offset.getw()
    default_motor_resolution = pv_motor_resolution.getw()
    default_backlash = pv_backlash.getw()
    default_direction = pv_direction.getw()
    default_spmg = pv_spmg.getw()
# Calculate the middle position    
    middle_position = round(default_high_limit - (default_high_limit - default_low_limit) / 2, places)
# Calculate tweak step
    step = round((default_high_limit - default_low_limit) / 4, places)
    
    start_position = 0
    
    print 'default velocity: ' + str(default_velocity)
    print 'low velocity limit: ' + str(default_low_velocity_limit)
    print 'high velocity limit: ' + str(default_high_velocity_limit)
    print 'default homing velocity: ' + str(default_homing_velocity)
    print 'default acceleration time: ' + str(default_acceleration_time)
    print 'default low limit: ' + str(default_low_limit)
    print 'default high limit: ' + str(default_high_limit)
    print 'default step: ' + str(default_step)
    print 'default offset: ' + str(default_offset)
    print 'default motor resolution: ' + str(default_motor_resolution)
    print 'default backlash: ' + str(default_backlash)
    print 'default direction: ' + str(default_direction)
    print 'default spmg (0 - stop, 1 - pause, 2 - move, 3 - go): ' + str(default_spmg)
    print 'middle position: ' + str(middle_position)
    print 'test step size: ' + str(step)
    
    
    def setUp(self):
        print 'set up'
        start_position = self.pv_position.getw()
        self.assertNotEqual(self.pv_low_limit.getw(), self.pv_high_limit.getw(), 'Software position limits are disabled')
        self.pv_set_position.putw(self.middle_position)
        time.sleep(1)
        self.pv_spmg.putw(3)
        time.sleep(abs(self.pv_position.getw() - self.middle_position) / self.default_velocity + 40 * self.default_acceleration_time)
        self.assertAlmostEqual(self.pv_position.getw(), self.middle_position, self.places, 'The motor is not at the middle position (setUp)')
        time.sleep(1)
    
    def tearDown(self):
        print 'clean up'
        self.pv_kill.putw(1)
        time.sleep(1)
        self.pv_set_position.putw(self.pv_position.getw()) 
        if self.pv_motor_resolution.getw() != self.default_motor_resolution:
            self.pv_motor_resolution.putw(self.default_motor_resolution)
        if self.pv_direction.getw() != self.default_direction:
            self.pv_direction.putw(self.default_direction)
        if self.pv_offset.getw() != self.default_offset:
            self.pv_set_offset.putw(self.default_offset)
        if self.pv_acceleration_time.getw() != self.default_acceleration_time:
            self.pv_set_acceleration_time.putw(self.default_acceleration_time)
        if self.pv_low_limit.getw() != self.default_low_limit:
            self.pv_low_limit.putw(self.default_low_limit)
        if self.pv_high_limit.getw() != self.default_high_limit:
            self.pv_high_limit.putw(self.default_high_limit)
        if self.pv_tweak_step.getw() != self.default_step:
            self.pv_tweak_step.putw(self.default_step)
        if self.pv_velocity.getw() != self.default_velocity:
            self.pv_velocity.putw(self.default_velocity)
        if self.pv_low_velocity_limit.getw() != self.default_low_velocity_limit:
            self.pv_low_velocity_limit.putw(self.default_low_velocity_limit)
        if self.pv_high_velocity_limit.getw() != self.default_high_velocity_limit:
            self.pv_high_velocity_limit.putw(self.default_high_velocity_limit)
        if self.pv_homing_velocity.getw() != self.default_homing_velocity:
            self.pv_set_homing_velocity.putw(self.default_homing_velocity)
        if self.pv_backlash.getw() != self.default_backlash:
            self.pv_backlash.putw(self.default_backlash)   
        self.pv_activate.putw(1)
        self.pv_spmg.putw(self.default_spmg)
        self.pv_set_position.putw(self.start_position)
        self.pv_spmg.putw(2)
        time.sleep(0.1)
        timeout = 20;
        while self.pv_motor_status.getw() != 1 and timeout > 0:
            time.sleep(1)
            timeout = timeout - 1
        self.pv_stop.putw(1)
            
    
# Put the motor to the pause mode and change the desired position
    def test_TC_01(self):
        print 'TC-01 Set Position'
        print 'This test case puts the motor in to Pause move mode and'
        print 'changes its desired position. It verifies that the motor does not move.'
        self.pv_spmg.putw(1)
        time.sleep(1)
        self.pv_set_position.putw(self.default_low_limit)
        time.sleep(1)
        self.assertEqual(self.pv_set_position.getw(), self.default_low_limit, 'The desired position was not set to the low limit')
        self.pv_set_position.putw(self.default_high_limit)
        time.sleep(1)
        self.assertEqual(self.pv_set_position.getw(), self.default_high_limit, 'The desired position was not set to the high limit')
    
# Put the motor to the pause mode, set a new desired position and move it to this position in the go mode
    def test_TC_02(self):
        print 'TC-02 Pause and Go Mode'
        print 'Tests if the motor waits for the move command in the pause mode'
        print 'and if it moves to the desired position immediately'
        print 'after it is set when in the go mode.'
        self.pv_spmg.putw(1)
        print 'Go to negative limit'
        self.pv_set_position.putw(self.default_low_limit)
        time.sleep(1)
        self.assertAlmostEqual(self.pv_position.getw(), self.middle_position, self.places, 'The motor moved without the move command when in the "pause" mode')
        self.pv_spmg.putw(3)
        time.sleep((self.middle_position - self.default_low_limit) / self.default_velocity + self.sleep_deadband * self.default_acceleration_time)
        self.assertAlmostEqual(self.pv_position.getw(), self.default_low_limit, self.places, 'The motor did not move when a new position was set in the "go" mode')
        
        print 'Go back to start position'
        self.pv_set_position.putw(self.middle_position)
        time.sleep(1)
        time.sleep((self.middle_position - self.default_low_limit) / self.default_velocity + self.sleep_deadband * self.default_acceleration_time)
        self.assertAlmostEqual(self.pv_position.getw(), self.middle_position, self.places, 'The motor did not move when a new position was set in the "go" mode')
        
        
# Put the motor in the pause mode, set a new desired position and then move it with the move command 
    def test_TC_03(self):
        print 'TC-03 Move Command'
        print 'Tests if the motor moves to the desired position after move command is issued'
        self.pv_spmg.putw(1)
        self.pv_set_position.putw(self.default_low_limit)
        self.pv_spmg.putw(2)
        time.sleep((self.middle_position - self.default_low_limit) / self.default_velocity + self.sleep_deadband * self.default_acceleration_time)
        self.assertAlmostEqual(self.pv_position.getw(), self.default_low_limit, self.places, 'The motor did not move when the move command was issued')

# Tweak the desired position in the positive and in the negative direction         
    def test_TC_04(self):
        print 'TC-04 Tweak Position'
        print 'Changes the tweak step and tweaks the desired position in negative and positive direction'
        if self.step != 0.0:
            self.pv_tweak_step.putw(self.step / 2)
        else:
            self.pv_tweak_step.putw(0.1)
        self.pv_spmg.putw(3)  # put into Go mode
        time.sleep(1)
        print 'Tweak backwards'
        self.pv_tweak_backward.putw(1)
        time.sleep(self.step / self.default_velocity + self.sleep_deadband * self.default_acceleration_time)
        self.assertAlmostEqual(self.pv_position.getw(), self.middle_position - self.step / 2, self.places,
                               'Tweak backwards did not move the motor {0} {1}'.format(self.pv_position.getw(),
                                                                                       self.middle_position - self.step / 2))
        time.sleep(1)
        print 'Tweak forward'
        self.pv_tweak_forward.putw(1)
        time.sleep(self.step / self.default_velocity + self.sleep_deadband * self.default_acceleration_time)
        self.assertAlmostEqual(self.pv_position.getw(), self.middle_position, self.places, 'Tweak forward did not move the motor')
        
        
# Test the "set desired position to zero" command        
    def test_TC_05(self):
        print 'TC-05 Set to 0 Command'
        self.pv_spmg.putw(1)
        time.sleep(1)
        if self.pv_set_position.getw() == 0:
            if self.default_low_limit != 0:
                self.pv_set_position.putw(self.default_low_limit)
                time.sleep(1)
            else:
                self.pv_set_position.putw(self.default_high_limit)
                time.sleep(1)
        self.pv_set_position.putw(0)
        time.sleep(1)
        zero = 0
        if self.pv_low_limit.getw() > 0:
            zero = self.pv_low_limit.getw()
        if self.pv_high_limit.getw() < 0:
            zero = self.pv_low_limit.getw()
        self.assertEqual(self.pv_set_position.getw(), zero, 'The "to zero" command did not change the desired position to 0')
        
# Test the "Positioned" and "Not positioned" motor status
    def test_TC_06(self):
        print 'TC-06 Moving/Not Moving Status'
        print 'Move to the negative limit'
        self.pv_set_position.putw(self.default_low_limit)
        time.sleep((self.middle_position - self.default_low_limit) / self.default_velocity + self.sleep_deadband * self.default_acceleration_time)
        print 'Move to positive and checking for moving status'
        self.pv_set_position.putw(self.default_high_limit)
        time.sleep(self.sleep_deadband * self.default_acceleration_time)
        self.assertEqual(self.pv_motor_status.getw(), 0, 'The general motor status is not "Not positioned"')
        time.sleep((self.default_high_limit - self.default_low_limit) / self.default_velocity + self.sleep_deadband * self.default_acceleration_time)
        self.assertEqual(self.pv_motor_status.getw(), 1, 'The general motor status is not "Positioned"')
        print('Finished successfully')
        
# Change the software position limits and test them 
# This test is deprecated due different implementation by the motor record
# It is not possible to set a new position outside of high and low limit.
    def test_TC_07(self):
        '''
        print 'TC-07 Position Limits'
        print 'Check that set points out side the limit values cannot be set.'
        self.pv_tweak_step.putw(self.step)
        self.pv_low_limit.putw(self.default_low_limit + self.step)
        time.sleep(0.5)
        self.assertEqual(self.pv_low_limit.getw(), self.pv_low_limit.getw(), 'The new low position limit was not set')
        self.pv_high_limit.putw(self.default_high_limit - self.step)
        time.sleep(0.5)
        self.assertEqual(self.pv_high_limit.getw(), self.pv_high_limit.getw(), 'The new high position limit was not set')
        self.pv_set_position.putw(self.default_low_limit)
        time.sleep((self.middle_position - self.default_low_limit) / self.default_velocity + self.sleep_deadband * self.default_acceleration_time)
        self.assertAlmostEqual(self.pv_position.getw(), self.default_low_limit + self.step, self.places, 'The motor is not at the new low position limit')
        self.pv_set_position.putw(self.default_high_limit)
        time.sleep((self.default_high_limit - self.default_low_limit) / self.default_velocity + self.sleep_deadband * self.default_acceleration_time)
        self.assertAlmostEqual(self.pv_position.getw(), self.default_high_limit - self.step, self.places, 'The motor is not at the new high position limit')
        '''
        
# Change the motor velocity, test the motor movement and different velocities and check the actual velocity read back
    def test_TC_08(self):
        print 'TC-08 Velocity Settings'
        print 'Test low velocity limit by moving to negative limit'
        self.pv_velocity.putw(self.pv_low_velocity_limit.getw() - 100)
        time.sleep(1)
        self.assertEqual(self.pv_velocity.getw(), self.pv_low_velocity_limit.getw(),
                         'The velocity is less than the low velocity limit')
        self.pv_set_position.putw(self.default_low_limit)
        time.sleep((self.middle_position - self.default_low_limit) / self.pv_velocity.getw()
                   + self.sleep_deadband * self.default_acceleration_time)
        self.assertAlmostEqual(self.pv_position.getw(), self.default_low_limit, self.places,
                               'The motor (minimum velocity) is not at the low limit')
        print 'OK'
        print 'Test high velocity limit by moving to positive limit'
        self.pv_velocity.putw(self.pv_high_velocity_limit.getw() + 100)
        time.sleep(1)
        self.assertEqual(self.pv_velocity.getw(), self.pv_high_velocity_limit.getw(),
                         'The velocity is more than the high velocity limit')
        self.pv_set_position.putw(self.default_high_limit)
        time.sleep(self.sleep_deadband * self.default_acceleration_time)
        actual_velocity = self.pv_actual_velocity.getw()
        self.assertNotAlmostEqual(actual_velocity, 0, self.places,
                            'The reported velocity of a moving motor is almost zero ({0})'.format(actual_velocity))
        time.sleep((self.default_high_limit - self.default_low_limit) / self.pv_velocity.getw()
                   + self.sleep_deadband * self.default_acceleration_time)
        self.assertAlmostEqual(self.pv_position.getw(), self.default_high_limit, self.places,
                               'The motor (maximum velocity) is not at the high limit')
        print 'OK'
        
# Change low and high velocity limits        
    def test_TC_09(self):   
        print 'TC-09 Velocity Limits'
        print 'Test if velocity limits can be modified.'
        
        velocity = self.default_low_velocity_limit - 1
        if velocity < 0:
            velocity = 0 # don't test velocities less than 0
        self.pv_velocity.putw(velocity)
        time.sleep(0.5)
        vel_rb = self.pv_velocity.getw()
        self.assertEqual(vel_rb, self.default_low_velocity_limit,
                            'Velocity was set outside its minimum limit ({0})'.format(self.default_low_velocity_limit))
        self.pv_low_velocity_limit.putw(velocity)
        self.assertEqual(self.pv_low_velocity_limit.getw(), velocity,
                         'The low velocity limit was not changed')
        time.sleep(0.5)
        self.pv_velocity.putw(velocity)
        time.sleep(0.5)
        vel_rb = self.pv_velocity.getw()
        self.assertEqual(vel_rb, velocity,
                            'Velocity was not set to current minimum ({0})'.format(velocity))
        self.pv_low_velocity_limit.putw(self.default_low_velocity_limit)
        print 'Low limit tested successfully'
        
        velocity = self.default_high_velocity_limit + 1
        self.pv_velocity.putw(velocity)
        time.sleep(0.5)
        vel_rb = self.pv_velocity.getw()
        self.assertEqual(vel_rb, self.default_high_velocity_limit,
                            'Velocity was set outside its maximum limit ({0})'.format(self.default_high_velocity_limit))
        self.pv_high_velocity_limit.putw(velocity)
        self.assertEqual(self.pv_high_velocity_limit.getw(), velocity,
                         'The high velocity limit was not changed')
        time.sleep(0.5)
        self.pv_velocity.putw(velocity)
        time.sleep(0.5)
        vel_rb = self.pv_velocity.getw()
        self.assertEqual(vel_rb, velocity,
                            'Velocity was not set to current maximum ({0})'.format(velocity))
        self.pv_low_velocity_limit.putw(self.default_low_velocity_limit)
        print 'High limit tested successfully'

 # Test acceleration time, home-search move speed and direction, motor resolution, backlash and both offsets settings.       
    def test_TC_10(self):
        """
        print 'TC-10 Motor Settings'
        print 'Tests if the acceleration time, home-search move speed and direction,' 
        print 'motor resolution, backlash, offset and direction of the axis settings can be changed'
        self.pv_spmg.putw(1)
        self.pv_tweak_step.putw(self.step)
        if self.pv_acceleration_time.getw() == self.pv_set_acceleration_time_min.getw():
            self.pv_set_acceleration_time.putw(self.pv_set_acceleration_time_max.getw())
        else:
            self.pv_set_acceleration_time.putw(self.pv_set_acceleration_time_min.getw())
        time.sleep(1) 
        self.assertEqual(self.pv_acceleration_time.getw(), self.pv_set_acceleration_time.getw(), 'The new acceleration time was not set')
        self.pv_set_acceleration_time.putw(self.default_acceleration_time)
        time.sleep(1)
        self.assertEqual(self.pv_acceleration_time.getw(), self.default_acceleration_time, 'The default acceleration time was not set')
        if self.default_offset == 0:
            self.pv_set_offset.putw(1)
        else:
            self.pv_set_offset.putw(self.pv_offset.getw() * -1)
        time.sleep(1)
        self.assertEqual(self.pv_set_offset.getw(), self.pv_offset.getw(), 'The new offset was not set')
        self.pv_set_offset.putw(self.default_offset)
        time.sleep(1)
        self.assertEqual(self.default_offset, self.pv_offset.getw(), 'The default offset was not set')
        self.pv_second_offset.putw(1)
        time.sleep(1)
        self.assertEqual(self.pv_offset.getw(), self.default_offset + 1, 'The second offset did not change the primary offset')
        self.assertEqual(self.pv_high_limit.getw(), self.default_high_limit + 1, 'The second offset did not change the high limit')
        self.assertEqual(self.pv_low_limit.getw(), self.default_low_limit + 1, 'The second offset did not change the low limit')
        self.pv_second_offset.putw(0)
        time.sleep(1)
        self.assertEqual(self.pv_offset.getw(), self.default_offset, 'The second offset did not remember the original offset')
        self.assertEqual(self.pv_high_limit.getw(), self.default_high_limit, 'The second offset did not remember the original high limit')
        self.assertEqual(self.pv_low_limit.getw(), self.default_low_limit, 'The second offset did not remember the original low limit')
        if self.pv_homing_velocity.getw() == 0:
            self.pv_set_homing_velocity.putw(1);
            time.sleep(1)
            self.assertEqual(self.pv_homing_velocity.getw(), 1, 'New homing velocity and direction was not set')
        else:
            self.pv_set_homing_velocity.putw(self.default_homing_velocity * -1)
            time.sleep(2)
            self.assertEqual(self.pv_homing_velocity.getw(), self.default_homing_velocity * -1, 'New homing velocity and direction was not set')
        self.pv_set_homing_velocity.putw(self.default_homing_velocity)
        time.sleep(2)
        self.assertEqual(self.pv_homing_velocity.getw(), self.default_homing_velocity, 'Default homing velocity and direction was not set')
        if self.pv_backlash.getw() == 0:
            self.pv_backlash.putw(1)
            time.sleep(1)
            self.assertEqual(self.pv_backlash.getw(), 1, 'New backlash value was not set')
        else:
            self.pv_backlash.putw(self.default_backlash * -1)
            time.sleep(1)
            self.assertEqual(self.pv_backlash.getw(), self.default_backlash * -1, 'New backlash value was not set')
        self.pv_backlash.putw(self.default_backlash)
        time.sleep(1)
        self.assertEqual(self.pv_backlash.getw(), self.default_backlash, 'Default backlash was not set')
        raw_position_old = self.pv_raw_position.getw()
        self.pv_spmg.putw(1)
        if self.pv_direction.getw() == 0:
            self.pv_direction.putw(1);
            time.sleep(1)
            self.assertEqual(self.pv_direction.getw(), 1, 'New direction was not set')
            self.pv_tweak_forward.putw(1)
            time.sleep(self.step / self.default_velocity + self.sleep_deadband * self.default_acceleration_time)
            self.assertTrue(raw_position_old > self.pv_raw_position.getw(), 'Forward movement with negative axis direction setting did not move the motor backward (raw position)')
            self.pv_tweak_backward.putw(1)
            time.sleep(self.step / self.default_velocity + self.sleep_deadband * self.default_acceleration_time)
            self.pv_direction.putw(0);
            time.sleep(1)
            self.assertEqual(self.pv_direction.getw(), 0, 'Default direction was not set')
        else:
            self.pv_direction.putw(0);
            time.sleep(1)
            self.assertEqual(self.pv_direction.getw(), 0, 'New direction was not set')
            self.pv_tweak_forward.putw(1)
            time.sleep(self.step / self.default_velocity + self.sleep_deadband * self.default_acceleration_time)
            self.assertTrue(raw_position_old < self.pv_raw_position.getw(), 'Forward movement with positive axis direction setting did not move the motor forward (raw position)')
            self.pv_tweak_backward.putw(1)
            time.sleep(self.step / self.default_velocity + self.sleep_deadband * self.default_acceleration_time)
            self.pv_direction.putw(1);
            time.sleep(1)
            self.assertEqual(self.pv_direction.getw(), 1, 'Default direction was not set')
        self.pv_spmg.putw(0)
        self.pv_motor_resolution.putw(self.default_motor_resolution * 10)
        time.sleep(1)
        self.assertEqual(self.pv_motor_resolution.getw(), self.default_motor_resolution * 10, 'New motor resolution was not set')
        self.assertEqual(self.pv_high_velocity_limit.getw(), self.default_high_velocity_limit * 10, 'New motor resolution did not change the high velocity limit')
        self.pv_motor_resolution.putw(self.default_motor_resolution)
        time.sleep(1)
        """

# Test the hard hardware switches
    def test_TC_11(self):
        '''
        print 'TC-11 Hardware Limit Switches'
        if self.prompt == 'y':
            user_input = raw_input('This test will disable software limits and drive the motor to the hardware limits. Continue (y/n)? ')
        else:
            user_input = 'y'
        if user_input == 'y' or user_input == 'Y':
            self.pv_high_limit.putw(0)
            self.pv_low_limit.putw(0)
            time.sleep(1)
            self.pv_set_position.putw(-10000)
            time.sleep(5)
            i = 5
            while self.pv_motor_status.getw() == 0 and i < ((self.default_high_limit - self.default_low_limit) * self.default_velocity * 3):
                time.sleep(1)
                i = i + 1
            if i == 300:
                self.assertEqual(self.pv_motor_status.getw(), 1, 'Hardware low limit was not found after 5 minutes')
            if self.pv_motor_status.getw() == 1:
                self.assertEqual(self.pv_negative_end_limit.getw(), 1, 'The Low limit status is not "Low limit"')
            time.sleep(1)
            self.pv_set_position.putw(10000)
            self.pv_spmg.putw(3)
            time.sleep(5)
            i = 5
            while self.pv_motor_status.getw() == 0 and i < ((self.default_high_limit - self.default_low_limit) * self.default_velocity * 3):
                time.sleep(1)
                i = i + 1
            if i == 300:
                self.assertEqual(self.pv_motor_status.getw(), 1, 'Hardware high limit was not found after 5 minutes')
            if self.pv_motor_status.getw() == 1:
                self.assertEqual(self.pv_positive_end_limit.getw(), 1, 'The High limit status is not "High limit"')
            self.pv_set_position.putw(self.middle_position)
            time.sleep(5)
            self.pv_stop.putw(1)
            '''
            
        
# Test the motor status bits        
    def test_TC_14(self):
        print 'TC-14 Motor Statuses'
        self.assertEqual(self.pv_amplifier_enabled.getw(), 1, 'The Amplifier enabled status is not "Enabled"')
        self.assertEqual(self.pv_loop_mode.getw(), 0, 'The Loop mode status is not "Closed"')
        self.assertEqual(self.pv_amplifier_fault.getw(), 0, 'The Amplifier fault error status is not "No fault"')
        self.assertEqual(self.pv_negative_end_limit.getw(), 0, 'The Negative end limit set status is not "OK"')
        self.assertEqual(self.pv_positive_end_limit.getw(), 0, 'The Positive end limit set status is not "OK"')
        self.assertEqual(self.pv_stopped_on_des_limit.getw(), 0, 'The Stopped on des. pos. limit status is not "No limit stop"')
        self.assertEqual(self.pv_stopped_on_limit.getw(), 0, 'The Stopped on position limit status is not "Not st. on limit"')
        self.assertEqual(self.pv_fatal_following_err.getw(), 0, 'The Fatal following error status is not "OK"')
        self.assertEqual(self.pv_I2T_amplifier_fault.getw(), 0, 'The I2T amplifier fault error status is not "No fault"')
        self.assertEqual(self.pv_phasing_active.getw(), 0, 'The phasing search/read active status is not "Phase inactive"')
        self.assertEqual(self.pv_phasing_reference_err.getw(), 0, 'The amplifier fault error status is not "No phase error"')

    def test_TC_15(self):
        '''
        print 'TC-15 Homing procedure'
        print 'This test will start the home-search procedure.'
        print 'If your motor setup does not have any home-search procedure implemented,'
        print 'run the test case with option --prompt=yes and once asked for input, type \'n\'.'
        if self.prompt == 'y':
            user_input = raw_input('Continue (y/n)? ')
        else:
            user_input = 'y'
        if user_input == 'y' or user_input == 'Y':
            self.pv_home.putw(1)
            time.sleep(5)
            i = 5
            while self.pv_homed.getw() == 0 and i < ((self.default_high_limit - self.default_low_limit) * self.default_homing_velocity * 3):
                time.sleep(1)
                i = i + 1
            self.assertEqual(self.pv_homed.getw(), 1, 'The motor is not homed')
        '''

# Test the "kill" command
    def test_TC_16(self):
        print 'TC-16 Disable the Motor'
        self.pv_kill.putw(1)
        time.sleep(1)
        self.assertEqual(self.pv_motor_status.getw(), 9, 
                         'The general motor status is not "Disabled" after the kill command')
        
# Test the "reset" command        
    def test_TC_17(self):
        print 'TC-17 Reset the Motor'
        self.pv_kill.putw(1)
        time.sleep(1)
        self.assertEqual(self.pv_motor_status.getw(), 9, 
                         'The general motor status is not "Disabled" before the activate command will be tested')
        self.pv_activate.putw(1)
        time.sleep(1)
        self.assertEqual(self.pv_motor_status.getw(), 1, 
                         'The general motor status is not "Positioned" after the activate command')
        
# Test the "stop" command
    def test_TC_18(self):
        print 'TC-18 Stop the Motor'
        self.pv_tweak_step.putw(self.step)
        time.sleep(1)
        self.pv_set_position.putw(self.default_high_limit)
        time.sleep(self.step / self.default_velocity)
        self.pv_stop.putw(1)
        time.sleep(self.step / self.default_velocity)
        self.assertNotEqual(self.default_high_limit, self.pv_position.getw(), 'The motor did not stop at the stop command')  
   
if __name__ == '__main__':
    unittest.main() 
