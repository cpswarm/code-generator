#!/usr/bin/env python

import rospy
import smach
import smach_ros
import mavros_msgs

from std_msgs.msg import Bool
from std_srvs.srv import Empty

# define state Idle
class Idle(smach.State):

	def __init__(self):
	    smach.State.__init__(self, outcomes=['succeeded', 'preempted'])

	def execute(self, userdata):
		rospy.loginfo('Executing state Idle')
		while True:
			# Check for preempt
			if self.preempt_requested():
				rospy.loginfo("Idle state has been preempted")
				self.service_preempt()
				return 'preempted'
			rospy.sleep(1.0)
			
		return 'succeeded'

def out_cb(outcome_map):
	rospy.loginfo('Executing out_cb')
	if outcome_map['EventMonitoring'] == 'invalid':
		rospy.loginfo('Returning emergencyEvent')
		return 'emergencyEvent'
	
	return 'aborted'


def monitor_cb(ud, msg):
	rospy.loginfo('Executing monitor_cb')
	return False


def child_term_cb(outcome_map):
  # terminate all running states
  rospy.loginfo('Executing child_term')
  
  if outcome_map['EventMonitoring'] == 'invalid':
	rospy.loginfo('child_term return TRUE')
  	return True
  else:
   return False


def main():
	rospy.init_node('cps_behavior')
	
	# Create a SMACH state machine
	top_sm = smach.StateMachine(['succeeded', 'preempted', 'aborted'])
	
	# Open the container
	with top_sm:
		# Create a Concurrence container
		logisticthreads_concurrence = smach.Concurrence(outcomes=['aborted', 'emergencyEvent'],
												default_outcome='aborted',
												child_termination_cb=child_term_cb,
												outcome_cb=out_cb)

		# Open the container
		with logisticthreads_concurrence:
			# Create a State Machine container
			logisticbehavior_sm = smach.StateMachine(
				outcomes=['succeeded', 'preempted', 'aborted'])

			# Open the container
			with logisticbehavior_sm:
	
				smach.StateMachine.add('Idle',
					Idle(),
					transitions={})
	
			smach.Concurrence.add('LogisticBehavior', logisticbehavior_sm)
			smach.Concurrence.add('EventMonitoring', 
				smach_ros.MonitorState("/pushed", Bool, monitor_cb)
			)
		
		smach.StateMachine.add('LogisticThreads',
			logisticthreads_concurrence,
			transitions={'emergencyEvent':'EmergencyRoutine'})
	
		smach.StateMachine.add('EmergencyRoutine',
			smach_ros.ServiceState('emergency_global_state', 						
			Empty),
			transitions={})
		
	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('smach_server', top_sm, '/SM_ROOT')
	sis.start()

	# Execute SMACH plan
	outcome = top_sm.execute()
	
	
if __name__ == '__main__':
	main()
