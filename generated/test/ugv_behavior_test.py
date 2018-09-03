#!/usr/bin/env python

import rospy
import smach
import smach_ros
import mavros_msgs

from cpswarm_msgs.msg import *
from ugv_mavros_moveto.msg import *


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


def main():
	rospy.init_node('state_machine_node')
	
	# Create a SMACH state machine
	top_sm = smach.StateMachine(['succeeded', 'preempted', 'aborted'])
	
	# Open the container
	with top_sm:
		# Create a State Machine container
		sarbehavior_sm = smach.StateMachine(
			outcomes=['succeeded', 'preempted', 'aborted'])

		# Open the container
		with sarbehavior_sm:
			
			def out_cb(outcome_map):
				if outcome_map['IdleEventMonitoring'] == 'invalid':
					rospy.loginfo('Returning targetFound Event')
					return 'targetFound'
				
				return 'aborted'

			# Create a Concurrence container
			idlethreads_concurrence = smach.Concurrence(
				outcomes=['targetFound', 'aborted'],
				default_outcome='targetFound',
				child_termination_cb=lambda so: True,
				outcome_cb=out_cb,
				output_keys=['latitude', 'longitude'])
			
			# Open the container
			with idlethreads_concurrence:

				smach.Concurrence.add('Idle',
					Idle())
				
				def monitor_cb(ud, msg):
					rospy.loginfo('Executing monitor_cb')
					ud.latitude = msg.lat
					ud.longitude = msg.lon
					return False
				
				smach.Concurrence.add('IdleEventMonitoring',
					smach_ros.MonitorState('/bridge/events/target_found',
						GlobalTargetPositionEvent,
						cond_cb=monitor_cb,
						output_keys=['latitude', 'longitude']))

			smach.StateMachine.add('IdleThreads',
				idlethreads_concurrence,
				transitions={'targetFound':'MoveToTarget'})
	
			smach.StateMachine.add('MoveToTarget',
				smach_ros.SimpleActionState('cpswarm/cmd/moveto',
					GlobalMoveToAction,
					goal_slots=['latitude', 'longitude']),
				transitions={})

		smach.StateMachine.add('SarBehavior',
			sarbehavior_sm,
			transitions={})

	# Execute SMACH plan
	outcome = top_sm.execute()
	
	rospy.signal_shutdown('state_machine_node shutdown')

	
if __name__ == '__main__':
	main()
