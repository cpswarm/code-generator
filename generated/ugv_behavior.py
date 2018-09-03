#!/usr/bin/env python

import rospy
import smach
import smach_ros
import mavros_msgs


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
			# Create a Concurrence container
			idlethreads_concurrence = smach.Concurrence(
				outcomes=['targetFound', 'aborted'],
				default_outcome='targetFound',
				child_termination_cb=lambda so: True)
	
			# Open the container
			with idlethreads_concurrence:

				smach.Concurrence.add('Idle',
					# NOT_SUPPORTED
				)

				smach.Concurrence.add('IdleEventMonitoring',
					smach_ros.MonitorState('cpswarm/event/target_found', 
						Bool,
						cond_cb=lambda so: False))

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
