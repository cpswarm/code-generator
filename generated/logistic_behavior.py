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
		# Create a Concurrence container
		logisticthreads_concurrence = smach.Concurrence(
			outcomes=['missionAbort'],
			default_outcome='missionAbort')
	
		# Open the container
		with logisticthreads_concurrence:
			# Create a State Machine container
			logisticbehavior_sm = smach.StateMachine(
				outcomes=['succeeded', 'preempted', 'aborted'])

			# Open the container
			with logisticbehavior_sm:
	
				smach.StateMachine.add('LogisticAlgoritm',
					smach_ros.SimpleActionState('cpswarm/cmd/logistic_algorithm',
						Empty),
					transitions={})

			smach.Concurrence.add('LogisticBehavior', logisticbehavior_sm)

			smach.Concurrence.add('LogisticEventMonitoring',
				smach_ros.MonitorState('cpswarm/event/mission_abort', 
					Bool,
					child_termination_cb=lambda so: False))

		smach.StateMachine.add('LogisticThreads',
			logisticthreads_concurrence,
			transitions={'missionAbort':'MissionAbort'})
	
		smach.StateMachine.add('MissionAbort',
			smach_ros.SimpleActionState('cpswarm/cmd/mission_abort',
				Empty),
			transitions={})

	# Execute SMACH plan
	outcome = top_sm.execute()
	
	rospy.signal_shutdown('state_machine_node shutdown')

	
if __name__ == '__main__':
	main()
