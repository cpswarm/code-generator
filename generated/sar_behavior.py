#!/usr/bin/env python

import rospy
import smach
import smach_ros
import mavros_msgs


def main():
	rospy.init_node('state_machine_node')
	
	# Create a SMACH state machine
	top_sm = smach.StateMachine(['succeeded', 'aborted'])
	
	# Open the container
	with top_sm:
		# Create a Concurrence container
		sarthreads_concurrence = smach.Concurrence(outcomes=['succeeded', 'aborted'])

		# Open the container
		with sarthreads_concurrence:
			# Create a State Machine container
			sarbehavior_sm = smach.StateMachine(
				outcomes=['succeeded', 'aborted'])

			# Open the container
			with sarbehavior_sm:
	
				smach.StateMachine.add('StartUp',
					smach_ros.ServiceState('cpswarm/cmd/startup',
						StartUp),
					transitions={'succeeded':'Idle'})
	
				smach.StateMachine.add('Idle',
					# NOT_SUPPORTED
					transitions={'missionStart':'TakeOff'})
	
				smach.StateMachine.add('TakeOff',
					smach_ros.SimpleActionState('cpswarm/cmd/takeoff',
						TakeOffAction,
						goal=TakeOffGoal(10)),
					transitions={'succeeded':'TakeOff'})
	
				smach.StateMachine.add('Coverage',
					# NOT_SUPPORTED
					transitions={})

			smach.Concurrence.add('SarBehavior', sarbehavior_sm)

			smach.Concurrence.add('SarEventMonitoring', 
				# NOT_SUPPORTED
			)

		smach.StateMachine.add('SarThreads',
			sarthreads_concurrence,
			transitions={'missionAbort':'MissionAbort', 'emergencyEvent':'EmergencyRoutine'})
	
		smach.StateMachine.add('MissionAbort',
			# NOT_SUPPORTED
			transitions={})
	
		smach.StateMachine.add('EmergencyRoutine',
			# NOT_SUPPORTED
			transitions={})

	# Execute SMACH plan
	outcome = top_sm.execute()
	
	
if __name__ == '__main__':
	main()
