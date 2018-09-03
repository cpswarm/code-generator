#!/usr/bin/env python

import rospy
import smach
import smach_ros
import mavros_msgs


def main():
	rospy.init_node('state_machine_node')
	
	# Create a TOP level SMACH state machine
	top_sm = smach.StateMachine(['succeeded', 'preempted', 'aborted'])
	
	# Open the container
	with top_sm:

		#  ===================================== SarThreads =====================================
		# Create a Concurrence container
		sarthreads_concurrence = smach.Concurrence(
			outcomes=['missionAbort', 'emergencyEvent', 'aborted'],
			default_outcome='missionAbort',
			child_termination_cb=lambda so: True)
	
		# Open the container
		with sarthreads_concurrence:

			# ===================================== SarBehavior =====================================
			# Create a State Machine container
			sarbehavior_sm = smach.StateMachine(
				outcomes=['succeeded', 'preempted', 'aborted'])

			# Open the container
			with sarbehavior_sm:

				# ADD StartUp to SarBehavior #
				smach.StateMachine.add('StartUp',
					smach_ros.ServiceState('cmd/startup',
						StartUp),
					transitions={'succeeded':'Idle'})

				# ADD Idle to SarBehavior #
				smach.StateMachine.add('Idle',
					# NOT_SUPPORTED
					transitions={'missionStart':'TakeOff'})

				# ADD TakeOff to SarBehavior #
				smach.StateMachine.add('TakeOff',
					smach_ros.ServiceState('cmd/takeoff',
						TakeOff,
						request=TakeOffRequest(10)),
					transitions={'succeeded':'CoverageThreads'})

				#  ===================================== CoverageThreads =====================================
				# Create a Concurrence container
				coveragethreads_concurrence = smach.Concurrence(
					outcomes=['targetFound', 'aborted'],
					default_outcome='targetFound',
					child_termination_cb=lambda so: True)
	
				# Open the container
				with coveragethreads_concurrence:

					# ADD Coverage to CoverageThreads #
					smach.Concurrence.add('Coverage',
						smach_ros.SimpleActionState('uav_coverage',
							EmptyAction))

					# ADD CoverageEventMonitoring to CoverageThreads #
					smach.Concurrence.add('CoverageEventMonitoring',
						smach_ros.MonitorState('TO BE DEFINED', 
							Bool,
							cond_cb=lambda so: False))
				#  ===================================== CoverageThreads END =====================================

				# ADD CoverageThreads to SarBehavior #
				smach.StateMachine.add('CoverageThreads',
					coveragethreads_concurrence,
					transitions={'targetFound':'SelectRover'})

				# ADD SelectRover to SarBehavior #
				smach.StateMachine.add('SelectRover',
					smach_ros.SimpleActionState('cmd/assignTask',
						EmptyAction),
					transitions={})
			#  ===================================== SarBehavior END =====================================

			# ADD SarBehavior to SarThreads #
			smach.Concurrence.add('SarBehavior', sarbehavior_sm)

			# ADD SarEventMonitoring to SarThreads #
			smach.Concurrence.add('SarEventMonitoring',
				smach_ros.MonitorState('TO BE DEFINED', 
					Bool,
					cond_cb=lambda so: False))
		#  ===================================== SarThreads END =====================================

		# ADD SarThreads to TOP state #
		smach.StateMachine.add('SarThreads',
			sarthreads_concurrence,
			transitions={'missionAbort':'MissionAbort', 'emergencyEvent':'EmergencyRoutine'})

		# ADD MissionAbort to TOP state #
		smach.StateMachine.add('MissionAbort',
			# NOT_SUPPORTED
			transitions={})

		# ADD EmergencyRoutine to TOP state #
		smach.StateMachine.add('EmergencyRoutine',
			# NOT_SUPPORTED
			transitions={})

	# Execute SMACH plan
	outcome = top_sm.execute()

	
if __name__ == '__main__':
	main()
