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
			outcomes=['missionAbort', 'aborted'],
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

				#  ===================================== IdleThreads =====================================
				# Create a Concurrence container
				idlethreads_concurrence = smach.Concurrence(
					outcomes=['missionStart', 'aborted'],
					default_outcome='missionStart',
					child_termination_cb=lambda so: True)
	
				# Open the container
				with idlethreads_concurrence:

					# ADD Idle to IdleThreads #
					smach.Concurrence.add('Idle',
						# NOT_SUPPORTED
					)

					# ADD IdleEventMonitoring to IdleThreads #
					smach.Concurrence.add('IdleEventMonitoring',
						smach_ros.MonitorState('TO BE DEFINED',
							Bool,
							cond_cb=lambda so: False))
				#  ===================================== IdleThreads END =====================================

				# ADD IdleThreads to SarBehavior #
				smach.StateMachine.add('IdleThreads',
					idlethreads_concurrence,
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
					transitions={'targetFound':'Idle'})
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
			transitions={'missionAbort':'MissionAbort'})

		# ADD MissionAbort to TOP state #
		smach.StateMachine.add('MissionAbort',
			# NOT_SUPPORTED
			transitions={})

	# Execute SMACH plan
	outcome = top_sm.execute()

	
if __name__ == '__main__':
	main()
