#!/usr/bin/env python

import rospy
import smach
import smach_ros

from cpswarm_msgs.msg import *
from swarmros.msg import *


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
	
	# Create a TOP level SMACH state machine
	top_sm = smach.StateMachine(['succeeded', 'preempted', 'aborted'])
	
	# Open the container
	with top_sm:

		# ===================================== SarBehavior =====================================
		# Create a State Machine container
		sarbehavior_sm = smach.StateMachine(
			outcomes=['succeeded', 'preempted', 'aborted']
		)

		# Open the container
		with sarbehavior_sm:

			#  ===================================== IdleThreads =====================================
			# Callback for custom outcomes from IdleThreads #
			def out_cb(outcome_map):
				# To be completed
				return 'aborted'

			# Create a Concurrence container
			idlethreads_concurrence = smach.Concurrence(
				outcomes=['missionStart', 'aborted'],
				default_outcome='missionStart',
				child_termination_cb=lambda so: True,
				outcome_cb=out_cb
			)

			# Open the container
			with idlethreads_concurrence:

				# ADD Idle to IdleThreads #
				smach.Concurrence.add('Idle',
					# NOT_SUPPORTED
				)

				# ADD IdleEventMonitoring to IdleThreads #
				smach.Concurrence.add('IdleEventMonitoring',
					smach_ros.MonitorState('bridge/events/mission_start',
						SimpleEvent,
						cond_cb=lambda ud, msg: False)
				)
			#  ===================================== IdleThreads END =====================================

			# ADD IdleThreads to SarBehavior #
			smach.StateMachine.add('IdleThreads',
				idlethreads_concurrence,
				transitions={'missionStart':'TakeOff'}
			)

			# ADD TakeOff to SarBehavior #
			smach.StateMachine.add('TakeOff',
				smach_ros.SimpleActionState('cmd/takeoff',
					TakeOffAction),
				transitions={'succeeded':'IdleThreads'}
			)
		#  ===================================== SarBehavior END =====================================

		# ADD SarBehavior to TOP state #
		smach.StateMachine.add('SarBehavior',
			sarbehavior_sm,
			transitions={}
		)
	
	# Create and start the introspection server (uncomment if needed)
	# sis = smach_ros.IntrospectionServer('smach_server', top_sm, '/SM_TOP')
	# sis.start()
	
	# Execute SMACH plan
	outcome = top_sm.execute()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	# sis.stop()
	
	
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
