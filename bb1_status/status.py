#!/usr/bin/env python
import rospy
import message_filters
from vesc_msgs.msg import VescStateStamped
from bb1_status.msg import status

temperature = {
	"FL": 0,
	"BL": 0,
	"FR": 0,
	"BR": 0
}
voltage = {
	"FL": 0,
	"BL": 0,
	"FR": 0,
	"BR": 0
}

def average(dictionary):
	total = 0
	for item in dictionary:
		total += dictionary[item]
	return total/len(dictionary)

def maximum(dictionary):
	highest = ""
	for item in dictionary:
		if highest == "":
			highest = dictionary[item]
		if dictionary[item] > highest:
			highest = dictionary[item]
	return highest

def callback(data, wheel):
	temperature[wheel] = data.state.temperature_pcb
	voltage[wheel] = data.state.voltage_input

def listener_talker():

	rospy.init_node("status", anonymous = True)

	rospy.Subscriber("front_left_wheel/sensors/core", VescStateStamped, callback, ("FL"))
	rospy.Subscriber("back_left_wheel/sensors/core", VescStateStamped, callback, ("BL"))
	rospy.Subscriber("front_right_wheel/sensors/core", VescStateStamped, callback, ("FR"))
	rospy.Subscriber("back_right_wheel/sensors/core", VescStateStamped, callback, ("BR"))

	r = rospy.Rate(1) #1Hz

	pub = rospy.Publisher("bb1_status", status, queue_size = 10)

	while not rospy.is_shutdown():

		msg = status()
		msg.average_voltage = average(voltage)
		msg.maximum_temperature = maximum(temperature)
		msg.average_temperature = average(temperature)

		pub.publish(msg)

		r.sleep()

	rospy.spin()

if __name__ == "__main__":
	listener_talker()