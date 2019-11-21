#!/usr/bin/env python
import rospy
import message_filters
from vesc_msgs.msg import VescStateStamped
from bb1_status.msg import status

empty =  {
	"FL": 0,
	"BL": 0,
	"FR": 0,
	"BR": 0
}
temperature = empty
voltage = empty

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
	rospy.loginfo("Avg temperature: %s", round(average(temperature), 1))
	rospy.loginfo("Max temperature: %s", round(maximum(temperature), 1))
	rospy.loginfo("Avg voltage: %s", round(average(voltage), 1))

def listener():

	rospy.init_node("listener", anonymous = True)

	rospy.Subscriber("front_left_wheel/sensors/core", VescStateStamped, callback, ("FL"))
	rospy.Subscriber("back_left_wheel/sensors/core", VescStateStamped, callback, ("BL"))
	rospy.Subscriber("front_right_wheel/sensors/core", VescStateStamped, callback, ("FR"))
	rospy.Subscriber("back_right_wheel/sensors/core", VescStateStamped, callback, ("BR"))

	rospy.spin()

def talker():
	# this needs to be made into a seperate thing, and then they both get launched together

	pub = rospy.Publisher("bb1_status", status)
	rospy.init_node("talker", anonymous = True)
	r = rospy.Rate(2) #2Hz
	msg = status()
	msg.average_voltage = round(average(voltage), 1)
	msg.maximum_temperature = round(maximum(temperature), 1)
	msg.average_temperature = round(average(temperature), 1)

	while not rospy.is_shutdown():
		rospy.loginfo(msg)
		pub.publish(msg)
		r.sleep()

if __name__ == "__main__":
	listener()

	try:
		talker()
	except rospy.ROSInterruptException: pass