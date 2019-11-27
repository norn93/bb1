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
cell_capacity = [
	[3.27, 0],
	[3.61, 5],
	[3.69, 10],
	[3.71, 15],
	[3.73, 20],
	[3.75, 25],
	[3.77, 30],
	[3.79, 35],
	[3.8, 40],
	[3.82, 45],
	[3.84, 50],
	[3.85, 55],
	[3.87, 60],
	[3.91, 65],
	[3.95, 70],
	[3.98, 75],
	[4.02, 80],
	[4.08, 85],
	[4.11, 90],
	[4.15, 95],
	[4.2, 100]
]

def capacity(voltage, cells = 6):
	cell_voltage = round(voltage/cells, 2)
	if cell_voltage == 0:
		return 0
	print("cell:", cell_voltage)
	for i, data in enumerate(cell_capacity):
		this_voltage = data
		next_voltage = cell_capacity[i+1]
		print(this_voltage, next_voltage)
		if cell_voltage == this_voltage[0]:
			print('here')
			return data[1]
		if cell_voltage < next_voltage[0] and cell_voltage > this_voltage[0]:
			print('ther')
			rise = next_voltage[1] - this_voltage[1]
			run = next_voltage[0] - this_voltage[0]
			gradient = rise / run
			print(gradient)
			return this_voltage[1] + gradient * (cell_voltage - this_voltage[0])
	return 100

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
		msg.capacity = capacity(msg.average_voltage)
		pub.publish(msg)

		r.sleep()

	rospy.spin()

if __name__ == "__main__":
	listener_talker()
