#!/usr/bin/env python

import roslib
roslib.load_manifest('diagnostic_updater')
import rospy
import diagnostic_updater
import diagnostic_msgs
import std_msgs

from vesc_msgs.msg import VescStateStamped

def maximum(dictionary):
    highest = ""
    for item in dictionary:
        if highest == "":
            highest = dictionary[item]
        if dictionary[item] > highest:
            highest = dictionary[item]
    return highest

def minimum(dictionary):
    lowest = ""
    for item in dictionary:
        if lowest == "":
            lowest = dictionary[item]
        if dictionary[item] < lowest:
            lowest = dictionary[item]
    return lowest

def average(dictionary):
    total = 0
    for item in dictionary:
        total += dictionary[item]
    return total/len(dictionary)

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
    try:
        for i, data in enumerate(cell_capacity):
            this_voltage = data
            next_voltage = cell_capacity[i+1]
            if cell_voltage == this_voltage[0]:
                return data[1]
            if cell_voltage < next_voltage[0] and cell_voltage > this_voltage[0]:
                rise = next_voltage[1] - this_voltage[1]
                run = next_voltage[0] - this_voltage[0]
                gradient = rise / run
                return this_voltage[1] + gradient * (cell_voltage - this_voltage[0])
    except IndexError:
        return 0
    return 100

class MotorDriver:
    def __init__(self):
        # Start the subscriber
        # self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback) # Note that this is a class method
        self.scan_sub1 = rospy.Subscriber("front_left_wheel/sensors/core", VescStateStamped, self.callback, ("FL"))
        self.scan_sub2 = rospy.Subscriber("back_left_wheel/sensors/core", VescStateStamped, self.callback, ("BL"))
        self.scan_sub3 = rospy.Subscriber("front_right_wheel/sensors/core", VescStateStamped, self.callback, ("FR"))
        self.scan_sub4 = rospy.Subscriber("back_right_wheel/sensors/core", VescStateStamped, self.callback, ("BR"))

        self.temperature = {
            "FL": 0,
            "BL": 0,
            "FR": 0,
            "BR": 0
        }
        self.voltage = {
            "FL": 0,
            "BL": 0,
            "FR": 0,
            "BR": 0
        }

    def check_lower_bound_battery(self, stat):
        if average(self.voltage) > 3.73 * 6:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Voltage lower bound OK")
        elif minimum(self.voltage) > 3.2 * 6:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Approaching voltage lower bound...")
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Voltage lower bound breached!")
        stat.add("Battery voltage (V)", average(self.voltage))
        stat.add("Battery capacity (%)", capacity(average(self.voltage)))
        return stat

    def check_upper_bound_battery(self, stat):
        if average(self.voltage) < 4.2 * 6:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Voltage upper bound OK")
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Voltage upper bound breached!")
        return stat

    def check_upper_bound_temperature(self, stat):
        if average(self.temperature) < 40:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Temperature upper bound OK")
        elif maximum(self.temperature) < 50:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Approaching temperature upper bound...")
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature upper bound breached!")
        stat.add("Driver temperature (C)", average(self.temperature))
        return stat

    def callback(self, data, wheel):
        self.temperature[wheel] = float(data.state.temperature_pcb)
        self.voltage[wheel] = float(data.state.voltage_input)

if __name__=='__main__':
    rospy.init_node("bb1_motor_driver")

    updater = diagnostic_updater.Updater()

    updater.setHardwareID("none")

    motor_driver = MotorDriver()
    lower_battery = diagnostic_updater.FunctionDiagnosticTask("Undervoltage check",
        motor_driver.check_lower_bound_battery)
    upper_battery = diagnostic_updater.FunctionDiagnosticTask("Overvoltage check",
        motor_driver.check_upper_bound_battery)
    upper_temperature = diagnostic_updater.FunctionDiagnosticTask("Overtemperature check",
        motor_driver.check_upper_bound_temperature)

    bounds = diagnostic_updater.CompositeDiagnosticTask("Bound check")
    bounds.addTask(lower_battery)
    bounds.addTask(upper_battery)

    updater.add(bounds)

    updater.broadcast(0, "Initializing...")

    pub_battery = rospy.Publisher("bb1_motor_driver", std_msgs.msg.Bool, queue_size=10)
    
    freq_bounds = {'min': 9, 'max': 11}
    pub_battery_freq = diagnostic_updater.HeaderlessTopicDiagnostic("motor_driver", updater,
        diagnostic_updater.FrequencyStatusParam(freq_bounds, 0.1, 10))

    pub_battery_freq.addTask(bounds)
    pub_battery_freq.addTask(upper_temperature)

    # If we know that the state of the node just changed, we can force an
    # immediate update.
    updater.force_update()

    # We can remove a task by refering to its name.
    if not updater.removeByName("Bound check"):
        rospy.logerr("The Bound check task was not found when trying to remove it.")

    while not rospy.is_shutdown():
        msg = std_msgs.msg.Bool()
        rospy.sleep(0.1)

        # Calls to pub1 have to be accompanied by calls to pub1_freq to keep
        # the statistics up to date.
        msg.data = False
        pub_battery.publish(msg)
        pub_battery_freq.tick()

        # We can call updater.update whenever is convenient. It will take care
        # of rate-limiting the updates.
        updater.update()