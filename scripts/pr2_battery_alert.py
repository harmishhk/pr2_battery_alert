#!/usr/bin/env python

LOW_LEVEL = 25
CRITICAL_LEVEL = 10

import rospy
from pr2_msgs.msg import PowerState
from sound_play.msg import SoundRequest


class Pr2BatteryAlert:

    def main(self):
        self.last_relative_capacity = 100
        self.last_AC_present = 16

        rospy.init_node('pr2_battery_alert')

        rospy.Subscriber("power_state", PowerState, self.callback)

        pub = rospy.Publisher('robotsound', SoundRequest, queue_size=1)

        sound_request = SoundRequest()
        sound_request.command = 1
        rate = rospy.Rate(0.2)  # every 5 second
        while not rospy.is_shutdown():
            if self.last_AC_present == 0:
                if self.last_relative_capacity <= CRITICAL_LEVEL:
                    rospy.logdebug(
                        "Battery critically low, level %d", self.last_relative_capacity)
                    sound_request.sound = 5
                    pub.publish(sound_request)
                elif self.last_relative_capacity <= LOW_LEVEL:
                    rospy.logdebug(
                        "Battery low, level %d", self.last_relative_capacity)
                    sound_request.sound = 3
                    pub.publish(sound_request)
                else:
                    rospy.logdebug(
                        "Battery OK, level %d", self.last_relative_capacity)
            rate.sleep()

    def callback(self, data):
        self.last_relative_capacity = data.relative_capacity
        self.last_AC_present = data.AC_present

if __name__ == '__main__':
    try:
        Pr2BatteryAlert().main()
    except rospy.ROSInterruptException:
        pass
