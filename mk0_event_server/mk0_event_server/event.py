from asyncio import current_task
from enum import Enum
from multiprocessing import Event
from time import sleep
from playsound import playsound
from glob import glob
import rclpy
from rclpy.node import Node
from kobuki_ros_interfaces.msg import SensorState

# this will monitor various events (bumper, battery, docking status, etc)
# and will generate a sound to let outside user know something is occurring.
# my batteries have died too often without me knowing :C


class KobukiEvents(Enum):
    BUMPER = 0
    CLIFF = 1
    WHEEL_DROP = 2
    BATTERY_WARNING = 3
    BATTERY_CRITICAL = 4


class EventListener(Node):
    def __init__(self):
        super().__init__('mk0_event_listener')
        self.subscription = self.create_subscription(
            SensorState,
            '/sensors/core',
            self.state_recieved,
            10
        )
        self.kobuki_base_max_charge = 160  # empirically obtained from full charge
        self.sounds = glob('/home/cherry/sounds/*')
        self.event_info = {
            KobukiEvents.BUMPER: {
                "timer": 0,
                "reset_time": 10,
                "sound": self.sounds[0]
            },
            KobukiEvents.CLIFF: {
                "timer": 0,
                "reset_time": 10,
                "sound": self.sounds[0]
            },
            KobukiEvents.WHEEL_DROP: {
                "timer": 0,
                "reset_time": 10,
                "sound": self.sounds[0]
            },
            KobukiEvents.BATTERY_WARNING: {
                "timer": 0,
                "reset_time": 10,
                "sound": self.sounds[0]
            },
            KobukiEvents.BATTERY_CRITICAL: {
                "timer": 0,
                "reset_time": 2,
                "sound": self.sounds[0]
            },
        }

    def state_recieved(self, msg: SensorState):
        self.get_logger().info("state message recieved")
        # something sus about this..
        battery_percentage = round(
            float(msg.battery)/float(self.kobuki_base_max_charge)*100)
        self.get_logger().info(f"Kobuki Battery: {battery_percentage} %")
        if (msg.bumper != 0):
            self.trigger_event(KobukiEvents.BUMPER)

        if (msg.cliff != 0):
            self.trigger_event(KobukiEvents.CLIFF)

        if (msg.wheel_drop != 0):
            self.trigger_event(KobukiEvents.WHEEL_DROP)

        if (battery_percentage <= 25.0):
            self.trigger_event(KobukiEvents.BATTERY_WARNING)
        elif (battery_percentage <= 15.0):
            self.trigger_event(KobukiEvents.BATTERY_CRITICAL)

    def trigger_event(self, event, value):
        current_time = self.get_clock().now()
        if (current_time > self.event_info[event]["timer"]):
            playsound(self.event_info[event]["sound"])
            self.event_info[event] = current_time + \
                rclpy.time.Duration(
                    seconds=self.event_info[event]["reset_time"])


def main(args=None):
    rclpy.init(args=args)
    event_listener = EventListener()

    rclpy.spin(event_listener)

    event_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
