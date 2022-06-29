from time import sleep
from playsound import playsound
from glob import glob
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Joy
from enum import IntEnum
from kobuki_ros_interfaces.action import AutoDocking

# The primary controller I use is the switch pro controller,
# it seems to have the same joy layout as the xbox controller
# index | Switch control
class SWITCH(IntEnum):
    B = 0
    A = 1
    Y = 2
    X = 3
    ZL = 4
    ZR = 5
    L = 6
    R = 7
    MINUS = 8
    PLUS = 9
    LS = 10
    RS = 11
    HOME = 12
    SCREENSHOT = 13


class JoyController(Node):
    def __init__(self):
        super().__init__("joy_controller")
        self._joy_subscription = self.create_subscription(
            Joy, "/joy", self.control_recieved, 10
        )
        self._autodocking_action = ActionClient(
            self, AutoDocking, "auto_docking_action"
        )
        print("Waiting for autodocking server..")
        # self._autodocking_action.wait_for_server()
        # self.goal_msg = None
        self.button_state = []
        for b in range(14):
            self.button_state.append(
                {
                    "active": False,
                }
            )
        print("Controller receiver ready.")

    def get_button_down(self, button: int, current_state: bool) -> bool:
        if current_state:
            if not self.button_state[button]["active"]:
                self.button_state[button]["active"] = True
                return True
            else:
                return False
        self.button_state[button]["active"] = False
        return False

        # if self.button_state[button]["active"] == False and current_state == True:
        #     self.button_state[button]["active"] = True
        #     return True
        # self.button_state[button]["active"] = False
        # return False

    def control_recieved(self, msg: Joy):
        # this kinda icky.
        # also this currently would howevermanytimes/sec so maybe the actions need "cooldowns"
        if self.get_button_down(SWITCH.A, msg.buttons[SWITCH.A]):
            print("A Pushed")
        if self.get_button_down(SWITCH.B, msg.buttons[SWITCH.B]):
            print("B Pushed")
        if self.get_button_down(SWITCH.Y, msg.buttons[SWITCH.Y]):
            print("Y Pushed")
        if self.get_button_down(SWITCH.X, msg.buttons[SWITCH.X]):
            print("X Pushed")
        if self.get_button_down(SWITCH.HOME, msg.buttons[SWITCH.HOME]):
            print("Home Pressed")
            # if self.goal_msg != None:
            #     print("GOING HOME")
            #     self.goal_msg = AutoDocking.Goal()
            #     self._autodocking_action.send_goal_async(self.goal_msg)
            # else:
            #     print("Autodock action already set.")


def main(args=None):
    print("Starting joy controller. Listening to events on /joy")
    rclpy.init(args=args)
    joy_controller = JoyController()
    rclpy.spin(joy_controller)

    joy_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
