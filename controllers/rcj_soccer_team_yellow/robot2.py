import math
import utils
import struct
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot2(RCJSoccerRobot):
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                self.left_motor.setVelocity(0)
                self.right_motor.setVelocity(0)
                while self.is_new_team_data():
                    packet = self.team_receiver.getData()
                    self.team_receiver.nextPacket()
                    unpacked = struct.unpack("iii?", packet)
                    print(unpacked)