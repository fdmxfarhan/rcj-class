import math
import utils
import struct
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot2(RCJSoccerRobot):
    def readData(self):
        self.heading = self.get_compass_heading()*180/math.pi
        # if(self.name[0] == 'B'):
        #     self.heading = self.heading + 180
        #     if self.heading > 180: self.heading -= 360
        #     if self.heading <-180: self.heading += 360
        self.robot_pos = self.get_gps_coordinates()
        if(self.name[0] == 'B'):
            self.robot_pos[0] *= -1
            self.robot_pos[1] *= -1
        self.sonar = self.get_sonar_values()
        if self.is_new_ball_data():
            self.isBall = True
            self.ball_data = self.get_new_ball_data()
            self.ball_angle = math.atan2(self.ball_data['direction'][1], self.ball_data['direction'][0])*180/math.pi
            self.ball_distance = abs(0.0166/math.sin(self.ball_data['direction'][2]))
            self.ball_x =-math.sin((self.ball_angle + self.heading)*math.pi/180) * self.ball_distance + self.robot_pos[0]
            self.ball_y = math.cos((self.ball_angle + self.heading)*math.pi/180) * self.ball_distance + self.robot_pos[1]
            self.ball_pos = [self.ball_x, self.ball_y]
        else:
            self.isBall = False
        self.robot_x = self.robot_pos[0]
        self.robot_y = self.robot_pos[1]
        self.behind_ball = [self.ball_x, self.ball_y - 0.2]

    def moveToAngle(self, angle):
        if angle > 180: angle -= 360
        if angle <-180: angle += 360
        if -90 < angle < 90:
            if angle > 40:
                self.right_motor.setVelocity(10)
                self.left_motor.setVelocity(-10)
            elif angle <-40:
                self.right_motor.setVelocity(-10)
                self.left_motor.setVelocity(10)
            else:
                self.right_motor.setVelocity(utils.velocity(10 + angle/5))
                self.left_motor.setVelocity(utils.velocity(10 - angle/5))
        else:
            if angle < 0: angle = -180 - angle
            elif angle > 0: angle =  180 - angle
            if angle > 40:
                self.right_motor.setVelocity(-10)
                self.left_motor.setVelocity(10)
            elif angle <-40:
                self.right_motor.setVelocity(10)
                self.left_motor.setVelocity(-10)
            else:
                self.right_motor.setVelocity(utils.velocity(-10 - angle/5))
                self.left_motor.setVelocity(utils.velocity(-10 + angle/5))
    def move(self, dest):
        dest_angle = math.atan2(self.robot_pos[0]-dest[0],dest[1]-self.robot_pos[1])*180/math.pi
        angle = self.heading - dest_angle
        self.moveToAngle(angle)
    def stop(self):
        self.right_motor.setVelocity(0)
        self.left_motor.setVelocity(0)
    def sendTeamData(self):
        packet = struct.pack(utils.dataFormat, self.robot_id, self.robot_x, self.robot_y, self.isBall, self.ball_x, self.ball_y)
        self.team_emitter.send(packet)
    def getTeamData(self):
        self.robot_positions[self.robot_id - 1][0] = self.robot_x
        self.robot_positions[self.robot_id - 1][1] = self.robot_y
        while self.is_new_team_data():
            packet = self.team_receiver.getData()
            self.team_receiver.nextPacket()
            unpacked = struct.unpack(utils.dataFormat, packet)
            self.robot_positions[unpacked[0] - 1][0] = unpacked[1]
            self.robot_positions[unpacked[0] - 1][1] = unpacked[2]
            if not self.isBall and unpacked[3]:
                self.ball_x = unpacked[4]
                self.ball_y = unpacked[5]
                self.ball_pos = [self.ball_x, self.ball_y]
                self.behind_ball = [self.ball_x, self.ball_y - 0.2]
                self.isBall = True
    def run(self):
        self.ball_x = 0
        self.ball_y = 0
        self.isBall = False
        self.T_Goal = [0, -0.7]
        self.O_Goal = [0, 0.7]
        self.ball_pos = [0, 0]
        self.robot_positions = [ [0, 0] , [0, 0] , [0, 0] ]
        self.robot_id = int(self.name[1])
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                self.readData()
                self.sendTeamData()
                self.getTeamData()
                if self.isBall:
                    if utils.getDistance(self.robot_pos, self.behind_ball) > 0.2:
                        self.move(self.behind_ball)
                    else:
                        self.move(self.ball_pos)
                else: 
                    self.move(self.T_Goal)
                