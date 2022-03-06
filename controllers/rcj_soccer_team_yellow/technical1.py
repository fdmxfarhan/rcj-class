import math
import utils
import struct
import geometry
import time
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class technical1(RCJSoccerRobot):
    def readData(self):
        self.heading = self.get_compass_heading()*180/math.pi
        self.robot_pos = self.get_gps_coordinates()
        if self.name[0] == 'B':
            self.robot_pos[0] *= -1
            self.robot_pos[1] *= -1
        self.sonar = self.get_sonar_values()
        if self.is_new_ball_data():
            self.isBall = True
            self.ball_data = self.get_new_ball_data()
            self.ball_angle = math.atan2(self.ball_data['direction'][1], self.ball_data['direction'][0])*180/math.pi
            self.ball_distance = abs(0.0166666/(abs(self.ball_data['direction'][2])/math.sqrt(1 - self.ball_data['direction'][2]**2)))
            self.ball_x =-math.sin((self.ball_angle + self.heading)*math.pi/180) * self.ball_distance + self.robot_pos[0]
            self.ball_y = math.cos((self.ball_angle + self.heading)*math.pi/180) * self.ball_distance + self.robot_pos[1]
            self.ball_pos = [self.ball_x, self.ball_y]
        else:
            self.isBall = False
        self.robot_x = self.robot_pos[0]
        self.robot_y = self.robot_pos[1]
        self.behind_ball = [self.ball_x, self.ball_y - 0.2]
        self.behind_ball2 = [self.ball_x, self.ball_y + 0.2]
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
        distances = [0, 0, 0]
        distances[0] = utils.getDistance([self.robot_positions[0][0], self.robot_positions[0][1]], self.ball_pos)
        distances[1] = utils.getDistance([self.robot_positions[1][0], self.robot_positions[1][1]], self.ball_pos)
        distances[2] = utils.getDistance([self.robot_positions[2][0], self.robot_positions[2][1]], self.ball_pos)
        if distances[self.robot_id - 1] == max(distances):
            self.gaolKeeper = True
        else:
            self.gaolKeeper = False
    def initiate_variables(self):
        self.ball_x = 0
        self.ball_y = 0
        self.isBall = False
        self.T_Goal = [0, -0.65]
        self.O_Goal = [0, 0.65]
        self.ball_pos = [0, 0]
        self.robot_positions = [[0, 0] , [0, 0] , [0, 0]]
        self.robot_id = int(self.name[1])
        self.gaolKeeper = False
        self.goalKeeper_x = 0
        self.last_ball_pos = self.ball_pos
        self.side = False
    def run(self):
        self.initiate_variables()
        flag1 = False
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                self.waitingForKick = self.get_new_data()['waiting_for_kickoff']
                self.readData()
                self.sendTeamData()
                self.getTeamData()
                if self.isBall and ((self.ball_y > 0.7 and not self.side) or (self.ball_y < -0.7 and self.side)):
                    self.side = not self.side
                    print(self.side)
                if self.waitingForKick or not self.isBall:
                    self.stop()
                elif self.side:
                    if self.robot_y < self.ball_y or ((self.robot_x > self.ball_x + 0.2 or self.robot_x < self.ball_x - 0.2) and self.robot_y < self.ball_y + 0.2) or flag1:
                        if((self.robot_x > self.ball_x + 0.2 or self.robot_x < self.ball_x - 0.2) and self.robot_y < self.ball_y + 0.2) or flag1:
                            if self.robot_x > self.ball_x:
                                self.move([self.ball_x, self.ball_y + 0.3])
                            else:
                                self.move([self.ball_x, self.ball_y + 0.3])
                        else:
                            if self.robot_x > self.ball_x:
                                self.move([self.ball_x - 0.2, self.ball_y + 0.3])
                            else:
                                self.move([self.ball_x + 0.2, self.ball_y + 0.3])
                        if(self.robot_y > self.ball_x + 0.3): 
                            flag1 = False
                        else:
                            flag1 = True
                    elif abs(self.robot_y - self.ball_y) > 0.2 and abs(self.robot_x - self.ball_x) > 0.05:
                        self.move(self.behind_ball2)
                    else:
                        self.move(self.ball_pos)
                else:
                    if self.robot_y > self.ball_y or ((self.robot_x > self.ball_x + 0.2 or self.robot_x < self.ball_x - 0.2) and self.robot_y > self.ball_y - 0.2) or flag1:
                        if((self.robot_x > self.ball_x + 0.2 or self.robot_x < self.ball_x - 0.2) and self.robot_y > self.ball_y - 0.2) or flag1:
                            if self.robot_x > self.ball_x:
                                self.move([self.ball_x, self.ball_y + 0.3])
                            else:
                                self.move([self.ball_x, self.ball_y + 0.3])
                        else:
                            if self.robot_x > self.ball_x:
                                self.move([self.ball_x + 0.3, self.ball_y - 0.3])
                            else:
                                self.move([self.ball_x - 0.3, self.ball_y - 0.3])
                        if(self.robot_y < self.ball_x - 0.3): 
                            flag1 = False
                        else:
                            flag1 = True
                    elif abs(self.robot_y - self.ball_y)  > 0.2 and abs(self.robot_x - self.ball_x) > 0.05:
                        self.move(self.behind_ball)
                    else:
                        self.move(self.ball_pos)
                self.last_ball_pos = self.ball_pos