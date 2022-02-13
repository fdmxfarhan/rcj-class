import math
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot1(RCJSoccerRobot):
    def move(self, dest):
        dest_angle = math.atan2(self.robot_pos[0]-dest[0],dest[1]-self.robot_pos[1])*180/math.pi
        angle = self.heading - dest_angle
        if angle > 180: angle -= 360
        if angle <-180: angle += 360
        if angle > 10:
            self.right_motor.setVelocity(10)
            self.left_motor.setVelocity(-10)
        elif angle <-10:
            self.right_motor.setVelocity(-10)
            self.left_motor.setVelocity(10)
        else:
            self.right_motor.setVelocity(10)
            self.left_motor.setVelocity(10)
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                self.heading = self.get_compass_heading()*180/math.pi
                self.robot_pos = self.get_gps_coordinates()
                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                    ball_angle = math.atan2(ball_data['direction'][1], ball_data['direction'][0])*180/math.pi
                    ball_distance = abs(0.0166/math.sin(ball_data['direction'][2]))
                    ball_x =-math.sin((ball_angle + self.heading)*math.pi/180) * ball_distance + self.robot_pos[0]
                    ball_y = math.cos((ball_angle + self.heading)*math.pi/180) * ball_distance + self.robot_pos[1]
                    self.move([ball_x, ball_y])
                    print([ball_x, ball_y], self.robot_pos, self.heading)
                else:
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)