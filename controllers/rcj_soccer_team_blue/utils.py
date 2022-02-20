import math
dataFormat = "idd?dd"

def get_direction(ball_vector: list) -> int:
    """Get direction to navigate robot to face the ball

    Args:
        ball_vector (list of floats): Current vector of the ball with respect
            to the robot.

    Returns:
        int: 0 = forward, -1 = right, 1 = left
    """
    if -0.13 <= ball_vector[1] <= 0.13:
        return 0
    return -1 if ball_vector[1] < 0 else 1
def velocity(v):
    if v > 10: return 10
    if v <-10: return -10
    return v
def getDistance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)