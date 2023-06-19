from math import sqrt

def goal_distance(robot, our_goal_x, our_goal_y):

    dist = sqrt((robot.xPos - our_goal_x) ** 2 + (robot.yPos - our_goal_y) ** 2)
    return dist

def estimate_radius(enemy_robots, their_goal_x, their_goal_y):

    distances = []
    for robot in enemy_robots:
        dist = goal_distance(robot, their_goal_x, their_goal_y)
        distances.append(dist)
    
    distances.sort()
    
    radius1 = distances[1]
    radius2 = distances[2]
    
    radius = (radius1 + radius2) / 2

    return radius

def estimate_ball_trajectory(ball, xPos):

    ball_coordinates_x, ball_coordinates_y = ball.get_coordinates()
    time = (xPos - ball_coordinates_x) / ball.vy
    proj_y = ball_coordinates_y + ball.vx*time

    return proj_y