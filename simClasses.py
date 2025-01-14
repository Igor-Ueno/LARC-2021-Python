from numpy import sqrt, zeros, int32

# Units: cm, rad, s


class KinematicBody:
    """Base class for all moving bodies"""

    def __init__(self):
        self._coordinates = SpatialCoordinates()
        self._velocities = Velocities()

    def set_coordinates(self, x, y, rotation):
        self._coordinates.X = x
        self._coordinates.Y = y
        self._coordinates.rotation = rotation

    def set_velocities(self, linear, angular, x, y):
        self._velocities.linear = linear
        self._velocities.angular = angular
        self._velocities.X = x
        self._velocities.Y = y

    def get_coordinates(self):
        """Returns coordinates"""
        coordinates = SpatialCoordinates(self._coordinates.X, self._coordinates.Y, self._coordinates.rotation)
        return coordinates

    def get_velocities(self):
        """Returns velocities"""
        velocities = Velocities(self._velocities.linear, self._velocities.angular, self._velocities.X,
                                self._velocities.Y)
        return velocities

    def calculate_distance(self, body):
        """calculates the distance between self and another kinematic body"""
        return sqrt((self.get_coordinates().X - body.get_coordinates().X) ** 2 +
                    (self.get_coordinates().Y - body.get_coordinates().Y) ** 2)

    def calculate_distance_from_goal(self, mray):
        """Checks distance from object to one of the goals, choose which goal by using mray"""
        if not mray:  # Goal coordinates for each team
            x_gol = 160
            y_gol = 65
        else:
            x_gol = 10
            y_gol = 65

        return sqrt((x_gol - self.get_coordinates().X) ** 2 + (y_gol - self.get_coordinates().Y) ** 2)

    def show_info(self):
        """Input: None
        Description: Logs location and velocity info on the console.
        Output: Obstacle data."""
        print('coordinates.X: {:.2f} | coordinates.Y: {:.2f} | theta: {:.2f} | velocity: {:.2f}'.format(
            self._coordinates.X, self._coordinates.Y, float(self._coordinates.rotation), self._velocities.linear))


class SpatialCoordinates:
    def __init__(self, x=0, y=0, rotation=0):
        self.X = x
        self.Y = y
        self.rotation = rotation


class Velocities:
    def __init__(self, linear=0, angular=0, x=0, y=0):
        self.linear = linear
        self.angular = angular
        self.X = x
        self.Y = y


class Target(KinematicBody):
    """Input: Current target coordinates.
    Description: Stores coordinates for the robots' current target.
    Output: Current target coordinates."""

    def __init__(self):
        super().__init__()


class Obstacle(KinematicBody):
    """Input: Coordinates and velocity of object.
    Description: Stores coordinates and velocity of an obstacle to a robot.
    Output: Coordinates and velocity of object."""

    def __init__(self, robot):
        super().__init__()
        self.robot = robot

    def set_obst(self, x, y, rotation):
        """Input: Coordinates of obstacle.
        Description: Sets obstacle coordinates with data from vision.
        Output: None"""
        self.set_coordinates(x, y, rotation)

    def update(self):
        """Input: Object lists.
        Description: Detects the nearest object and sets it as the current obstacle to avoid.
        Output: Current obstacle."""
        enemies = self.robot.get_enemies()
        friends = self.robot.get_friends()
        distances = []
        distances.extend(enemies)
        distances.extend(friends)

        distances.sort(key=lambda a: self.robot.calculate_distance(a))
        obstacle = distances[0]
        self.set_obst(obstacle.get_coordinates().X, obstacle.get_coordinates().Y, obstacle.get_coordinates().rotation)

    def update2(self, ball, friends, enemies):
        """Input: ball, robot's friends and enemies list
        Description: Detects the nearest object and sets it as the current obstacle to avoid with some exceptions:
                         1 - The enemy player closest to the goal is not be considered obstacle
                         2 - If ball is too close to the enemy robot, he is not be considered obstacle
        Output:"""
        obstacles = enemies

        ball_distances = [enemy.calculate_distance(ball) for enemy in enemies]  # Distance to ball of all enemies robots
        goal_distances = [enemy.calculate_distance_from_goal(enemy.teamYellow) for enemy in enemies]

        for index in range(len(obstacles)-1, -1, -1):
            if ball_distances[index] < 15:
                obstacles.pop(index)
            elif goal_distances[index] < 20 and ball.calculate_distance_from_goal(self.robot.teamYellow) < 20:
                obstacles.pop(index)

        obstacles.extend(friends)
        obstacles.sort(key=lambda a: self.robot.calculate_distance(a))

        # Setting current obstacle
        self.set_obst(obstacles[0].get_coordinates().X, obstacles[0].get_coordinates().Y,
                      obstacles[0].get_coordinates().rotation)


class Ball(KinematicBody):
    """Input: Ball coordinates.
    Description: Stores data on the game ball.
    Output: Ball data."""

    def __init__(self):
        super().__init__()
        self.pastPose = zeros(4).reshape(2, 2)  # Stores the last 3 positions (x,y) => updated on self.simGetPose()

    def set_simulator_data(self, data_ball):
        """Input: FIRASim ball location data.
        Description: Sets positional and velocity data from simulator.
        Output: None"""
        self._coordinates.X = data_ball.x + data_ball.vx * 100 * 8 / 60
        self._coordinates.Y = data_ball.y + data_ball.vy * 100 * 8 / 60

        # Check if prev is out of field, in this case reflect ball movement to reproduce the collision
        if self._coordinates.X > 160:
            self._coordinates.X = 160 - (self._coordinates.Y - 160)
        elif self._coordinates.X < 10:
            self._coordinates.X = 10 - (self._coordinates.Y - 10)

        if self._coordinates.Y > 130:
            self._coordinates.Y = 130 - (self._coordinates.Y - 130)
        elif self._coordinates.Y < 0:
            self._coordinates.Y = - self._coordinates.Y

        self._velocities.X = data_ball.vx
        self._velocities.Y = data_ball.vy


class Robot(KinematicBody):
    """Input: Robot data.
    Description: Stores data about robots in the game.
    Output: Robot data."""

    def __init__(self, index, actuator, mray):
        super().__init__()
        self.flagTrocaFace = False
        self.isLeader = None
        self.teamYellow = mray
        self.spin = False
        self.contStopped = 0
        self.holdLeader = 0
        self.index = int32(index)
        self.actuator = actuator
        self.face = 1  # ? Defines the current face of the robot
        self.rightMotor = 0  # ? Right motor handle
        self.leftMotor = 0  # ? Left motor handle
        self.vL = 0  # ? Left wheel velocity (cm/s) => updated on simClasses.py -> simSetVel()
        self.vR = 0  # ? Right wheel velocity (cm/s) =>  updated on simClasses.py -> simSetVel()
        if self.index == 0:  # ! Robot max velocity (cm/s)
            self.vMax = 40  # 35
        else:
            self.vMax = 50
        self.rMax = 3 * self.vMax  # ! Robot max rotation velocity (rad*cm/s)
        self.L = 7.5  # ? Base length of the robot (cm)
        self.LSimulador = 6.11 # ? Base length of the robot on copelia (cm)
        self.R = 3.4  # ? Wheel radius (cm)
        self.obst = Obstacle(self)  # ? Defines the robot obstacle
        self.target = Target()  # ? Defines the robot target
        self._enemies = []
        self._friends = []
        self.pastPose = zeros(12).reshape(4,
                                          3)

    def arrive(self):
        """Input: None.
        Description: Returns True if the distance between the target and the robot is less than 3cm - False otherwise
        Output: True or False."""
        if self.calculate_distance(self.target) <= 3:
            return True
        else:
            return False

    def set_simulator_data(self, data_robot):
        """Input: Simulator robot data.
        Description: Sets positional and velocity data from simulator data
        Output: None."""
        self.set_coordinates(data_robot.x, data_robot.y, data_robot.a)
        linear_velocity = sqrt(data_robot.vx ** 2 + data_robot.vy ** 2)
        self.set_velocities(linear_velocity, data_robot.va, data_robot.x, data_robot.y)

    def sim_set_vel(self, v, w):
        """Input: Linear and angular velocity data.
        Description: Sends velocity data to simulator to move the robots.
        Output: None."""
        if self.face == 1:
            self.vR = v + 0.5 * self.L * w
            self.vL = v - 0.5 * self.L * w
        else:
            self.vL = -v - 0.5 * self.L * w
            self.vR = -v + 0.5 * self.L * w
        self.actuator.send(self.index, self.vL, self.vR)

    def sim_set_vel2(self, v1, v2):
        """Input: Wheels velocity data.
        Description: Sends velocity data to simulator to move the robots.
        Output: None."""
        self.actuator.send(self.index, v1, v2)

    def set_friends(self, friends):
        self._friends = friends
        for index, friend in enumerate(self._friends):
            if friend is self:
                friends.pop(index)
                return

    def set_enemies(self, enemies):
        self._enemies = enemies

    def get_friends(self) -> list:
        return self._friends.copy()

    def get_enemies(self) -> list:
        return self._enemies.copy()

    def get_target(self) -> Target:
        return self.target
