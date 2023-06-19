import action, math, math_aux, identify_radius
from numpy import *

class Strategy:
    def __init__(self, robots, enemy_robots, ball, mray):
        self.robots = robots
        self.enemy_robots = enemy_robots
        self.ball = ball
        self.mray = mray
        self.penaltyDefensive = False
        self.penaltyOffensive = False
        if mray:  # Goal coordinates for each team
            self.our_goal_x = 235
            self.our_goal_y = 90
            self.their_goal_x = 15
            self.their_goal_y = 90
        else:
            self.our_goal_x = 15
            self.our_goal_y = 90
            self.their_goal_x = 235
            self.their_goal_y = 90
        self.radius = None
        self.marking_count = 0
        self.upper_marking = None
        self.is_marking = False

    def coach(self):
        """"Picks a strategy depending on the status of the field"""
        # For the time being, the only statuses considered are which side of the field the ball is in
        if self.penaltyDefensive == True:
            self.penaltyModeDefensive()
        elif self.penaltyOffensive == True:
            self.penaltyModeOffensiveSpin()
        else:
            if self.mray:
                if self.ball.xPos > 135:
                    self.basicStgDef()
                else:
                    self.att_stg_5v5()
            else:
                if self.ball.xPos > 115:
                    self.att_stg_5v5()
                else:
                    self.basicStgDef()

    def basicStgDef(self):
        """Basic original strategy"""
        action.screenOutBall(self.robots[3], self.ball, 150, leftSide=not self.mray, upperLim=85, lowerLim=5)
        action.screenOutBall(self.robots[4], self.ball, 150, leftSide=not self.mray, upperLim=175, lowerLim=95)
        if not self.mray:
            if self.ball.xPos < 40 and self.ball.yPos > 50 and self.ball.yPos < 130:
                action.defenderPenalty(self.robots[0], self.ball, leftSide=not self.mray)
                action.screenOutBall(self.robots[1], self.ball, 55, leftSide=not self.mray, upperLim=85, lowerLim=5)
                action.screenOutBall(self.robots[2], self.ball, 55, leftSide=not self.mray, upperLim=175, lowerLim=95)
            else:
                #listRobots = [self.robots[0], self.robots[3], self.robots[4], self.enemy_robots[0], self.enemy_robots[1], self.enemy_robots[2], self.enemy_robots[3], self.enemy_robots[4]]
                friends = [self.robots[0], self.robots[3], self.robots[4]]
                enemys = [self.enemy_robots[0], self.enemy_robots[1], self.enemy_robots[2], self.enemy_robots[3], self.enemy_robots[4]]
                action.followLeader(self.robots[0], self.robots[1], self.robots[2], self.ball,
                                    self.enemy_robots[0], self.enemy_robots[1], self.enemy_robots[2],self.enemy_robots[3],self.enemy_robots[4])
                action.screenOutBall(self.robots[0], self.ball, 20, leftSide=not self.mray, upperLim=110, lowerLim=70)
        else:
            if self.ball.xPos > 195 and self.ball.yPos > 50 and self.ball.yPos < 130:
                action.defenderPenalty(self.robots[0], self.ball, leftSide=not self.mray)
                action.screenOutBall(self.robots[1], self.ball, 55, leftSide=not self.mray, upperLim=85, lowerLim=5)
                action.screenOutBall(self.robots[2], self.ball, 55, leftSide=not self.mray, upperLim=175, lowerLim=95)
            else:
                #listRobots = [self.robots[0], self.robots[3], self.robots[4], self.enemy_robots[0], self.enemy_robots[1], self.enemy_robots[2], self.enemy_robots[3], self.enemy_robots[4]]
                friends = [self.robots[0], self.robots[3], self.robots[4]]
                enemys = [self.enemy_robots[0], self.enemy_robots[1], self.enemy_robots[2], self.enemy_robots[3], self.enemy_robots[4]]
                action.followLeader(self.robots[0], self.robots[1], self.robots[2], self.ball,
                                    self.enemy_robots[0], self.enemy_robots[1], self.enemy_robots[2], self.enemy_robots[3], self.enemy_robots[4])
                action.screenOutBall(self.robots[0], self.ball, 20, leftSide=not self.mray, upperLim=110, lowerLim=70)
        if ((abs(self.robots[0].theta) < deg2rad(10)) or (abs(self.robots[0].theta) > deg2rad(170))) and (self.robots[0].xPos < 25 or self.robots[0].xPos > 225):
            self.robots[0].contStopped += 1
        else:
            self.robots[0].contStopped = 0
    
    def basicStgDef_alt(self):
        """Basic original strategy"""
        action.screenOutBall(self.robots[3], self.ball, 150, leftSide=not self.mray, upperLim=85, lowerLim=5)
        action.screenOutBall(self.robots[4], self.ball, 150, leftSide=not self.mray, upperLim=175, lowerLim=95)
        ball_trajectory = identify_radius.estimate_ball_trajectory(self.ball, self.our_goal_x)
        if not self.mray:
            if self.ball.xPos < 40 and self.ball.yPos > 50 and self.ball.yPos < 130:
                action.defenderPenalty(self.robots[0], self.ball, leftSide=not self.mray)
                action.screenOutBall(self.robots[1], self.ball, 55, leftSide=not self.mray, upperLim=85, lowerLim=5)
                action.screenOutBall(self.robots[2], self.ball, 55, leftSide=not self.mray, upperLim=175, lowerLim=95)
            else:
                #listRobots = [self.robots[0], self.robots[3], self.robots[4], self.enemy_robots[0], self.enemy_robots[1], self.enemy_robots[2], self.enemy_robots[3], self.enemy_robots[4]]
                friends = [self.robots[0], self.robots[3], self.robots[4]]
                enemys = [self.enemy_robots[0], self.enemy_robots[1], self.enemy_robots[2], self.enemy_robots[3], self.enemy_robots[4]]
                action.followLeader(self.robots[0], self.robots[1], self.robots[2], self.ball,
                                    self.enemy_robots[0], self.enemy_robots[1], self.enemy_robots[2],self.enemy_robots[3],self.enemy_robots[4])
                if self.our_goal_y - 20 < ball_trajectory < self.our_goal_x + 20:
                    action.screenOutBall_alt(self.robots[0], self.ball, 20, ball_trajectory, leftSide=not self.mray, upperLim=110, lowerLim=70)
                else:
                    action.screenOutBall(self.robots[0], self.ball, 20, leftSide=not self.mray, upperLim=110, lowerLim=70)
        else:
            if self.ball.xPos > 195 and self.ball.yPos > 50 and self.ball.yPos < 130:
                action.defenderPenalty(self.robots[0], self.ball, leftSide=not self.mray)
                action.screenOutBall(self.robots[1], self.ball, 55, leftSide=not self.mray, upperLim=85, lowerLim=5)
                action.screenOutBall(self.robots[2], self.ball, 55, leftSide=not self.mray, upperLim=175, lowerLim=95)
            else:
                #listRobots = [self.robots[0], self.robots[3], self.robots[4], self.enemy_robots[0], self.enemy_robots[1], self.enemy_robots[2], self.enemy_robots[3], self.enemy_robots[4]]
                friends = [self.robots[0], self.robots[3], self.robots[4]]
                enemys = [self.enemy_robots[0], self.enemy_robots[1], self.enemy_robots[2], self.enemy_robots[3], self.enemy_robots[4]]
                action.followLeader(self.robots[0], self.robots[1], self.robots[2], self.ball,
                                    self.enemy_robots[0], self.enemy_robots[1], self.enemy_robots[2], self.enemy_robots[3], self.enemy_robots[4])
                if self.our_goal_y - 20 < ball_trajectory < self.our_goal_x + 20:
                    action.screenOutBall_alt(self.robots[0], self.ball, 20, ball_trajectory, leftSide=not self.mray, upperLim=110, lowerLim=70)
                else:
                    action.screenOutBall(self.robots[0], self.ball, 20, leftSide=not self.mray, upperLim=110, lowerLim=70)
                
        if ((abs(self.robots[0].theta) < deg2rad(10)) or (abs(self.robots[0].theta) > deg2rad(170))) and (self.robots[0].xPos < 25 or self.robots[0].xPos > 225):
            self.robots[0].contStopped += 1
        else:
            self.robots[0].contStopped = 0
    
    def basicStgAtt(self):
        """Basic alternative strategy"""
        # listRobots = [self.robots[0], self.robots[1], self.robots[2],
        #               self.enemy_robots[0], self.enemy_robots[1], self.enemy_robots[2], self.enemy_robots[3], self.enemy_robots[4]]
        friends = [self.robots[0], self.robots[1], self.robots[2]]
        enemys = [self.enemy_robots[0], self.enemy_robots[1], self.enemy_robots[2], self.enemy_robots[3], self.enemy_robots[4]]
        action.followLeader(self.robots[0], self.robots[3], self.robots[4], self.ball, self.enemy_robots[0], self.enemy_robots[1],
                            self.enemy_robots[2], self.enemy_robots[3], self.enemy_robots[4])

        action.screenOutBall(self.robots[0], self.ball, 20, leftSide=not self.mray, upperLim=110, lowerLim=70)
        action.screenOutBall(self.robots[1], self.ball, 90, leftSide=not self.mray, upperLim=85, lowerLim=5)
        action.screenOutBall(self.robots[2], self.ball, 90, leftSide=not self.mray, upperLim=175, lowerLim=95)
    
    def att_stg_5v5(self):
        """Input: None
        Description: Attack strategy to 5v5, 2 defenders follow a semi circle where the ball is projected,
                     3 attakers chase the ball.
        Output: None."""
        ball_coordinates_x, ball_coordinates_y = self.ball.get_coordinates()
        # print(f"Ball's velocity vx = {self.ball.vx}, vy = {self.ball.vy}")

        dist = sqrt((ball_coordinates_x - self.their_goal_x) ** 2 + (ball_coordinates_y - self.their_goal_y) ** 2)

        up_corner_y = 180
        down_corner_y = 0
        
        # Calculates a estimated value to the radius of the enemy deffence
        if self.radius is None:
            self.radius = identify_radius.estimate_radius(self.enemy_robots, self.their_goal_x, self.their_goal_y)
        else:
            self.radius = (identify_radius.estimate_radius(self.enemy_robots, self.their_goal_x, self.their_goal_y) + self.radius) / 2
        
        self.radius = 37.5

        # Robôs 0 e 1 mantém-se na defese
        # Robôs 2, 3 e 4 vão para o ataque

        # Maximum angle to implement the mark of the deffence
        max_angle = math.pi / 6

        # Calculates angle between ball, goal's center and field borders
        if ball_coordinates_y > self.their_goal_y:
            ball_angle = math_aux.angle_between_pair_lines(ball_coordinates_x, ball_coordinates_y, self.their_goal_x, up_corner_y, self.their_goal_x, self.their_goal_y)
        else:
            ball_angle = math_aux.angle_between_pair_lines(ball_coordinates_x, ball_coordinates_y, self.their_goal_x, down_corner_y, self.their_goal_x, self.their_goal_y)
        # print(f"---> Angle between ball an enemy goal = {ball_angle}")

        # Marks deffence depending on the angle value
        if ball_angle < max_angle and dist > self.radius:
            
            if not self.mray and ball_coordinates_x > 125:
                action.marking_their_deffence(self.robots[4], self.their_goal_x - self.radius, self.their_goal_y, self.marking_count)
            elif ball_coordinates_x < 125:
                action.marking_their_deffence(self.robots[4], self.their_goal_x + self.radius, self.their_goal_y, self.marking_count)

            if not self.is_marking:
                self.is_marking = True

                if ball_coordinates_y > 90:
                    self.upper_marking = True
                else:
                    self.upper_marking = False

            self.two_attackers(id_robot1=2, id_robot2=3, upper_marking=self.upper_marking, strategy='defence infiltration')

        else:

            self.marking_count = 0
            self.is_marking = False
            self.three_attackers(id_robot1=2, id_robot2=3, id_robot3=4, strategy='1 leader')
        
        # Sends the remaining robots to form the Troia's Defence
        # (projection onto semi-circle goal centered)
        action.screenOutBall(self.robots[1], self.ball, 90, leftSide=not self.mray, upperLim=180, lowerLim=0)
        action.screenOutBall(self.robots[0], self.ball, 20, leftSide=not self.mray, upperLim=110, lowerLim=70)

    def penaltyModeDefensive(self):
        '''Strategy to defend penalty situations'''
        action.defenderPenalty(self.robots[0], self.ball, leftSide=not self.mray)

        enemys = [self.enemy_robots[0], self.enemy_robots[1], self.enemy_robots[2], self.enemy_robots[3], self.enemy_robots[4]]
        friends = [self.robots[0], self.robots[2], self.robots[3], self.robots[4]]
        action.shoot(self.robots[1],self.ball,not self.mray,friends,enemys)

        friends = [self.robots[0], self.robots[1], self.robots[3], self.robots[4]]
        action.shoot(self.robots[2],self.ball,not self.mray,friends,enemys)

        if not self.mray:
            if self.ball.xPos >53 or self.ball.yPos < 60 or self.ball.yPos > 120:
                self.penaltyDefensive = False
        else:
            if self.ball.xPos < 182 or self.ball.yPos < 60 or self.ball.yPos > 100:
                self.penaltyDefensive = False

    def penaltyModeOffensiveSpin(self):
        '''Strategy to convert penalty offensive situations'''
        action.screenOutBall(self.robots[0], self.ball, 20, leftSide=not self.mray)
        action.screenOutBall(self.robots[1], self.ball, 90, leftSide=not self.mray, upperLim=85, lowerLim=5)

        enemys = [self.enemy_robots[0], self.enemy_robots[1], self.enemy_robots[2], self.enemy_robots[3], self.enemy_robots[4]]
        friends = [self.robots[0], self.robots[1], self.robots[2], self.robots[4]]
        action.shoot(self.robots[3],self.ball,not self.mray,friends,enemys)
        friends = [self.robots[0], self.robots[1], self.robots[3], self.robots[4]]
        action.shoot(self.robots[2],self.ball,not self.mray,friends,enemys)
        if not self.robots[4].dist(self.ball) < 9:
            action.girar(self.robots[4], 100, 100)
        else:
            if self.robots[4].teamYellow:
                if self.robots[4].yPos < 90:
                    action.girar(self.robots[4], 0, 100)
                else:
                    action.girar(self.robots[4], 100, 0)
            else:
                if self.robots[4].yPos > 90:
                    action.girar(self.robots[4], 0, 100)
                else:
                    action.girar(self.robots[4], 100, 0)
        if sqrt((self.ball.xPos-self.robots[4].xPos)**2+(self.ball.yPos-self.robots[4].yPos)**2) > 30:
            self.penaltyOffensive = False
    
    def two_attackers(self, id_robot1, id_robot2, upper_marking, strategy = 'default'):
        """Input: IDs of the 2 attackers.
        Description: Attack strategy to 5v5 with 2 attackers (follower and leader).
        Output: None."""
        match strategy:
            case 'default':
                action.followLeader(self.robots[0], self.robots[id_robot1], self.robots[id_robot2], self.ball,
                                    self.enemy_robots[0], self.enemy_robots[1], self.enemy_robots[2], self.enemy_robots[3], self.enemy_robots[4])
            case 'defence infiltration':
                action.follow_leader_to_quadrant(self.robots[0], self.robots[2], self.robots[3], self.ball,
                                                 self.enemy_robots[0], self.enemy_robots[1], self.enemy_robots[2], self.enemy_robots[3], self.enemy_robots[4],
                                                 upper_marking)
            case _:
                print("Strategy is not defined, hence using default strategy, i.e., basic follower/leader).")
                action.followLeader(self.robots[0], self.robots[id_robot1], self.robots[id_robot2], self.ball,
                                    self.enemy_robots[0], self.enemy_robots[1], self.enemy_robots[2], self.enemy_robots[3], self.enemy_robots[4])
    
    def three_attackers(self, id_robot1, id_robot2, id_robot3, strategy = '1 leader'):
        """Input: IDs of the 3 attackers and attack strategy.
        Description: Attack strategy to 5v5 with 3 attackers between 3 different options.
        Output: None."""

        # print(f"Strategy used: {strategy}")

        match strategy:
            case '3 leaders':
                action.attack_3_leaders(self.ball, self.robots[id_robot1], self.robots[id_robot2], self.robots[id_robot3], self.mray, self.enemy_robots)
            case '2 leaders':
                action.attack_2_leaders(self.ball, self.robots[id_robot1], self.robots[id_robot2], self.robots[id_robot3], self.mray, self.enemy_robots)
            case '1 leader':
                action.attack_1_leaders(self.ball, self.robots[id_robot1], self.robots[id_robot2], self.robots[id_robot3], self.mray, self.enemy_robots)
            case _:
                print("Strategy is not defined, hence using default strategy, i.e., 3 leaders).")
                action.attack_3_leaders(self.ball, self.robots[id_robot1], self.robots[id_robot2], self.robots[id_robot3], self.mray, self.enemy_robots)
