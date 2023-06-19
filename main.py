from bridge import (Actuator, Replacer, Vision, Referee,
                        NUM_BOTS, convert_angle, Entity)

from math import pi, fmod, atan2, fabs
import sys

from simClasses import *
import action
import fouls

import time

from strategy import *

if __name__ == "__main__":

    try:
        team = sys.argv[1]
    except:
        print("[ERRO]")
        print("Digite como parâmetro o time que você ira jogar!")
        print("Exemplo: python3 main.py yellow")
        sys.exit()

    if team != "blue" and team != "yellow":
        print("[ERRO]")
        print("Selecione um time válido! ")
        print("Para jogar com o azul, o primeiro argumento deve ser 'blue'")
        print("Para jogar com o amarelo, o primeiro argumento deve ser 'yellow'")
        sys.exit()

    # Choose team (my robots are yellow)
    if team == "yellow":
        mray = True
    else:
        mray = False

    # Initialize all clients
    actuator = Actuator(mray, "127.0.0.1", 20011)
    replacement = Replacer(mray, "224.5.23.2", 10004)
    vision = Vision(mray, "224.0.0.1", 10002)
    referee = Referee(mray, "224.5.23.2", 10003)

    # Initialize all  objects
    num_robots = 5

    robots = []
    for i in range(num_robots):
        robot = Robot(i, actuator, mray)
        robots.append(robot)

    enemy_robots = []
    for i in range(num_robots):
        robot = Robot(i, actuator, not mray)
        enemy_robots.append(robot)

    # for robot in robots:
    #     robot.set_enemies(enemy_robots)
    #     robot.set_friends(robots.copy())

    ball = Ball()

    strategy = Strategy(robots, enemy_robots, ball, mray)

    # Main infinite loop
    while True:
        t1 = time.time()
        # Atualiza a situação das faltas
        referee.update()
        ref_data = referee.get_data()

        # Atualiza os dados da visão
        vision.update()
        field = vision.get_field_data()

        data_our_bot = field["our_bots"]        #Salva os dados dos robôs aliados
        data_their_bots = field["their_bots"]   #Salva os dados dos robôs inimigos
        data_ball = field["ball"]               #Salva os dados da bola

        # Atualiza em cada objeto do campo os dados da visão
        for index, robot in enumerate(robots):
            robot.set_simulator_data(data_our_bot[index])
        
        for index, robot in enumerate(enemy_robots):
            robot.set_simulator_data(data_their_bots[index])

        ball.set_simulator_data(data_ball)

        if ref_data["game_on"]:
            # Se o modo de jogo estiver em "Game on"
            strategy.coach()

        elif ref_data["foul"] == 1 and ref_data["yellow"] == (not mray):
            #Detectando penalti defensivo
            strategy.penaltyDefensive = True
            actuator.stop()
            fouls.replacement_fouls(replacement,ref_data,mray)

        elif ref_data["foul"] == 1 and ref_data["yellow"] == (mray):
            #Detectando penalti ofensivo
            strategy.penaltyOffensive = True
            actuator.stop()
            fouls.replacement_fouls(replacement,ref_data,mray)

        elif ref_data["foul"] != 7:
            if ref_data["foul"] != 5: # Mudando a flag exceto em caso de Stop
                strategy.penaltyOffensive = False
                strategy.penaltyDefensive = False
            fouls.replacement_fouls(replacement,ref_data,mray)
            actuator.stop()

        else:
            actuator.stop()

        t2 = time.time()
        if t2-t1<1/60:
            time.sleep(1/60 - (t2-t1))
