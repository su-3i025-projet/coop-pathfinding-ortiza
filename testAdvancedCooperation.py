# -*- coding: utf-8 -*-

# Nicolas, 2015-11-18

from __future__ import absolute_import, print_function, unicode_literals

import random
import sys
import time
from itertools import chain

import numpy as np
import pygame

import utils.glo as glo
from coop.advanced_players import AdvancedPlayer, TimeAStar
from coop.tools import Node
from utils.gameclass import Game, check_init_game_done
from utils.ontology import Ontology
from utils.players import Player
from utils.sprite import MovingSprite
from utils.spritebuilder import SpriteBuilder

# ---- ---- ---- ---- ---- ----
# ---- Main                ----
# ---- ---- ---- ---- ---- ----

game = Game()


def init(_boardname=None):
    global player, game
    # pathfindingWorld_MultiPlayer4
    name = _boardname if _boardname is not None else 'pathfindingWorld_MultiPlayer4'
    game = Game('../Cartes/' + name + '.json', SpriteBuilder)
    game.O = Ontology(
        True, '../SpriteSheet-32x32/tiny_spritesheet_ontology.csv')
    game.populate_sprite_names(game.O)
    game.fps = 70  # frames per second
    game.mainiteration()
    game.mask.allow_overlaping_players = True
    # player = game.player


def main():

    # for arg in sys.argv:
    iterations = 50  # default
    if len(sys.argv) == 2:
        iterations = int(sys.argv[1])
    print("Iterations: ")
    print(iterations)

    init()

    # -------------------------------
    # Initialisation
    # -------------------------------

    players = [o for o in game.layers['joueur']]
    nbPlayers = len(players)
    score = [0] * nbPlayers

    # on localise tous les états initiaux(loc du joueur)
    initStates = [o.get_rowcol() for o in game.layers['joueur']]
    print("Init states:", initStates)

    # on localise tous les murs
    wallStates = [w.get_rowcol() for w in game.layers['obstacle']]
    # print ("Wall states:", wallStates)

    # -------------------------------
    # Placement aleatoire des fioles
    # -------------------------------
    goalPos = []
    for o in game.layers['ramassable']:  # les rouges puis jaunes puis bleues
        # et on met la fiole qqpart au hasard
        x = random.randint(7, 12)
        y = random.randint(7, 12)
        while (x, y) in wallStates + goalPos:
            x = random.randint(7, 12)
            y = random.randint(7, 12)
        o.set_rowcol(x, y)
        goalPos.append((x, y))
        game.layers['ramassable'].add(o)
        game.mainiteration()

    print(game.layers['ramassable'])

    # on localise tous les objets ramassables
    goalStates = [o.get_rowcol() for o in game.layers['ramassable']]
    print("Goal states:", goalStates)

    # on donne a chaque joueur une fiole a ramasser
    # en essayant de faire correspondre les couleurs pour que ce soit plus simple à suivre

    # -------------------------------
    # Boucle principale de déplacements
    # -------------------------------

    goalPos = {i: goalStates[i:i + 1] for i in range(nbPlayers)}

    coop_players = []

    t_0 = time.process_time()

    for i in range(nbPlayers):
        coop_players.append(AdvancedPlayer(
            initStates[i], goalPos[i], wallStates))

    Node.set_world_dimensions(game.spriteBuilder.rowsize,
                              game.spriteBuilder.colsize)

    AdvancedPlayer.set_pathfinding_frequence(6)
    AdvancedPlayer.set_search_epochs()

    cpu_time = time.process_time() - t_0

    previous = [(-1, -1)] * nbPlayers
    epoch = 0

    done = 0

    for i in range(iterations):
        epoch += 1

        for j in range(nbPlayers):  # on fait bouger chaque joueur séquentiellement
            t_0 = time.process_time()
            next_row, next_col = coop_players[j].next()
            t_f = time.process_time()

            cpu_time += t_f - t_0

            # and ((next_row,next_col) not in posPlayers)
            if ((next_row, next_col) not in wallStates) and next_row >= 0 and next_row <= 19 and next_col >= 0 and next_col <= 19:
                players[j].set_rowcol(next_row, next_col)
                print("player", j, "in (", next_row, next_col, ") at t =", epoch)
                # game.mainiteration()

            # si on a  trouvé un objet on le ramasse
            print("\tgoal", goalPos[j])
            if (next_row, next_col) in goalPos[j]:
                o = players[j].ramasse(game.layers)
                # game.mainiteration()
                print("\nObjet trouvé par le joueur ", j)  # , end='')
                # on enlève ce goalState de la liste
                goalPos[j].remove((next_row, next_col))
                score[j] += 1
                done += 1

                # et on remet un même objet à un autre endroit
                # x = random.randint(3, 15)
                # y = random.randint(3, 15)
                # while (x, y) in wallStates:
                #     x = random.randint(3, 15)
                #     y = random.randint(3, 15)
                # o.set_rowcol(x, y)
                # goalPos[j].append((x, y))  # on ajoute ce nouveau goalState
                # game.layers['ramassable'].add(o)
                # coop_players[j].add_goal((x, y))
                # print('\tnew goal at', (x, y))

            if done == nbPlayers:
                break

        current = [p.current_position for p in coop_players]
        collision = False
        concurrent = [(p, q) for p in range(nbPlayers)
                      for q in range(p + 1, nbPlayers)]
        for p, q in concurrent:
            if current[p] == current[q]:
                collision = True
                break
            if previous[p] == current[q] and current[p] == previous[q]:
                collision = True
                break
        if collision:
            print("===== collision =====")
            print(AdvancedPlayer.reservation_table)
            while True:
                pass
        previous = current
        game.mainiteration()
        print("Ended iteration", i + 1)
        print("===================================")
        if done == nbPlayers:
            break

    print("===================", "STATS", "===================")
    print("Total CPU time:", cpu_time)
    print("Number of epochs needed to complete the tasks:", epoch)
    print("Final reservation table length:",
          len(AdvancedPlayer.reservation_table))
    print("Average number of space-time A* iterations:",
          TimeAStar.NB_ITERS / TimeAStar.NB_CALLS)
    print("scores:", score)
    pygame.quit()


if __name__ == '__main__':
    main()
