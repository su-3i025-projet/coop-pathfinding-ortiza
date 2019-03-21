# -*- coding: utf-8 -*-

# Nicolas, 2015-11-18

from __future__ import absolute_import, print_function, unicode_literals

import random
import sys
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
    game.fps = 50  # frames per second
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

    # initStates = [(3, 3), (6, 3), (12, 13)]
    # for i, p in enumerate(game.layers['joueur']):
    #     p.set_rowcol(*initStates[i])
    #
    # goalStates = [(6, 1), (3, 8), (19, 19)]
    # for i, o in enumerate(game.layers['ramassable']):
    #     o.set_rowcol(*goalStates[i])
    #
    # game.mainiteration()

    # on localise tous les états initiaux(loc du joueur)
    initStates = [o.get_rowcol() for o in game.layers['joueur']]
    print("Init states:", initStates)

    # on localise tous les murs
    wallStates = [w.get_rowcol() for w in game.layers['obstacle']]
    # print ("Wall states:", wallStates)

    # -------------------------------
    # Placement aleatoire des fioles
    # -------------------------------

    for o in game.layers['ramassable']:  # les rouges puis jaunes puis bleues
        # et on met la fiole qqpart au hasard
        x = random.randint(7, 12)
        y = random.randint(7, 12)
        while (x, y) in wallStates:
            x = random.randint(7, 12)
            y = random.randint(7, 12)
        o.set_rowcol(x, y)
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
    for i in range(nbPlayers):
        coop_players.append(AdvancedPlayer(
            initStates[i], goalPos[i], wallStates))

    Node.set_world_dimensions(game.spriteBuilder.rowsize,
                              game.spriteBuilder.colsize)

    AdvancedPlayer.set_cooperation_period(8)
    AdvancedPlayer.set_search_epochs()

    # posPlayers=initStates
    # while True:
    #     pass

    previous = [(-1, -1)] * nbPlayers
    time = 1
    for i in range(iterations):

        for j in range(nbPlayers):  # on fait bouger chaque joueur séquentiellement
            # _ = input("next")
            next_row, next_col = coop_players[j].next

            # and ((next_row,next_col) not in posPlayers)
            if ((next_row, next_col) not in wallStates) and next_row >= 0 and next_row <= 19 and next_col >= 0 and next_col <= 19:
                players[j].set_rowcol(next_row, next_col)
                print("player", j, "in (", next_row, next_col, ") at t =", time)
                # game.mainiteration()

            # si on a  trouvé un objet on le ramasse
            print("\tgoal", goalPos[j])
            if (next_row, next_col) in goalPos[j]:
                o = players[j].ramasse(game.layers)
                # game.mainiteration()
                print("\nObjet trouvé par le joueur ", j, end='')
                # on enlève ce goalState de la liste
                goalPos[j].remove((next_row, next_col))
                score[j] += 1

                # et on remet un même objet à un autre endroit
                x = random.randint(3, 15)
                y = random.randint(3, 15)
                while (x, y) in wallStates:
                    x = random.randint(3, 15)
                    y = random.randint(3, 15)
                o.set_rowcol(x, y)
                goalPos[j].append((x, y))  # on ajoute ce nouveau goalState
                game.layers['ramassable'].add(o)
                coop_players[j].add_goal((x, y))
                print('\tnew goal at', (x, y))
                # print("==================>", coop_players[j].goal_positions)

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
        time += 1
        print("Ended iteration", i + 1)
        print("===================================")
        # break

    print("Reservation table length:", len(AdvancedPlayer.reservation_table))
    print("Average number of A* iterations:",
          TimeAStar.NB_ITERS / TimeAStar.NB_CALLS)
    print("scores:", score)
    pygame.quit()


if __name__ == '__main__':
    main()
