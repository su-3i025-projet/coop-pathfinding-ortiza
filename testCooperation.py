# -*- coding: utf-8 -*-

# Nicolas, 2015-11-18

from __future__ import absolute_import, print_function, unicode_literals

import random
import sys
from itertools import chain

import numpy as np
import pygame

import utils.glo as glo
from coop.players import CoopPlayer
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
    name = _boardname if _boardname is not None else 'pathfindingWorld_MultiPlayer1'
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

    initStates = [(3, 3), (6, 3), (12, 13)]
    for i, p in enumerate(game.layers['joueur']):
        p.set_rowcol(*initStates[i])

    goalStates = [(6, 1), (3, 8), (19, 19)]
    for i, o in enumerate(game.layers['ramassable']):
        o.set_rowcol(*goalStates[i])

    game.mainiteration()

    # on localise tous les états initiaux (loc du joueur)
    # initStates = [o.get_rowcol() for o in game.layers['joueur']]
    # print("Init states:", initStates)

    # on localise tous les murs
    wallStates = [w.get_rowcol() for w in game.layers['obstacle']]
    # print ("Wall states:", wallStates)

    # -------------------------------
    # Placement aleatoire des fioles
    # -------------------------------

    # for o in game.layers['ramassable']:  # les rouges puis jaunes puis bleues
    #     # et on met la fiole qqpart au hasard
    #     x = random.randint(7, 12)
    #     y = random.randint(7, 12)
    #     while (x, y) in wallStates:
    #         x = random.randint(7, 12)
    #         y = random.randint(7, 12)
    #     o.set_rowcol(x, y)
    #     game.layers['ramassable'].add(o)
    #     game.mainiteration()

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
        coop_players.append(CoopPlayer(
            initStates[i], goalPos[i], wallStates))

    Node.set_world_dimensions(game.spriteBuilder.rowsize,
                              game.spriteBuilder.colsize)

    CoopPlayer.set_cut_off_limit(5)

    # posPlayers=initStates

    previous = [(-1, -1), (-1, -1), (-1, -1)]

    for i in range(iterations):

        for j in range(nbPlayers):  # on fait bouger chaque joueur séquentiellement
            next_row, next_col = coop_players[j].next

            # and ((next_row,next_col) not in posPlayers)
            if ((next_row, next_col) not in wallStates) and next_row >= 0 and next_row <= 19 and next_col >= 0 and next_col <= 19:
                players[j].set_rowcol(next_row, next_col)
                print("pos :", j, next_row, next_col)
                # game.mainiteration()

            # si on a  trouvé un objet on le ramasse
            print(next_row, next_col, goalPos[j])
            if (next_row, next_col) in goalPos[j]:
                o = players[j].ramasse(game.layers)
                # game.mainiteration()
                print("Objet trouvé par le joueur ", j)
                # on enlève ce goalState de la liste
                goalPos[j].remove((next_row, next_col))
                score[j] += 1

                # et on remet un même objet à un autre endroit
                x = random.randint(6, 12)
                y = random.randint(6, 12)
                while (x, y) in wallStates:
                    x = random.randint(6, 12)
                    y = random.randint(6, 12)
                o.set_rowcol(x, y)
                goalPos[j].append((x, y))  # on ajoute ce nouveau goalState
                game.layers['ramassable'].add(o)
                coop_players[j].add_goal((x, y))
                # print("==================>", coop_players[j].goal_positions)

        current = [p.current_position for p in coop_players]
        collision = False
        if current[0] == current[1] or current[0] == current[2] or current[1] == current[2]:
            collision = True
        if (previous[0] == current[1] and current[0] == previous[1]) or \
                (previous[0] == current[2] and current[0] == previous[2]) or \
                (previous[1] == current[2] and current[1] == previous[2]):
            collision = True
        if collision:
            print("===== collision =====")
            while True:
                pass
        previous = current
        game.mainiteration()

        # break

    print("scores:", score)
    pygame.quit()


if __name__ == '__main__':
    main()
