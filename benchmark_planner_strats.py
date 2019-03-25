from __future__ import absolute_import, print_function, unicode_literals

import random
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
import pygame

from coop.planner import CoopPlanner
from coop.players import CoopPlayer
from coop.strategies import AverageGroupDurationStrategy, GroupLengthStrategy
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
    name = _boardname if _boardname is not None else 'pathfindingWorld_MultiPlayer4'
    game = Game('../Cartes/' + name + '.json', SpriteBuilder)
    game.O = Ontology(
        True, '../SpriteSheet-32x32/tiny_spritesheet_ontology.csv')
    game.populate_sprite_names(game.O)
    game.fps = 200  # frames per second
    game.mainiteration()
    game.mask.allow_overlaping_players = True
    # player = game.player


def main():
    iterations = 1
    min_vials = 10
    max_vials = 51
    step_vials = 10

    vials = np.arange(min_vials, max_vials, step_vials)
    average_time = np.zeros_like(vials, dtype=np.float)
    length_time = np.zeros_like(vials, dtype=np.float)
    average_epochs = np.zeros_like(vials, dtype=np.float)
    length_epochs = np.zeros_like(vials, dtype=np.float)

    cpu_time = [average_time, length_time]
    epochs = [average_epochs, length_epochs]

    strategies = [AverageGroupDurationStrategy, GroupLengthStrategy]
    legend = ['Average group duration-based', 'Group length-based']

    for ist, strat in enumerate(strategies):
        for iv, vs in enumerate(vials):
            for _ in range(iterations):
                init()
                exec_time, ep = test(strat, vs)
                cpu_time[ist][iv] += exec_time
                epochs[ist][iv] += ep
            print("Vials:", vs, "done")
        cpu_time[ist] /= iterations
        epochs[ist] /= iterations
        print(legend[ist], "done")
    print(cpu_time)
    print(epochs)
    plot(vials, cpu_time, 'CPU time (s)', legend, '../img/planner_cpu_time')
    plot(vials, epochs, 'Average number of epochs',
         legend, '../img/planner_epoch')
    pygame.quit()


def plot(xs, ys, y_label, legend, name):
    for y in ys:
        plt.plot(xs, y)
    plt.xlabel("Number of vials")
    plt.ylabel(y_label)
    plt.legend(legend)
    plt.savefig(name + '.png')
    plt.clf()


def is_over(scores, vials):
    for sc in scores:
        if sc < vials:
            return False
    return True


def test(strategy, vials):
    # init()

    # -------------------------------
    # Initialisation
    # -------------------------------

    players = [o for o in game.layers['joueur']]
    nbPlayers = len(players)
    score = [0] * nbPlayers

    # on localise tous les états initiaux (loc du joueur)
    initStates = [o.get_rowcol() for o in game.layers['joueur']]

    # on localise tous les murs
    wallStates = [w.get_rowcol() for w in game.layers['obstacle']]

    # Placement aleatoire des fioles
    goalPos = []
    for o in game.layers['ramassable']:  # les rouges puis jaunes puis bleues
        # et on met la fiole qqpart au hasard
        x = random.randint(0, 19)
        y = random.randint(0, 19)
        while (x, y) in wallStates + goalPos:
            x = random.randint(0, 19)
            y = random.randint(0, 19)
        o.set_rowcol(x, y)
        goalPos.append((x, y))
        game.layers['ramassable'].add(o)
        game.mainiteration()

    # on localise tous les objets ramassables
    goalStates = [o.get_rowcol() for o in game.layers['ramassable']]

    # Boucle principale de déplacements
    goalPos = [goalStates[i:i + 1] for i in range(nbPlayers)]

    t_0 = time.process_time()

    Node.set_world_dimensions(game.spriteBuilder.rowsize,

                              game.spriteBuilder.colsize)

    CoopPlayer.set_cut_off_limit(5)

    coop_planner = CoopPlanner(initStates, goalPos, wallStates)

    cpu_time = time.process_time() - t_0

    epoch = 0
    average_epochs = 0

    done = [False] * nbPlayers

    while not is_over(score, vials):
        epoch += 1
        current = []

        for j in range(nbPlayers):  # on fait bouger chaque joueur séquentiellement
            t_0 = time.process_time()
            next_row, next_col = coop_planner.next()
            t_f = time.process_time()

            cpu_time += t_f - t_0

            current.append((next_row, next_col))

            # and ((next_row,next_col) not in posPlayers)
            if ((next_row, next_col) not in wallStates) and next_row >= 0 and next_row <= 19 and next_col >= 0 and next_col <= 19:
                players[j].set_rowcol(next_row, next_col)

            # si on a  trouvé un objet on le ramasse
            if (next_row, next_col) in goalPos[j]:
                o = players[j].ramasse(game.layers)
                # on enlève ce goalState de la liste
                goalPos[j].remove((next_row, next_col))
                score[j] += 1

                if score[j] < vials:
                    # et on remet un même objet à un autre endroit
                    x = random.randint(0, 19)
                    y = random.randint(0, 19)
                    while (x, y) in wallStates + current + [el for sub in goalPos for el in sub]:
                        x = random.randint(0, 19)
                        y = random.randint(0, 19)
                    o.set_rowcol(x, y)
                    goalPos[j].append((x, y))  # on ajoute ce nouveau goalState
                    game.layers['ramassable'].add(o)
                    coop_planner.add_goal(j, (x, y))
                elif not done[j]:
                    done[j] = True
                    average_epochs += epoch

        # print(score)
        game.mainiteration()

    return cpu_time, (average_epochs / nbPlayers)
    # pygame.quit()


if __name__ == '__main__':
    main()
