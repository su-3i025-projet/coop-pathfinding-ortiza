#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 14:20:19 2019

@author: angelo
"""

from .players import Coop_Player
from .strategies import Group_Length_Strategy


class Coop_Planner:
    """
    Class representing a cooperative entity that calculates the path
    of all of its players and defines a sequence of groups of players
    whose paths do not cross
    """

    def __init__(self, initial_position, goal_positions, walls, seq_sorting_choice=Group_Length_Strategy):
        for pos, goal in zip(initial_position, goal_positions):
            print(pos, goal)
        self.players = [Coop_Player(init_pos, goal_pos, walls)
                        for init_pos, goal_pos in zip(initial_position, goal_positions)]
        self.walls = walls
        self.seq_sorting_choice = seq_sorting_choice
        self.sequence = []
        self.current_player = -1
        self.initialise_sequence()

    def initialise_sequence(self):
        for player in self.players:
            player.find_path_to_goal()
        self.update_sequence([i for i, _ in enumerate(self.players)])
        self.seq_sorting_choice = self.seq_sorting_choice(
            self.players, self.sequence)
        print(self.sequence)
        self.seq_sorting_choice.sort()
        print(self.sequence)
        self.current_group = self.sequence[0]

    def add_goal(self, player, goal_pos):
        """
        Adds a new goal for the given player

        -------------------
        args:
            player (int): the index of the agent

            goal_pos (tuple[int]): the coordinates of a new goal for the agent
        -------------------
        """
        self.players[player].add_goal(goal_pos)

    def find_initial_paths(self):
        for player in self.players:
            player.find_path_to_goal()

    def exists_collision(self, player1, player2):
        # save current state
        saved_current_pos1, saved_steps1 = player1.current_position, player1.steps
        saved_current_pos2, saved_steps2 = player2.current_position, player2.steps
        player1.steps, player2.steps = player1.steps[:], player2.steps[:]

        collision = False
        while player1.has_next_step() and player2.has_next_step():
            if player1.collision([], [player2]) is not None:
                collision = True
                break
            player1.get_next_position()
            player2.get_next_position()

        # restore initial state
        player1.current_position, player1.steps = saved_current_pos1, saved_steps1
        player2.current_position, player2.steps = saved_current_pos2, saved_steps2

        return collision

    def add_to_sequence(self, player):
        was_dealt_with = False
        for other_indices in self.sequence:
            must_add = True
            for other in other_indices:
                if self.exists_collision(self.players[player], self.players[other]):
                    must_add = False
                    break
            if must_add:
                other_indices.append(player)
                was_dealt_with = True
                break
        if not was_dealt_with:
            self.sequence.append([player])

    def update_sequence(self, players):
        for player in players:
            self.add_to_sequence(player)

    @property
    def next(self):
        self.current_player = (self.current_player + 1) % len(self.players)
        print(self.sequence, self.current_player,
              self.players[self.current_player].current_goal)
        if not self.players[self.current_player].has_next_step() and self.players[self.current_player].current_goal is None:
            while True:
                pass
        # print("position:", self.players[self.current_player].current_position,
        #       "steps:", self.players[self.current_player].steps)
        if not self.players[self.current_player].has_next_step():
            self.players[self.current_player].find_path_to_goal(
                placed=[immobile for sublist in self.sequence[1:] for immobile in sublist])
            print(self.players[self.current_player].steps)
            self.add_to_sequence(self.current_player)
        if self.current_group == []:
            self.sequence.pop(0)
            self.current_group = self.sequence[0]
        if self.current_player in self.current_group:
            next_position = self.players[self.current_player].get_next_position(
            )
            if self.players[self.current_player].is_at_goal():
                self.current_group.remove(self.current_player)
            return next_position
        # TODO: if the player stays put in a cell pertaining to the path
        # of another player, then the latter will collide with the former
        # hanlding: check this possibility and random step
        return self.players[self.current_player].current_position
