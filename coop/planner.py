#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 14:20:19 2019

@author: angelo
"""

from .players import CoopPlayer
from .strategies import GroupLengthStrategy


class CoopPlanner:
    """
    Class representing a cooperative entity that calculates the path
    of all of its players and defines a sequence of groups of players
    whose paths do not cross each other
    """

    def __init__(self, initial_position, goal_positions, walls, seq_sorting_choice=GroupLengthStrategy):
        for pos, goal in zip(initial_position, goal_positions):
            print(pos, goal)
        self.players = [CoopPlayer(init_pos, goal_pos, walls)
                        for init_pos, goal_pos in zip(initial_position, goal_positions)]
        self.walls = walls
        self.seq_sorting_choice = seq_sorting_choice
        self.sequence = []
        self.current_player = -1
        self.collision_baseline_positions = []
        self.initialise_sequence()

    def initialise_sequence(self):
        self.find_initial_paths()
        self.update_sequence([i for i, _ in enumerate(self.players)])
        self.seq_sorting_choice = self.seq_sorting_choice(
            self.players, self.sequence)
        # print(self.sequence)
        self.seq_sorting_choice.sort()
        # print(self.sequence)
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
            bef, aft = player.others
            others = [oth.current_goal for oth in bef] + \
                [oth.goal_positions[0] for oth in aft]
            player.find_path_to_goal(others)

    def update_paths(self):
        for player in self.players:
            bef, aft = player.others
            others = [oth.current_goal for oth in bef + aft]
            others += [oth.current_position for oth in bef + aft]
            player.find_path_to_goal(others, resume=True)

    def exists_collision(self, player1, player2):
        # for the stationary case
        restore = False

        # save current state
        saved_current_pos2, saved_steps2 = player2.current_position, player2.steps
        player2.steps = player2.steps[:]

        # find all the cells of the to-be-added agent's path for the first and only time
        if self.collision_baseline_positions == []:
            # save current state
            saved_current_pos1, saved_steps1 = player1.current_position, player1.steps
            player1.steps = player1.steps[:]
            restore = True

            self.collision_baseline_positions.append(saved_current_pos1)
            while player1.has_next_step():
                self.collision_baseline_positions.append(
                    player1.get_next_position())

        # find all the cells of an already-in-the-sequence agent's path
        other_positions = [saved_current_pos2]
        while player2.has_next_step():
            other_positions.append(player2.get_next_position())

        # there is a collision iff at least one cell belongs to both paths
        collision = False
        for pos1 in self.collision_baseline_positions:
            if pos1 in other_positions:
                collision = True
                break

        # restore initial state
        if restore:
            player1.current_position, player1.steps = saved_current_pos1, saved_steps1
        player2.current_position, player2.steps = saved_current_pos2, saved_steps2

        return collision

    def add_to_sequence(self, player):
        was_dealt_with = False

        # seek a place preferently at the end
        for other_indices in self.sequence[::-1]:
            must_add = True
            for other in other_indices:
                if self.exists_collision(self.players[player], self.players[other]):
                    must_add = False
                    break
            if must_add:
                other_indices.append(player)
                was_dealt_with = True
                break

        # reset path cells storage for <player>
        self.collision_baseline_positions = []

        if not was_dealt_with:
            self.sequence.append([player])

    def update_sequence(self, players):
        for player in players:
            self.add_to_sequence(player)

    @property
    def next(self):
        self.current_player = (self.current_player + 1) % len(self.players)

        # calculate a new path for the agent when already met its previous one
        if not self.players[self.current_player].has_next_step():
            bef, aft = self.players[self.current_player].others
            placed = [oth.current_position for oth in bef + aft]
            self.players[self.current_player].find_path_to_goal(
                placed=placed)
            print(self.players[self.current_player].steps)
            self.add_to_sequence(self.current_player)

        # change the current active group when empty
        if self.current_group == []:
            self.sequence.pop(0)
            self.current_group = self.sequence[0]
            self.update_paths()

        # the current agent will move if being part of the current active group
        if self.current_player in self.current_group:
            next_position = self.players[self.current_player].get_next_position(
            )
            if self.players[self.current_player].is_at_goal():
                self.current_group.remove(self.current_player)
            return next_position

        return self.players[self.current_player].current_position
