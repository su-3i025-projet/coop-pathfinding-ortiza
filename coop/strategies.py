#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 14:20:19 2019

@author: angelo
"""


class Goal_Choice_Strategy:
    """
    Strategy for the choice of the next goal to pursue
    """

    def __init__(self, goal_positions):
        self.goal_positions = goal_positions

    def get_next_goal(self, current_position):
        """
        Returns the preferred next goal

        -------------------
        args:
            current_position (tuple[int]): the coordinates of the current
                position of the agent
        -------------------
        """
        raise NotImplementedError


class Naive_Strategy(Goal_Choice_Strategy):
    """
    Firsts-as-a-rule goal choice strategy
    """

    def __init__(self, goal_positions):
        Goal_Choice_Strategy.__init__(self, goal_positions)

    def get_next_goal(self, current_position):
        """
        Returns the first goal in the list

        -------------------
        return:
            (tuple[int]): the coordinates of the agent's next goal
        -------------------
        """
        return self.goal_positions.pop(0)


class Closest_Strategy(Goal_Choice_Strategy):
    """
    Closest-first goal choice strategy
    """

    def __init__(self, goal_positions):
        Goal_Choice_Strategy.__init__(self, goal_positions)

    def get_next_goal(self, current_position):
        """
        Returns the closest goal to the given current position

        -------------------
        return:
            (tuple[int]): the coordinates of the agent's next goal
        -------------------
        """
        dists = [distance(current_position, goal_pos)
                 for goal_pos in self.goal_positions]
        index_of_best = dists.index(min(dists))
        return self.goal_positions.pop(index_of_best)


class Sequence_Sorting_Strategy:

    def __init__(self, players, sequence):
        self.players = players
        self.sequence = sequence

    def sort(self):
        raise NotImplementedError

    def get_next_group(self, current_group):
        raise NotImplementedError


class Average_Group_Duration_Strategy(Sequence_Sorting_Strategy):
    """
    Quickest groups first strategy
    """

    def __init__(self, players, sequence):
        Sequence_Sorting_Strategy.__init__(self, players, sequence)

    def sort(self):
        durations = {}
        for group in self.sequence:
            durations[tuple(group)] = sum([len(players[p].steps)
                                           for p in group]) / len(group)
        self.sequence.sort(key=lambda x: durations[tuple(x)])

    def get_next_group(self, current_group):
        raise NotImplementedError


class Group_Length_Strategy(Goal_Choice_Strategy):
    """
    Larger groups first strategy
    """

    def __init__(self, players, sequence):
        Sequence_Sorting_Strategy.__init__(self, players, sequence)

    def sort(self):
        lengths = {tuple(group): len(group) for group in self.sequence}
        self.sequence.sort(key=lambda x: lengths[tuple(x)])

    def get_next_group(self, current_group):
        raise NotImplementedError
