#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 14:20:19 2019

@author: angelo
"""

import random
from functools import reduce

from .strategies import NaiveStrategy
from .tools import AStar, Node


class CoopPlayer:
    """
    Class representing a cooperative player that handles
    collisions on a rolling basis
    """

    PLAYERS = []
    CUT_OFF_LIMIT = None

    def __init__(self, initial_position, goal_positions, walls, goal_choice=NaiveStrategy):
        self.initial_position = initial_position
        self.current_position = initial_position
        self.goal_positions = goal_positions[:]
        self.current_goal = None
        self.walls = walls
        self.a_star = None
        self.steps = []
        self.goal_choice = goal_choice(self.goal_positions)
        CoopPlayer.PLAYERS.append(self)

    def add_goal(self, goal_position):
        """
        Adds a new goal for this agent

        -------------------
        args:
            goal_position (tuple[int]): the coordinates of a new goal for the agent
        -------------------
        """
        self.goal_positions.append(goal_position)

    @property
    def others(self):
        """
        The list of all the cooperative peers of this agent

        -------------------
        return:
            (list[CoopPlayer], list[CoopPlayer]): the list of the cooperative
                peers already placed during this iteration, and the list of
                those yet to be placed
        -------------------
        """
        my_index = CoopPlayer.PLAYERS.index(self)
        return CoopPlayer.PLAYERS[:my_index], CoopPlayer.PLAYERS[my_index + 1:]

    def has_next_step(self):
        """
        Tests whether this agent has a planified step to take

        -------------------
        return:
            (bool): True iff the list of planified steps is not empty
        -------------------
        """
        return self.steps != []

    def has_next_goal(self):
        """
        Tests whether this agent has a goal to pursue

        -------------------
        return:
            (bool): True iff the list of goals is not empty
        -------------------
        """
        return self.goal_positions != []

    def is_at_goal(self):
        """
        Tests whether this agent has arrived at its goal

        -------------------
        return:
            (bool): True iff the agent is currently at its goal position
        -------------------
        """
        return self.current_position == self.current_goal

    def find_path_to_goal(self, placed=[], resume=False):
        """
        Finds a path to a goal considering players already fixed on the grid

        -------------------
        args:
            [optional] placed (list[CoopPlayer]): the list of players already
                placed that need to be avoided

            [optional] resume (bool): True iff the agent must resume its pursuit
                of the current goal
        -------------------
        """
        if resume is False:  # for a new path
            self.current_goal = self.goal_choice.get_next_goal(
                self.current_position)
        self.a_star = AStar(self.current_position,
                            self.current_goal, self.walls + placed)
        self.steps = self.a_star.run()

    def go_through_one_another(self, other):
        """
        Tests whether this agent and the given one go would go through
        one another had no collision check been made

        -------------------
        args:
            other (Node): the node to check
        -------------------
        return:
            (bool): True iff the next position of this agent is the current
                position of the other, and vice versa
        -------------------
        """
        return self.current_position == other.next_position and \
            self.next_position == other.current_position

    def collision(self, placed, simultaneous):
        """
        Checks that there would be no collisions had the agents continued
        as usual, and returns a potential collision

        -------------------
        args:
            placed (list[CoopPlayer]): the list of players already placed
                likely to collide with this agent

            simultaneous (list[CoopPlayer]): the list of players to be placed
                at the same time or after this player likely to collide with it
        -------------------
        return:
            (tuple[int]): the position of a potential collision,
                otherwise (NoneType)
        -------------------
        """
        for oth in placed:
            if oth.current_position == self.next_position:
                return self.next_position
        for oth in simultaneous:
            if oth.next_position == self.next_position or self.go_through_one_another(oth):
                return self.next_position
        return None

    def get_valid_shifts(self, obstacle):
        """
        Finds all of the valid neighbours of this node, i.e. those nodes
        wrapping non-wall and inside-the-grid cells

        -------------------
        args:
            obstacle (tuple[int]): the coordinates of an obstacle to be avoided
        -------------------
        return:
            (list[Node]): the list of all this node's valid neighbours
        -------------------
        """
        def is_valid(shift):
            pos = (self.current_position[0] + shift[0],
                   self.current_position[1] + shift[1])
            if pos in self.walls:
                return False
            if not Node.is_valid(*pos):
                return False
            placed, _ = self.others
            if pos in [oth.current_position for oth in placed]:
                return False
            if pos == obstacle:
                return False
            return True

        shifts = [(1, 0), (-1, 0), (0, 1), (0, -1), (0, 0)]
        return [sh for sh in shifts if is_valid(sh)]

    def handle_collision(self, obstacle):
        """
        Recalculates a part of its immediate path considering the given obstacle

        -------------------
        args:
            obstacle (tuple[int]): the coordinates of an obstacle to be
                considered when recalculating its path
        -------------------
        """
        cut_path = self.steps[-CoopPlayer.CUT_OFF_LIMIT:]
        if len(cut_path) < CoopPlayer.CUT_OFF_LIMIT:
            valid_steps = self.get_valid_shifts(obstacle)
            self.steps = [random.choice(valid_steps)]
        else:
            placed, _ = self.others
            nearby_path = AStar(self.current_position, self.get_position_after(cut_path),
                                self.walls + [obstacle] + placed).run()
            self.steps = self.steps[:-CoopPlayer.CUT_OFF_LIMIT] + nearby_path

    def get_position_after(self, reversed_steps):
        """
        Calculates this agent's position if the given step list were to be
        followed in the reversed manner

        -------------------
        args:
            reversed_steps (list[tuple[int]]): the reversed list of steps to take
        -------------------
        return:
            (tuple[int]): this agent's position after taking the given steps
        -------------------
        """
        def take(p, s):
            """
            Takes the step <s> from position <p>
            """
            return tuple([p_i + s_i for p_i, s_i in zip(p, s)])

        return reduce(take, reversed_steps[::-1], self.current_position)

    @property
    def next_position(self):
        """
        Calculates this agent's position according to its next step without
        popping it out

        -------------------
        return:
            (tuple[int]): the next position of the agent
        -------------------
        """
        return self.get_position_after(self.steps[-1:])

    def get_next_position(self):
        """
        Pops the next step out of the list and takes it to calculate its
        next position

        -------------------
        return:
            (tuple[int]): the next position of the agent
        -------------------
        """
        self.current_position = self.next_position
        self.steps.pop()
        return self.current_position

    @property
    def next(self):
        """
        Calculates this agent's next position considering its current
        and remaining goals, and any potential collisions

        -------------------
        return:
            (tuple[int]): the next position of the agent
        -------------------
        """
        # when along the path
        if self.has_next_step():
            obstacle = self.collision(*self.others)
            if obstacle is not None:
                self.handle_collision(obstacle)
            return self.get_next_position()

        # the agent took a random step and needs a new path towards its current goal
        elif self.current_goal is not None and not self.is_at_goal():
            self.find_path_to_goal(resume=True)
            return self.next

        # the agent succeded and wishes to meet another goal
        elif self.has_next_goal():
            self.find_path_to_goal()
            return self.next

        return self.current_position

    @classmethod
    def set_cut_off_limit(cls, cut_point):
        """
        Sets the number of immediate steps to be recalculated when
        handling a collision
        """
        cls.CUT_OFF_LIMIT = cut_point
