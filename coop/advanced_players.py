#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 18 13:40:19 2019

@author: angelo
"""

import random
from functools import reduce

from .players import CoopPlayer
from .strategies import NaiveStrategy
from .tools import AStar, Node


class TimeNode(Node):  # TODO: doc
    """
    Class representing a node in A* algorithm's state graph
    """

    def __init__(self, a_star, x, y, t=None, parent=None):
        super().__init__(a_star, x, y, parent)
        if t is not None:
            self.t = t
        elif self.has_parent():
            self.t = parent.t + 1
        else:
            self.t = 0
        if self.has_parent() and self.get_step() == (0, 0, 1):
            self.g = parent.g

    @property
    def position(self):
        return *super().position, self.t

    @property
    def coordinates(self):
        return super().position

    @property
    def h(self):
        """
        Calculates the true distance heuristic value for this node, i.e.
        the exact distance between this node and the goalManhattan

        -------------------
        return:
            (int): the true distance heuristic value for this node
        -------------------
        """
        return self.a_star.true_distance(self.position)

    def get_valid_neighbours(self, player_id):  # TODO: doc
        """
        Finds all the valid neighbours of this node, i.e. those nodes
        wrapping non-wall and inside-the-grid cells

        -------------------
        return:
            (list[Node]): the list of all this node's valid neighbours
        -------------------
        """
        shifts = [(1, 0), (-1, 0), (0, 1), (0, -1), (0, 0)]
        neighbours = [(self.x + dx, self.y + dy) for dx, dy in shifts]
        valid_neighbours = []
        t = self.t + 1
        for x, y in neighbours:
            if (x, y) in self.a_star.walls or not TimeNode.is_valid(x, y):
                continue
            try:
                if AdvancedPlayer.reservation_table[(x, y, t)] != player_id:
                    continue
            except:
                pass
            valid_neighbours.append(
                TimeNode(self.a_star, x, y, t, parent=self))
        return valid_neighbours

    def get_step(self):
        """
        Calculates the step to take to come to this node from its parent

        -------------------
        return:
            (tuple[int]): the step the agent must take to come to this node
                from its parent
        -------------------
        """
        return *super().get_step(), self.t - parent.t

    def __repr__(self):
        return '[' + super().__repr__() + f' at t = {self.t}]'

    def __eq__(self, other):
        """
        Tests whether the given node is equal to this node, based solely on
        their coordinates
        """
        return self.__class__ == other.__class__ and \
            self.x == other.x and self.y == other.y and self.t == other.t

    def node_equals(self, other):
        return super().__eq__(other)


class TimeAStar(AStar):  # TODO:  doc
    """
    Class implementing the A* algorithm
    """

    def __init__(self, initial_state, goal_state, depth, initial_instant, player_id, walls):
        self.initial_state = TimeNode(self, *initial_state, t=initial_instant)
        self.goal_state = TimeNode(self, *goal_state)
        self.player_id = player_id
        self.walls = walls
        self.last_instant = timepoint + depth
        # the fringe
        self.open_set = [self.initial_state]
        # the set of extended nodes
        self.closed_set = []
        # backwards search
        self.backwards_search = None

    def true_distance(self, current_position):
        if self.backwards_search is None:
            self.backwards_search = AStar(
                self.goal_state.position, current_position, self.walls)
            self.backwards_search.run()
        node = self.backwards_search.get_node_at(current_position)
        if node is None:
            self.backwards_search.set_new_goal(current_position)
            self.backwards_search.run()
            node = self.backwards_search.get_node_at(current_position)
        return node.g

    def run(self):
        """
        Runs A* algorithm


        -------------------
        return:
            (list[tuple[int]]): the reversed list of steps to take to get
                to the goal state from the initial state
        -------------------
        """
        while not self.open_set_is_empty():
            current_state = self.select_best()
            if current_state.node_equals(self.goal_state):
                self.goal_state.t = current_state.t
                self.goal_state.set_parent(current_state.parent)
                return self.get_reversed_step_sequence()
            neighbours = current_state.get_valid_neighbours(self.player_id)
            not_extd_neighbours = self.get_not_extended(neighbours)
            self.add_to_open_set(not_extd_neighbours)
            self.closed_set.append(current_state)
            if current_state.t >= self.last_instant:
                self.goal_state = current_state
                return self.get_reversed_step_sequence()

    def get_reversed_step_sequence(self):
        """
        Finds the step sequence to follow from the initial state in order to
        get to the goal state

        -------------------
        return:
            (list[tuple[int]]): the reversed list of steps to take to get
                to the goal state from the initial state
        -------------------
        """
        t = self.goal_state.t
        current_state = self.goal_state
        steps = []
        while current_state != self.initial_state:
            steps.append(current_state.get_step())
            AdvancedPlayer.reservation_table[(
                *current_state.coordinates, t)] = player_id
            t -= 1
            current_state = current_state.parent
        return steps


class AdvancedPlayer(CoopPlayer):
    """
    Class representing an advanced cooperative player that utilises
    a space-time reservation table to handle collisions
    """

    time = 0
    coop_period = None  # half the depth
    players = []
    reservation_table = {}
    counter = 0

    def __init__(self, initial_position, goal_positions, walls, goal_choice=NaiveStrategy):
        super().__init__(initial_position, goal_positions, walls, goal_choice)
        self.id = counter
        AdvancedPlayer.counter += 1
        self.search_instant = None
        AdvancedPlayer.reservation_table[(*initial_position, 0)] = self.id
        AdvancedPlayer.players.append(self)

    def set_search_instants(cls):
        n = len(cls.players)
        d = cls.coop_period
        players_per_instant = n // d
        cnt = 0
        for t in range(d):
            for _ in range(players_per_instant):
                cnt += 1
                cls.players[cnt].search_instant = t
        rem = n - cnt
        for t in range(d):
            if rem == 0:
                break
            rem -= 1
            cnt += 1
            cls.players[cnt].search_instant = t

    @property
    def others(self):
        """
        The list of all the cooperative peers of this agent

        -------------------
        return:
            (list[AdvancedPlayer], list[AdvancedPlayer]): the list of the cooperative
                peers already placed during this iteration, and the list of
                those yet to be placed
        -------------------
        """
        my_index = AdvancedPlayer.players.index(self)
        return AdvancedPlayer.players[:my_index], AdvancedPlayer.players[my_index + 1:]

    def find_path_to_goal(self):  # TODO: what about resuming (see paper!)
        """
        Finds a path to a path considering players already fixed on the grid

        -------------------
        args:
            [optional] placed (list[Coop_Player]): the list of players already
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

    def is_last(self):
        return self.id == len(AdvancedPlayer.players) - 1

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
        if self.search_instant == AdvancedPlayer.time:
            self.find_path_to_goal()
            self.current_goal = None
            self.search_instant += AdvancedPlayer.coop_period

        # the agent succeded and wishes to meet another goal
        if self.has_next_goal():
            self.clear_unused_following_path()  # TODO: clear reservation table
            self.find_path_to_goal()
            self.current_goal = None

        # when along the path
        elif self.has_next_step():
            self.get_next_position()

        if self.is_last():
            AdvancedPlayer.time += 1

        return self.current_position
