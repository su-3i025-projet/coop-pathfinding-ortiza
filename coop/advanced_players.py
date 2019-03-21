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
from .tools import AStar, Heap, Node


class TimeNode(Node):
    """
    Class representing a node in space-time A* algorithm's state graph
    """

    def __init__(self, a_star, x, y, t=None, parent=None):
        self.a_star = a_star
        self.parent = parent
        # row index
        self.x = x
        # column index
        self.y = y
        if t is not None:
            self.t = t
        elif self.has_parent():
            self.t = parent.t + 1
        else:
            self.t = 32767  # large number for the goal
        # initialise cost of the path from the root to this node
        self.init_cost()

    def init_cost(self):
        if self.has_parent():
            # staying put has cost 0
            if self.get_step() == (0, 0, 1):
                self.g = self.parent.g
            else:
                self.g = self.parent.g + 1
        else:
            self.g = 0

    @property
    def coordinates(self):
        return (*self.position, self.t)

    @property
    def position(self):
        return super().coordinates

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
        t = self.t
        for x, y in neighbours:
            # the cell is not a wall nor an invalid position (outside the grid)
            if (x, y) in self.a_star.walls or not TimeNode.is_valid(x, y):
                continue

            # the cell is currently available
            try:
                if AdvancedPlayer.reservation_table[(x, y, t)] != player_id:
                    continue
            except:
                pass

            # the cell will be available at next epoch
            try:
                if AdvancedPlayer.reservation_table[(x, y, t + 1)] != player_id:
                    continue
            except:
                pass

            valid_neighbours.append(
                TimeNode(self.a_star, x, y, t + 1, parent=self))
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
        return (*super().get_step(), self.t - self.parent.t)

    def __repr__(self):
        return '[' + super().__repr__() + f' at t = {self.t}]'

    def __eq__(self, other):
        """
        Tests whether the given node is equal to this node, based solely on
        their coordinates
        """
        return self.__class__ == other.__class__ \
            and self.x == other.x and self.y == other.y and self.t == other.t

    def node_equals(self, other):
        return super().__eq__(other)


class TimeAStar(AStar):  # TODO:  doc
    """
    Class implementing the A* algorithm
    """

    NB_ITERS = 0
    NB_CALLS = 0

    def __init__(self, initial_state, goal_state, initial_epoch, player_id, walls, backwards_search, last_epoch=None):
        self.initial_state = TimeNode(self, *initial_state, t=initial_epoch)
        self.goal_state = TimeNode(self, *goal_state)
        self.current_state = self.initial_state
        self.start = True
        self.player_id = player_id
        self.walls = walls
        self.last_epoch = last_epoch if last_epoch is not None else initial_epoch + \
            AdvancedPlayer.coop_period
        # the fringe
        self.open_set = Heap([self.initial_state])
        # the set of extended nodes
        self.closed_set = []
        # backwards search
        self.backwards_search = backwards_search
        if self.backwards_search is None:
            self.backwards_search = AStar(goal_state, initial_state, walls)
            self.backwards_search.run()
        AdvancedPlayer.reservation_table[self.initial_state.coordinates] = player_id
        next_epoch = (*self.initial_state.position, initial_epoch + 1)
        AdvancedPlayer.reservation_table[next_epoch] = player_id

    def true_distance(self, current_position):
        node = self.backwards_search.get_node_at(current_position)
        if node is None:
            self.backwards_search.set_new_goal(current_position)
            self.backwards_search.run()
            node = self.backwards_search.get_node_at(current_position)
        return node.g

    def run(self, resume=False):
        """
        Runs A* algorithm

        -------------------
        return:
            (list[tuple[int]]): the reversed list of steps to take to get
                to the goal state from the initial state
        -------------------
        """
        TimeAStar.NB_CALLS += 1
        while not self.open_set_is_empty():
            TimeAStar.NB_ITERS += 1
            self.current_state = self.select_best()
            neighbours = self.current_state.get_valid_neighbours(
                self.player_id)
            not_extd_neighbours = self.get_not_extended(neighbours)
            self.add_to_open_set(not_extd_neighbours)
            self.closed_set.append(self.current_state)
            if self.current_state.t == self.last_epoch:
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
        t = self.current_state.t
        current_state = self.current_state
        steps = []
        while current_state != self.initial_state:
            t -= 1
            steps.append(current_state.get_step())
            AdvancedPlayer.reservation_table[current_state.coordinates] = self.player_id
            AdvancedPlayer.reservation_table[(
                *current_state.position, t)] = self.player_id
            current_state = current_state.parent
            AdvancedPlayer.reservation_table[(
                *current_state.position, t + 1)] = self.player_id
        return steps


class AdvancedPlayer(CoopPlayer):
    """
    Class representing an advanced cooperative player that utilises
    a space-time reservation table to handle collisions, and that limits
    the path search depth
    """

    epoch = 0  # universal timer
    coop_period = None  # half the depth
    players = []  # list of all the player
    reservation_table = {}
    counter = 0  # for id's initialisation

    def __init__(self, initial_position, goal_positions, walls, goal_choice=NaiveStrategy):
        super().__init__(initial_position, goal_positions, walls, goal_choice)
        self.current_goal = self.goal_choice.get_next_goal(initial_position)
        self.id = AdvancedPlayer.counter
        self.search_epoch = None

        AdvancedPlayer.counter += 1
        AdvancedPlayer.players.append(self)

    def set_search_epoch(self, search_epoch):
        """
        Sets the search epoch, i.e. the time at which this agent will plan its
        path, and reserves its current position in the reservation table for
        all the epochs prior to the given search epoch

        -------------------
        args:
            search_epoch (int): the epoch at which this agent will plan its
            path to its current goal
        -------------------
        """
        self.search_epoch = search_epoch
        self.a_star = TimeAStar(self.current_position, self.current_goal,
                                self.search_epoch, self.id, self.walls, backwards_search=None)
        for t in range(search_epoch):
            AdvancedPlayer.reservation_table[(
                *self.initial_position, t)] = self.id

    @classmethod
    def set_search_epochs(cls):
        """
        Initialises the search epoch for each agent of this class so that
        there are approximately 2n/depth agents replanning at any epoch
        """
        n = cls.counter
        d = cls.coop_period
        players_per_epoch = n // d
        cnt = 0
        for t in range(d):
            for _ in range(players_per_epoch):
                cls.players[cnt].set_search_epoch(t)
                cnt += 1
        rem = n - cnt
        for t in range(d):
            if rem == 0:
                break
            cls.players[cnt].set_search_epoch(t)
            rem -= 1
            cnt += 1
        for player in cls.players:
            print(player.id, player.search_epoch)
        print(cls.reservation_table)

    @classmethod
    def set_cooperation_period(cls, coop_period):
        """
        Sets the cooperative period for the agents of this class
        """
        cls.coop_period = coop_period

    def clear_trace(self):
        """
        Removes any trace of its use from the reservation table
        """
        keys_to_delete = [(x, y, t) for (x, y, t), id in AdvancedPlayer.reservation_table.items()
                          if id == self.id]
        for k in keys_to_delete:
            del AdvancedPlayer.reservation_table[k]

    def find_path_to_goal(self, resume=True):
        """
        Finds a path to a goal for this agent considering its peers as potential
        obstacles

        -------------------
        args:
            [optional] resume (bool): True iff the agent must resume its pursuit
                of the current goal
        -------------------
        """
        self.clear_trace()

        if resume is True:  # for continuing the pursuit of current goal
            backwards_search = self.a_star.backwards_search
            last_epoch = self.a_star.last_epoch + AdvancedPlayer.coop_period
        else:  # for a new path
            self.current_goal = self.goal_choice.get_next_goal(
                self.current_position)
            last_epoch = self.a_star.last_epoch

            # when replanning time
            if last_epoch == AdvancedPlayer.epoch + AdvancedPlayer.coop_period:
                last_epoch += AdvancedPlayer.coop_period

            backwards_search = None

        self.a_star = TimeAStar(self.current_position, self.current_goal,
                                AdvancedPlayer.epoch, self.id, self.walls, backwards_search, last_epoch=last_epoch)
        self.steps = self.a_star.run(resume=False)

    def is_last(self):
        """
        Tests whether this agent is the last one in the list of players for
        the purpose of timer update

        -------------------
        return:
            (bool): whether this agent is the last in the list of players
        -------------------
        """
        return self.id == len(AdvancedPlayer.players) - 1

    @property
    def next(self):
        """
        Calculates this agent's next position considering its current
        and remaining goals

        -------------------
        return:
            (tuple[int]): the next position of the agent
        -------------------
        """
        # the agent succeded and wishes to meet another goal
        if self.has_next_goal():
            self.find_path_to_goal(resume=False)
            if AdvancedPlayer.epoch == self.search_epoch:
                self.search_epoch += AdvancedPlayer.coop_period

        # replanning time
        elif AdvancedPlayer.epoch == self.search_epoch:
            self.find_path_to_goal(resume=True)
            self.search_epoch += AdvancedPlayer.coop_period

        # when along the path
        if self.has_next_step():
            self.get_next_position()

        # the last agent updates the timer
        if self.is_last():
            AdvancedPlayer.epoch += 1

        return self.current_position
