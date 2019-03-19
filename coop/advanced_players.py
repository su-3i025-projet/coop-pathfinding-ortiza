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
        super().__init__(a_star, x, y, parent)
        self.best_child = None  # for accelerating A* resuming
        if t is not None:
            self.t = t
        elif self.has_parent():
            self.t = parent.t + 1
        else:
            self.t = 32767  # large number for the goal
        # staying put has cost 0
        if self.has_parent() and self.get_step() == (0, 0, 1):
            self.g = parent.g

    def update_best(self):
        if self.parent is not None:
            if self.parent.best_child is None:
                self.parent.best_child = self
            elif self < self.parent.best_child:
                self.parent.best_child = self

    @property
    def position(self):
        return (*super().position, self.t)

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
        return self.a_star.true_distance(self.coordinates)

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

    NB_ITER = 0
    NB_CALLS = 0

    def __init__(self, initial_state, goal_state, initial_epoch, player_id, walls, last_epoch=None):
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
        self.backwards_search = AStar(goal_state, initial_state, walls)
        self.backwards_search.run()
        AdvancedPlayer.reservation_table[self.initial_state.position] = player_id
        next_epoch = (*self.initial_state.coordinates, initial_epoch + 1)
        AdvancedPlayer.reservation_table[next_epoch] = player_id

    def true_distance(self, current_position):
        node = self.backwards_search.get_node_at(current_position)
        if node is None:
            self.backwards_search.set_new_goal(current_position)
            self.backwards_search.run()
            node = self.backwards_search.get_node_at(current_position)
        return node.g

    def set_new_goal(self, new_goal):
        """
        Sets the new goal for the A* algorithm

        -------------------
        args:
            new_goal (tuple[int]): the coordinates of a new goal to meet
        -------------------
        """
        self.goal_state = TimeNode(self, *new_goal)

    def set_initial_state(self, current_state):
        for node in self.closed_set:
            if node.position == (*current_state, AdvancedPlayer.epoch):
                self.initial_state = node
                break

    def reset(self, current_state):
        self.set_initial_state(current_state)
        self.last_epoch += AdvancedPlayer.coop_period

    def select_best_prev(self, resume):
        if resume is True:
            iter = 0
            heap = Heap(
                self.initial_state.get_valid_neighbours(self.player_id))
            while True:
                iter += 1
                self.current_state = heap.pop()
                if self.current_state not in self.closed_set:
                    print("=====================\n",
                          iter, "\n==================")
                    return self.current_state
                for nb in self.current_state.get_valid_neighbours(self.player_id):
                    heap.push(nb)
        return super().select_best()

    def select_best(self, resume):
        if resume is True:
            current = self.initial_state
            while current.best_child is not None:
                current = current.best_child
            self.closed_set.append(self.open_set.pop())
            # print(self.open_set._Heap__set)
            # if current not in self.closed_set:  # first call = singleton fringe
            #     return current
            hp = Heap(current.get_valid_neighbours(self.player_id))
            try:
                while True:
                    next = hp.pop()
                    if next not in self.closed_set:
                        self.open_set.push(next)
                        return next
            except:
                0 / 0
                pass
        return super().select_best()

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
            TimeAStar.NB_ITER += 1
            self.current_state = self.select_best(resume)
            self.current_state.update_best()
            if not resume:
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
        print("========================")
        while current_state != self.initial_state:
            t -= 1
            steps.append(current_state.get_step())
            print(current_state, 'from', current_state.parent, current_state.position, (
                *current_state.coordinates, t))
            if current_state.position in AdvancedPlayer.reservation_table.keys() and \
                    AdvancedPlayer.reservation_table[current_state.position] != self.player_id:
                print("previous:",
                      AdvancedPlayer.reservation_table[current_state.position])
                print(AdvancedPlayer.reservation_table)
                print("===========================")
            AdvancedPlayer.reservation_table[current_state.position] = self.player_id
            AdvancedPlayer.reservation_table[(
                *current_state.coordinates, t)] = self.player_id
            current_state = current_state.parent
            AdvancedPlayer.reservation_table[(
                *current_state.coordinates, t + 1)] = self.player_id
        print("========================")
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
                                self.search_epoch, self.id, self.walls)
        # r
        #
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

    @classmethod
    def set_cooperation_period(cls, coop_period):
        """
        Sets the cooperative period for the agents of this class
        """
        cls.coop_period = coop_period

    def clear_unused_following_path(self):
        """
        Removes the keys correponding to steps that will be soon replaced and
        are thus useless
        """
        keys_to_delete = [(x, y, t) for (x, y, t), id in AdvancedPlayer.reservation_table.items()
                          if id == self.id and t > AdvancedPlayer.epoch]
        for k in keys_to_delete:
            del AdvancedPlayer.reservation_table[k]

    def clear_past_path(self):
        """
        Removes the keys correponding to past epochs from the reservation table
        """
        keys_to_delete = [(x, y, t) for (x, y, t), id in AdvancedPlayer.reservation_table.items()
                          if id == self.id and t < AdvancedPlayer.epoch - 2]
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
        # remove useless path slices from the reservation table
        self.clear_unused_following_path()
        self.clear_past_path()

        if resume is False:  # for a new path
            self.current_goal = self.goal_choice.get_next_goal(
                self.current_position)
            last_epoch = self.a_star.last_epoch - AdvancedPlayer.coop_period
            self.a_star = TimeAStar(self.current_position, self.current_goal,
                                    AdvancedPlayer.epoch, self.id, self.walls, last_epoch=last_epoch)

        self.a_star.reset(self.current_position)
        self.steps = self.a_star.run(resume)

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
        # replanning time
        if self.search_epoch == AdvancedPlayer.epoch:
            self.find_path_to_goal()
            self.search_epoch += AdvancedPlayer.coop_period

        # the agent succeded and wishes to meet another goal
        if self.has_next_goal():
            self.find_path_to_goal(resume=False)

        # when along the path
        if self.has_next_step():
            self.get_next_position()

        # the last agent updates the timer
        if self.is_last():
            AdvancedPlayer.epoch += 1

        return self.current_position
