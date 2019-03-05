#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 14:20:19 2019

@author: angelo
"""

import heapq
from functools import reduce


def set_world_dimensions(nb_rows, nb_columns):
    Node.NB_ROWS = nb_rows
    Node.NB_COLUMNS = nb_columns


class Node:
    """
    Class representing a node in A* algorithm's state graph
    """

    NB_ROWS = None
    NB_COLUMNS = None

    def __init__(self, a_star, x, y, parent=None):
        self.a_star = a_star
        self.x = x  # row index
        self.y = y  # column index
        self.parent = parent
        self.dist_to_root = parent.dist_to_root + 1 if parent is not None else 0

    def set_parent(self, parent):
        """
        Changes the parent of this node

        -------------------
        args:
            parent (Node): the new parent
        -------------------
        """
        self.parent = parent

    def has_parent(self):
        """
        Tests whether this node has a parent

        -------------------
        return:
            (bool): whether this node has a parent node assigned to it
        -------------------
        """
        return self.parent is not None

    def distance(self, other):
        """
        Calculates the Manhattan distance between this node and the given one

        -------------------
        args:
            other (Node): the node for which the distance will be calculated
        -------------------
        return:
            (int): the Manhattan distance between the two nodes
        -------------------
        """
        return abs(self.x - other.x) + abs(self.y - other.y)

    @property
    def heuristic(self):
        """
        Calculates the Manhattan heuristic value for this node

        -------------------
        return:
            (int): the heuristic value for this node
        -------------------
        """
        return self.distance(self.a_star.goal_state)

    def get_valid_neighbours(self):
        """
        Finds all the valid neighbours of this node, i.e. those nodes
        wrapping non-wall and inside-the-grid cells

        -------------------
        return:
            (list[Node]): the list of all this node's valid neighbours
        -------------------
        """
        shifts = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        neighbours = [(self.x + dx, self.y + dy) for dx, dy in shifts]
        return [Node(self.a_star, x, y, parent=self) for x, y in neighbours
                if (x, y) not in self.a_star.walls and x >= 0 and x < Node.NB_ROWS and
                y >= 0 and y < Node.NB_COLUMNS]

    def get_step(self):
        """
        Calculates the step to take to come to this node from its parent

        -------------------
        return:
            (tuple[int]): the step the agent must take to come to this node
            from its parent
        -------------------
        """
        return self.x - self.parent.x, self.y - self.parent.y

    def __repr__(self):
        return f'({self.x}, {self.y})'

    def __eq__(self, other):
        """
        Tests whether the given node is equal to this node, based solely on
        their coordinates
        """
        return self.__class__ == other.__class__ and \
            self.x == other.x and self.y == other.y

    def __lt__(self, other):
        """
        Tests whether this node is less than the given node, based on A*'s
        function f(n) = heur(n) + dist(root, n), for the purpose of taking
        the optimal node from the current fringe
        """
        return (self.dist_to_root + self.heuristic) < (other.dist_to_root + other.heuristic)


class A_star:
    """
    Class implementing the A* algorithm
    """

    def __init__(self, initial_pos, goal_pos, walls):
        self.initial_state = Node(self, *initial_pos)
        self.goal_state = Node(self, *goal_pos)
        self.walls = walls
        self.open_set = [self.initial_state]  # the fringe
        self.closed_set = []  # the set of extended nodes
        self.ending_condition = None  # TODO:

    def open_set_is_empty(self):
        """
        Tests whether the fringe is empty

        -------------------
        return:
            (bool): whether the fringe is empty
        -------------------
        """
        return self.open_set == []  # or self.goal_state.has_parent()

    def select_best(self):
        """
        Selects the best node in the fringe on the basis of its f-value

        -------------------
        return:
            (Node): the node of the fringe with the lowest f-value
        -------------------
        """
        return heapq.heappop(self.open_set)

    def add_to_open_set(self, states):
        """
        Adds the given nodes to the fringe

        -------------------
        args:
            states (list[Node]): a list of state-wrapping nodes to be added to
            the fringe
        -------------------
        """
        for st in states:
            heapq.heappush(self.open_set, st)

    def get_not_extended(self, states):
        """
        Filters out already extended nodes

        -------------------
        args:
            states (list[Node]): a list of state-wrapping nodes
        -------------------
        return:
            (list[Node]): the list obtained as a result of removing already
            extended nodes from the given list
        -------------------
        """
        return [st for st in states if st not in self.closed_set]

    def run(self):
        """
        Runs A* algorithm
        """
        while not self.open_set_is_empty():
            current_state = self.select_best()
            if current_state == self.goal_state:
                self.goal_state.set_parent(current_state.parent)
                return self.get_reversed_step_sequence()
            neighbours = current_state.get_valid_neighbours()
            not_extd_neighbours = self.get_not_extended(neighbours)
            self.add_to_open_set(not_extd_neighbours)
            self.closed_set.append(current_state)

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
        current_state = self.goal_state
        steps = []
        while current_state != self.initial_state:
            steps.append(current_state.get_step())
            current_state = current_state.parent
        return steps


class Coop_Player:

    PLAYERS = []
    DEFAULT = (0, 0)
    M = None

    def __init__(self, initial_pos, goal_pos, walls):
        self.initial_position = initial_pos
        self.current_position = initial_pos
        self.goal_positions = goal_pos
        self.walls = walls
        self.a_star = None
        self.steps = []
        Coop_Player.PLAYERS.append(self)

    @property
    def others(self):
        return [p for p in Coop_Player.PLAYERS if p is not self]

    def has_next_step(self):
        return self.steps != []

    def has_next_goal(self):
        return self.goal_positions != []

    def collision(self):
        oths = self.others
        # TODO: find collisions
        return None

    def handle_collision(self, obstacle):
        nearby_path = A_star(self.current_position, self.get_next_position(self.steps[Coop_Player.M - 1]),
                             self.walls + [obstacle]).run()
        self.steps = nearby_path + self.steps[Coop_Player.M:]

    def get_next_position(self, steps):
        def take(p, s):
            return p[0] + s[0], p[1] + s[1]

        return reduce(take, steps, self.current_position)

    def next_position(self, step):
        self.current_position = self.get_next_position([step])
        return self.current_position

    @property  # TODO: sprites pass through walls wtf ?!?!?!
    def next(self):
        if self.has_next_step():
            obstacle = self.collision()
            if obstacle is not None:
                self.handle_collision(obstacle)
            return self.next_position(self.steps.pop(0))
        elif self.has_next_goal():
            self.a_star = A_star(self.current_position,
                                 self.goal_positions.pop(0), self.walls)
            self.steps = self.a_star.run()
            return self.next
        else:
            return Coop_Player.DEFAULT
