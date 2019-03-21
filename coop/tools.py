#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 14:20:19 2019

@author: angelo
"""

import heapq


def distance(point, other):
    """
    Calculates the Manhattan distance between the given points

    -------------------
    args:
        point (tuple[int]): the first point

        other (tuple[int]): the second point
    -------------------
    return:
        (int): the Manhattan distance between the two points
    -------------------
    """
    dists = [abs(c1 - c2) for c1, c2 in zip(point, other)]
    return sum(dists)


class Node:
    """
    Class representing a node in A* algorithm's state graph
    """

    NB_ROWS = None
    NB_COLUMNS = None

    def __init__(self, a_star, x, y, parent=None):
        self.a_star = a_star
        self.parent = parent
        # row index
        self.x = x
        # column index
        self.y = y
        # initialise cost of the path from the root to this node
        self.init_cost()

    def init_cost(self):
        if self.has_parent():
            # staying put has cost 0
            if self.get_step() == (0, 0):
                self.g = self.parent.g
            else:
                self.g = self.parent.g + 1
        else:
            self.g = 0

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
            (bool): True iff this node has a parent node assigned to it
        -------------------
        """
        return self.parent is not None

    @property
    def coordinates(self):
        return (self.x, self.y)

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
        return distance(self.coordinates, other.coordinates)

    @property
    def h(self):
        """
        Calculates the Manhattan heuristic value for this node, i.e.
        an estimate of the cheapest path from this node to the goal

        -------------------
        return:
            (int): the heuristic value for this node
        -------------------
        """
        return self.distance(self.a_star.goal_state)

    @property
    def f(self):
        """
        Calculates f(n) = g(n) + h(n) for this node

        -------------------
        return:
            (int): the f value for this node
        -------------------
        """
        return self.g + self.h

    @classmethod
    def is_valid(cls, x, y):
        return x >= 0 and x < Node.NB_ROWS and y >= 0 and y < Node.NB_COLUMNS

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
                if (x, y) not in self.a_star.walls and Node.is_valid(x, y)]

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

    @classmethod
    def set_world_dimensions(cls, nb_rows, nb_columns):
        """
        Sets the dimensions of the "world", i.e. the grid
        """
        cls.NB_ROWS = nb_rows
        cls.NB_COLUMNS = nb_columns

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
        function f(n) = g(n) + f(n), for the purpose of taking
        the optimal node from the current fringe
        """
        return self.f < other.f


class Heap:

    def __init__(self, elements=[]):
        self.__set = []
        self.__size = 0
        for el in elements:
            self.push(el)

    @property
    def size(self):
        return self.__size

    def __content(self, n):
        return self.__set[n]

    def __left_child(self, n):
        return 2 * n + 1

    def __right_child(self, n):
        return 2 * n + 2

    def __is_leaf(self, n):
        return self.__left_child(n) > self.__last()

    def __min_child(self, n):
        min_child = self.__left_child(n)
        if self.__right_child(n) <= self.__last() and \
                self.__content(self.__right_child(n)) < self.__content(min_child):
            min_child = self.__right_child(n)
        return min_child

    def __parent(self, n):
        return (n - 1) // 2

    def __root(self):
        return 0

    def __last(self):
        return self.size - 1

    def __is_root(self, n):
        return n == self.__root()

    def __swap(self, n, m):
        self.__set[n], self.__set[m] = self.__set[m], self.__set[n]

    def __update_bottom_up(self):
        node = self.__last()
        while not self.__is_root(node):
            parent = self.__parent(node)
            if self.__content(node) < self.__content(parent):
                self.__swap(node, parent)
                node = parent
            else:
                break

    def __update_top_down(self):
        node = self.__root()
        while not self.__is_leaf(node):
            min_child = self.__min_child(node)
            if self.__content(node) < self.__content(min_child):
                break
            else:
                self.__swap(node, min_child)
                node = min_child

    def is_empty(self):
        return self.size == 0

    def push(self, el):
        """
        uniqueness
        """
        try:
            index = self.__set.index(el)
            if el < self.__content(index):
                self.__set[index] = el
        except:
            self.__size += 1
            self.__set.append(el)
            self.__update_bottom_up()

    def pop(self):
        self.__swap(self.__root(), self.__last())
        self.__size -= 1
        self.__update_top_down()
        return self.__set.pop()

    def clear(self):
        self.__set = []
        self.__size = 0


class AStar:
    """
    Class implementing the A* algorithm
    """

    def __init__(self, initial_state, goal_state, walls):
        self.initial_state = Node(self, *initial_state)
        self.goal_state = Node(self, *goal_state)
        self.walls = walls
        # the fringe
        self.open_set = Heap([self.initial_state])
        # the set of extended nodes
        self.closed_set = []
        self.ending_condition = None  # TODO: backwards search

    def set_new_goal(self, new_goal):
        """
        Sets the new goal for the A* algorithm

        -------------------
        args:
            new_goal (tuple[int]): the coordinates of a new goal to meet
        -------------------
        """
        self.goal_state = Node(self, *new_goal)

    def open_set_is_empty(self):
        """
        Tests whether the fringe is empty

        -------------------
        return:
        (bool): True iff the fringe is empty
        -------------------
        """
        return self.open_set.is_empty()  # self.open_set == []

    def select_best(self):
        """
        Selects the best node in the fringe on the basis of its f-value

        -------------------
        return:
            (Node): the node of the fringe with the lowest f-value
        -------------------
        """
        # return heapq.heappop(self.open_set)
        return self.open_set.pop()

    def get_node_at(self, coordinates):
        for node in self.closed_set:
            if coordinates == node.coordinates:
                return node
        return None

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
            # heapq.heappush(self.open_set, st)
            self.open_set.push(st)

    def get_not_extended(self, states):
        """
        Filters out already extended nodes from a list of nodes

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

        -------------------
        return:
            (list[tuple[int]]): the reversed list of steps to take to get
                to the goal state from the initial state
        -------------------
        """
        while not self.open_set_is_empty():
            current_state = self.select_best()
            neighbours = current_state.get_valid_neighbours()
            not_extd_neighbours = self.get_not_extended(neighbours)
            self.add_to_open_set(not_extd_neighbours)
            self.closed_set.append(current_state)
            if current_state == self.goal_state:
                self.goal_state.set_parent(current_state.parent)
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
        current_state = self.goal_state
        steps = []
        while current_state != self.initial_state:
            steps.append(current_state.get_step())
            current_state = current_state.parent
        return steps
