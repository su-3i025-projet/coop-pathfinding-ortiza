#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 14:20:19 2019

@author: angelo
"""

import heapq
import random
from functools import reduce


def distance(point, other):
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
        # cost of the path from the root to this node
        self.g = parent.g + 1 if parent is not None else 0

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
        return distance((self.x, self.y), (other.x, other.y))

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
                if (x, y) not in self.a_star.walls and x >= 0 and x < Node.NB_ROWS
                and y >= 0 and y < Node.NB_COLUMNS]

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


class A_star:
    """
    Class implementing the A* algorithm
    """

    def __init__(self, initial_state, goal_state, walls):
        self.initial_state = Node(self, *initial_state)
        self.goal_state = Node(self, *goal_state)
        self.walls = walls
        # the fringe
        self.open_set = [self.initial_state]
        # the set of extended nodes
        self.closed_set = []
        self.ending_condition = None  # TODO:

    def open_set_is_empty(self):
        """
        Tests whether the fringe is empty

        -------------------
        return:
            (bool): True iff the fringe is empty
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


class Goal_Choice_Strategy:

    def __init__(self, goal_positions):
        self.goal_positions = goal_positions

    def get_next_goal(self, current_position):
        raise NotImplementedError


class Naive_Strategy(Goal_Choice_Strategy):

    def __init__(self, goal_positions):
        Goal_Choice_Strategy.__init__(self, goal_positions)

    def get_next_goal(self, current_position):
        return self.goal_positions.pop()


class Closest_Strategy(Goal_Choice_Strategy):

    def __init__(self, goal_positions):
        Goal_Choice_Strategy.__init__(self, goal_positions)

    def get_next_goal(self, current_position):
        dists = [distance(current_position, goal_pos)
                 for goal_pos in self.goal_positions]
        index_of_best = dists.index(min(dists))
        return self.goal_positions.pop(index_of_best)


class Coop_Player:
    """
    Class representing a cooperative player that handles
    collisions on a rolling basis
    """

    PLAYERS = []
    M = None

    def __init__(self, initial_position, goal_positions, walls, goal_choice=Naive_Strategy):
        self.initial_position = initial_position
        self.current_position = initial_position
        self.goal_positions = goal_positions[:]
        self.current_goal = None
        self.walls = walls
        self.a_star = None
        self.steps = []
        self.goal_choice = goal_choice(self.goal_positions)
        Coop_Player.PLAYERS.append(self)

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
            (tuple[list[Coop_Player]]): the list of the cooperative peers already
                placed during this iteration, and the list of those yet to be placed
        -------------------
        """
        my_index = Coop_Player.PLAYERS.index(self)
        return Coop_Player.PLAYERS[:my_index], Coop_Player.PLAYERS[my_index + 1:]

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
        Tests whether this agent has a goal to look for

        -------------------
        return:
            (bool): True iff the list of goals is not empty
        -------------------
        """
        return self.goal_positions != []

    def is_at_goal(self):
        return self.current_position == self.current_goal

    def find_path_to_goal(self, current=False):
        if current is False:  # for a brand-new path
            self.current_goal = self.goal_choice.get_next_goal(
                self.current_position)
        self.a_star = A_star(self.current_position,
                             self.current_goal, self.walls)
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
        return self.current_position == other.next_position and self.next_position == other.current_position

    def collision(self, placed, simultaneous):
        """
        Checks that there would be no collision had the agents continue
        as usual, and returns a potential collision

        -------------------
        args:
            placed (list[Coop_Player]): the list of players already placed likely
                to collide with this agent

            simultaneous (list[Coop_Player]): the list of players to be placed
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

    def get_valid_shifts(self):
        """
        Finds all the valid neighbours of this node, i.e. those nodes
        wrapping non-wall and inside-the-grid cells

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
            if pos[0] < 0 or pos[0] >= Node.NB_ROWS:
                return False
            if pos[1] < 0 or pos[1] >= Node.NB_COLUMNS:
                return False
            placed, _ = self.others
            if pos in [oth.current_position for oth in placed]:
                return False
            return True

        shifts = [(1, 0), (-1, 0), (0, 1), (0, -1), (0, 0)]
        return [sh for sh in shifts if is_valid(sh)]

    def handle_collision(self, obstacle):
        """
        Recalculates a part of its immediate path considering the given obstacle

        -------------------
        args:
            obstacle (tuple[int]): the coordinates of an obstacle to be considered
                when recalculating its path
        -------------------
        """
        cutoff_steps = self.steps[-Coop_Player.M:]
        if len(cutoff_steps) < Coop_Player.M:
            valid_steps = self.get_valid_shifts()
            self.steps = [random.choice(valid_steps)]
        else:
            placed, _ = self.others
            nearby_path = A_star(self.current_position, self.get_position_after(cutoff_steps),
                                 self.walls + [obstacle] + placed).run()
            self.steps = self.steps[:-Coop_Player.M] + nearby_path

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
            return p[0] + s[0], p[1] + s[1]

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

        # TODO: quid de l'obstacle qui bloque la fiole

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
        if self.has_next_step():
            obstacle = self.collision(*self.others)
            if obstacle is not None:
                self.handle_collision(obstacle)
            return self.get_next_position()
        elif self.current_goal is not None and not self.is_at_goal():
            self.find_path_to_goal(current=True)
            return self.next
            # if self.has_next_step():
            #     return self.get_next_position()
        elif self.has_next_goal():
            self.find_path_to_goal()
            # if self.has_next_step():
            #     return self.get_next_position()
            return self.next
        # else:
        return self.current_position

    @classmethod
    def set_M(cls, M):
        """
        Sets the number of immediate steps to be recalculated when
        handling a collision
        """
        cls.M = M


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
        print("ttttttttttttttttttttttttttttttttttttttttttttt")
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
            if player1.collision([player2]) is not None:
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
        if not self.players[self.current_player].has_next_step() and not self.players[self.current_player].has_next_goal():
            while True:
                pass
        # print("position:", self.players[self.current_player].current_position,
        #       "steps:", self.players[self.current_player].steps)
        if not self.players[self.current_player].has_next_step():
            self.players[self.current_player].find_path_to_goal()
            self.add_to_sequence(self.current_player)
        if self.current_group == []:
            self.sequence.pop(0)
            self.current_group = self.sequence[0]
        if self.current_player in self.current_group:
            next_position = self.players[self.current_player].next
            if self.players[self.current_player].is_at_goal():
                self.current_group.remove(self.current_player)
            return next_position
        return self.players[self.current_player].current_position
