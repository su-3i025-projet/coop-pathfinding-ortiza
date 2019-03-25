"""
.. module:: players
   :synopsis: This file contains the cooperative player handling collisions as
    they happen.
.. moduleauthor:: Angelo Ortiz <github.com/angelo-ortiz>
"""


import random
from functools import reduce

from .strategies import NaiveStrategy
from .tools import AStar, Node


class CoopPlayer:
    """A cooperative player that handles collisions on a rolling basis.

    Parameters
    ----------
    initial_position : (int, int)
        This argument contains the initial coordinates of the agent.
    goal_positions : list of (int, int)
        This argument contains a list of all the goals of the agent.
    walls : list of (int, int)
        This argument contains the list of all the obstacles to be avoided.
    goal_choice : GoalChoiceStrategy, optional
        This argument defines the next goal choice mode.

    Attributes
    ----------
    others
    next_position
    initial_position : (int, int)
        The storage location of the initial coordinates of the player.
    current_position : (int, int)
        The coordinates of the agent's current position.
    previous_position : (int, int)
        The coordinates of the agent's previous position.
    goal_positions : list of (int, int)
        The storage location of the agent's goal list.
    current_goal : (int, int)
        The coordinates of the agent's current goal.
    walls : list of (int, int)
        The storage location of the walls position.
    a_star : AStar or None
        The A* algorithm execution leading the agent's steps.
    steps : list of (int, int)
        The list of steps the agent must take to get to its current goal.
    goal_choice : GoalChoiceStrategy
        The storage location of the agent's goal choice strategy.

    players : list of CoopPlayer
        The list of all cooperative agents in the grid.
    CUT_OFF_LIMIT : int
        The length of the path to be cut when handling collisions.

    """

    players = []
    CUT_OFF_LIMIT = 0

    def __init__(self, initial_position, goal_positions, walls, goal_choice=NaiveStrategy):
        self.initial_position = initial_position
        self.current_position = initial_position
        self.previous_position = tuple([-1 for _ in initial_position])
        self.goal_positions = goal_positions[:]
        self.current_goal = None
        self.walls = walls
        self.a_star = None
        self.steps = []
        self.goal_choice = goal_choice(self.goal_positions)
        CoopPlayer.players.append(self)

    @classmethod
    def set_cut_off_limit(cls, cut_point):
        """Sets the number of immediate steps to be recalculated when handling
        a collision.

        Parameters
        ----------
        cut_point : int
            The length of path to be replanned when a collision happens.

        """
        cls.CUT_OFF_LIMIT = cut_point

    def add_goal(self, goal_position):
        """Adds a new goal to this agent.

        Parameters
        ----------
        goal_position : (int, int)
            The coordinates of a new goal for the agent.

        """
        self.goal_positions.append(goal_position)

    @property
    def others(self):
        """The list of all the cooperative peers of this agent.

        Returns
        -------
        (list of CoopPlayer, list of CoopPlayer)
            The list of the cooperative peers already placed during the current
            iteration, and the list of those yet to be placed.

        """
        my_index = CoopPlayer.players.index(self)
        return CoopPlayer.players[:my_index], CoopPlayer.players[my_index + 1:]

    def has_next_step(self):
        """Tests whether this agent has a planned step to take.

        Returns
        -------
        bool
            True iff the list of planned steps is not empty.

        """
        return self.steps != []

    def has_next_goal(self):
        """Tests whether this agent has another goal to pursue.

        Returns
        -------
        bool
            True iff the list of goals is not empty.

        """
        return self.goal_positions != []

    def is_at_goal(self):
        """Tests whether this agent has reached its goal.

        Returns
        -------
        bool
            True iff the agent is currently at its goal position.

        """
        return self.current_position == self.current_goal

    def find_path_to_goal(self, placed=[], resume=False):
        """Finds a path to a goal considering players already fixed on the grid.

        Parameters
        ----------
        placed : list of CoopPlayer, optional
            The list of players already placed that need to be avoided.
        resume : bool, optional
            True iff the agent must resume its pursuit of the current goal.

        """
        if resume is False:  # for a new path
            self.current_goal = self.goal_choice.get_next_goal(
                self.current_position)

        placed = [pos for pos in placed if pos != self.current_goal]

        self.a_star = AStar(self.current_position,
                            self.current_goal, self.walls + placed)
        self.steps = self.a_star.run()

    def go_through_one_another(self, other, placed):
        """Tests whether this agent will have crossed the given one after the
        current iteration.

        Parameters
        ----------
        other : CoopPlayer
            The peer agent to check.
        placed : bool
            True iff the agent has already been placed on the grid during the
            current iteration.

        Returns
        -------
        bool
            True iff the agent's next position is the other's current position,
            and the other way round.

        """
        if placed is True:
            return self.current_position == other.current_position and \
                self.next_position == other.previous_position
        return self.current_position == other.next_position and \
            self.next_position == other.current_position

    def collision(self, placed, following):
        """Checks that this agent does not collide with any other.

        Parameters
        ----------
        placed : list of CoopPlayer
            The list of players already placed during the current iteration.
        following : list of CoopPlayer
            The list of players to be placed after this agent.

        Returns
        -------
        (int, int) or None
            The coordinates a potential collision if any, otherwise None.

        """
        for oth in placed:
            if self.next_position == oth.current_position or \
                    self.go_through_one_another(oth, placed=True):
                return self.next_position
        for oth in following:
            if self.next_position == oth.next_position or \
                    self.go_through_one_another(oth, placed=False):
                return self.next_position
        return None

    def __get_valid_shifts(self, obstacles):
        """Finds all the possible shifts the agent may take.

        A shift is said to be valid if it does not lead the agent into a wall,
        a cell outside the grid or into one of the given obstacles.

        Parameters
        ----------
        obstacles : list of (int, int)
            The coordinates of some obstacles to be avoided.

        Returns
        -------
        list of (int, int)
            The list of all the shifts the agent may take during the current
            iteration.

        """
        def is_valid(shift):
            pos = (self.current_position[0] + shift[0],
                   self.current_position[1] + shift[1])
            if pos in self.walls:
                return False
            if not Node.is_valid(*pos):
                return False
            if pos in obstacles:
                return False
            return True

        shifts = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        return [sh for sh in shifts if is_valid(sh)]

    def handle_collision(self, obstacle):
        """Replans a part of this agent's immediate path considering the given
        obstacle.

        Parameters
        ----------
        obstacle : (int, int)
            The coordinates of a cell to be avoided when replanning.

        """

        def are_adjacent(pos1, pos2):
            diff = (pos1[0] - pos2[0], pos1[1] - pos2[1])
            return diff in [(0, 1), (0, -1), (1, 0), (-1, 0), (0, 0)]

        cut_path = self.steps[-CoopPlayer.CUT_OFF_LIMIT:]
        placed, _ = self.others

        # the current and previous positions of already placed agents are
        # unavailable in order to avoid collisions.
        obstacles = [oth.current_position for oth in placed if are_adjacent(
            self.current_position, oth.current_position)]
        obstacles += [oth.previous_position for oth in placed if are_adjacent(
            self.current_position, oth.previous_position)]
        obstacles += [obstacle]

        # if the remaining path is too short, it just takes a valid random step
        if len(cut_path) < CoopPlayer.CUT_OFF_LIMIT:
            valid_steps = self.__get_valid_shifts(obstacles)
            self.steps = [random.choice(valid_steps)]
        else:
            temp_goal = self.get_position_after(cut_path)
            if temp_goal == self.current_goal:
                temp_goal = self.get_position_after(
                    self.steps[-CoopPlayer.CUT_OFF_LIMIT + 1:])
            nearby_path = AStar(self.current_position, temp_goal,
                                self.walls + obstacles).run()
            self.steps = self.steps[:-CoopPlayer.CUT_OFF_LIMIT] + nearby_path

    def get_position_after(self, reversed_steps):
        """Determines this agent's position after following the given step
        sequence.

        Parameters
        ----------
        reversed_steps : list of (int, int)
            The list of steps to take in a reverse manner.

        Returns
        -------
        (int, int)
            This agent's position after taking the given step sequence.

        """
        def take(p, s):
            """
            Takes the step `s` from position `p`
            """
            return tuple([p_i + s_i for p_i, s_i in zip(p, s)])

        return reduce(take, reversed_steps[::-1], self.current_position)

    @property
    def next_position(self):
        """Determines this agent's position after taking its next step.

        Returns
        -------
        (int, int)
            The next position of this agent.

        Notes
        -----
        This method does not pop out the next step. If you wish to do it,
        however, use :meth:`~coop.players.CoopPlayer.get_next_position`.

        """
        return self.get_position_after(self.steps[-1:])

    def get_next_position(self):
        """Pops the next step out of the list and takes it to calculate its
        next position.

        Returns
        -------
        (int, int)
            The next position of the agent.

        """
        self.previous_position = self.current_position
        self.current_position = self.next_position
        self.steps.pop()
        return self.current_position

    def next(self):
        """Determines this agent's next position in the grid.

        This method considers the agent's current goal and any potential
        collisions for this purpose.

        Returns
        -------
        (int, int)
            The next position of the agent.

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
            return self.next()

        # the agent succeded and wishes to meet another goal
        elif self.has_next_goal():
            self.find_path_to_goal()
            return self.next()

        return self.current_position
