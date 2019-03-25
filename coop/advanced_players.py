"""
.. module:: advanced_players
   :synopsis: This file contains the advanced cooperative players.
.. moduleauthor:: Angelo Ortiz <github.com/angelo-ortiz>
"""


import random
from functools import reduce

from .players import CoopPlayer
from .strategies import NaiveStrategy
from .tools import AStar, Node


class TimeNode(Node):
    """A node in space-time A* algorithm's state graph.

    Parameters
    ----------
    a_star : AStar
        This argument points to the space-time A* instance that created this node.
    x : int
        This argument contains the row number of this node.
    y : int
        This argument contains the column number of this node.
    t : int or None, optional
        This keyword argument sets the epoch for this node.
    parent : Node or None, optional
        This keyword argument points to the node preceding this node in `a_star`.

    Attributes
    ----------
    position
    coordinates
    a_star : AStar
        The storage location of the associated space-time A* instance.
    x : int
        The storage location of the node's row number.
    y : int
        The storage location of the node's column number.
    t : int
        The storage location of the node's epoch.
    parent : Node or None
        The storage location of the node's parent.
    cost : int
        The cost of the path from space-time A*'s root to this node.

    """

    def __init__(self, a_star, x, y, t=None, parent=None):
        self.a_star = a_star
        self.x = x
        self.y = y
        self.parent = parent
        if t is not None:
            self.t = t
        elif self.has_parent():
            self.t = parent.t + 1
        else:
            self.t = 32767  # large number for the goal
        self.__init_cost()

    def __init_cost(self):
        # initialise cost of the path from the root to this node
        if self.has_parent():
            # staying put has cost 0
            if self.get_step() == (0, 0, 1):
                self.cost = self.parent.cost
            else:
                self.cost = self.parent.cost + 1
        else:
            self.cost = 0

    @property
    def position(self):
        """The space coordinates of the node.

        Returns
        -------
        int
            The row number of this node.
        int
            The column number of this node.

        """
        return super().coordinates

    @property
    def coordinates(self):
        """The space-time coordinates of the node.

        Returns
        -------
        int
            The row number of this node.
        int
            The column number of this node.
        int
            The epoch of this node.

        """
        return (*self.position, self.t)

    def h(self):
        """Calculates the true distance heuristic value for this node.

        It corresponds to the exact distance between this node and the goal.

        Returns
        -------
        int
            The true distance value for this node.

        """
        return self.a_star.true_distance(self.position)

    def get_valid_neighbours(self, player_id):
        """Finds all the valid neighbours of this node.

        A future node is said to be valid if it does not enclose a wall, is
        located inside the grid, and its space coordinates have not been
        reserved by another agent this node's epoch as well as the next one.

        Returns
        -------
        list of Node
            The list of all this node's valid neighbours.

        """
        shifts = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        # when at goal position, no cost
        if self.position == self.a_star.goal_state.position:
            shifts.append((0, 0))
        neighbours = [(self.x + dx, self.y + dy) for dx, dy in shifts]
        valid_neighbours = []
        t = self.t
        for x, y in neighbours:
            # the cell is not a wall nor an invalid position (outside the grid)
            if (x, y) in self.a_star.walls or not TimeNode.is_valid(x, y):
                continue

            # the cell is currently available or there is not a collision if taken
            try:
                other_id = AdvancedPlayer.reservation_table[(x, y, t)]
                if other_id != player_id and \
                        other_id == AdvancedPlayer.reservation_table[(self.x, self.y, t + 1)]:
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
        """Determines the step taken to come to this node from its parent.

        Returns
        -------
        int
            The step taken along the column axis.
        int
            The step taken along the row axis.
        int
            The step taken in the timeline.

        """
        return (*super().get_step(), self.t - self.parent.t)

    def __repr__(self):
        """Textual representation of this node."""
        return '[' + super().__repr__() + f' at t = {self.t}]'

    def __eq__(self, other):
        """Equality test based on the nodes' coordinates."""
        return self.__class__ == other.__class__ \
            and self.x == other.x and self.y == other.y and self.t == other.t


class TimeAStar(AStar):
    """An execution of the space-time version of the A* algorithm.

    Parameters
    ----------
    initial_state : (int, int)
        This argument contains the coordinates of the initial node.
    goal_state : (int, int)
        This argument contains the coordinates of the goal node.
    initial_epoch : int
        This argument contains the epoch at which the algorithm will start.
    player_id : int
        This argument contains the id of the agent that run this A* instance.
    walls : list of (int, int)
        This argument contains the list of all the obstacles to be avoided.
    backwards_search : AStar or None
        This argument contains the backwards A* instance for true distances.
    last_epoch : int or None, optional
        This argument contains the epoch at which pathfinding will stop.

    Attributes
    ----------
    initial_state : TimeNode
        The initial node.
    goal_state : TimeNode
        The goal node.
    player_id : int
        The storage location for the associated agent's id.
    walls : list of (int, int)
        The storage location of the walls position.
    last_epoch : int
        The endpoint of the associated agent's pathfinding window.
    open_set : Heap
        The fringe of the algorithm.
    closed_set : list of Node
        The list of nodes already extended during the execution.
    backwards_search : AStar
        The A* instance used to obtain the true distances to the goal.

    NB_ITERS : int
        The number of iterations in all space-time A* instances.
    NB_CALLS : int
        The number of executions of space-time A*.

    Notes
    -----
    This space-time version was based on David Silver's algorithm for cooperative
    pathfinding. In particular, it fixes the depth for pathfinding.

    """

    NB_ITERS = 0
    NB_CALLS = 0

    def __init__(self, initial_state, goal_state, initial_epoch, player_id, walls, backwards_search, last_epoch=None):
        self.initial_state = TimeNode(self, *initial_state, t=initial_epoch)
        self.goal_state = TimeNode(self, *goal_state)
        self.player_id = player_id
        self.walls = walls
        self.last_epoch = last_epoch if last_epoch is not None \
            else initial_epoch + AdvancedPlayer.frequence
        self.open_set = [self.initial_state]
        self.closed_set = []
        self.backwards_search = backwards_search if backwards_search is not None \
            else AStar(goal_state, initial_state, walls)
        self.__initial_reservation(initial_epoch)

    def __initial_reservation(self, initial_epoch):
        AdvancedPlayer.reservation_table[self.initial_state.coordinates] = self.player_id
        next_coordinates = (*self.initial_state.position, initial_epoch + 1)
        AdvancedPlayer.reservation_table[next_coordinates] = self.player_id

    def true_distance(self, position):
        """Calculates the true distance from the given position to the goal.

        Returns
        -------
        int
            The Manhattan distance from the given position to the goal.

        """
        node = self.backwards_search.get_node_at(position)
        if node is None:
            self.backwards_search.set_new_goal(position)
            self.backwards_search.run()
            node = self.backwards_search.get_node_at(position)
        return node.cost

    def run(self):
        """Runs this space-time A* instance.

        Returns
        -------
        list of (int, int)
            The reversed list of steps to take to get to the goal state from
            the initial state.

        Notes
        -----
        The returned step sequence was built as a stack so the agent must use
        `pop()` in order to obtain the immediate next step to take.

        """
        TimeAStar.NB_CALLS += 1
        while not self.open_set_is_empty():
            TimeAStar.NB_ITERS += 1
            current_state = self.select_best()
            neighbours = current_state.get_valid_neighbours(self.player_id)
            not_extd_neighbours = self.get_not_extended(neighbours)
            self.add_to_open_set(not_extd_neighbours)
            self.closed_set.append(current_state)
            if current_state.t == self.last_epoch:
                return self.__get_reversed_step_sequence(current_state)

    def __get_reversed_step_sequence(self, final_state):
        """Determines the steps leading to the given state from the initial state.

        Parameters
        ----------
        final_state : TimeNode
            The state that will be reached.

        Returns
        -------
        list of (int, int)
            The step sequence to take to get to the given state from the initial
            state.

        """
        t = final_state.t
        current_state = final_state
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
    """A cooperative player that handles collisions based on a reservation table.

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
    id : int
        The agent's id number.
    walls : list of (int, int)
        The storage location of the walls position.
    a_star : AStar or None
        The A* algorithm execution leading the agent's steps.
    steps : list of (int, int)
        The list of steps the agent must take to get to its current goal.
    goal_choice : GoalChoiceStrategy
        The storage location of the agent's goal choice strategy.
    search_epoch : int
        The epoch at which the player must replan its path.

    timer : int
        A universal timer.
    frequence : int
        The frequence of pathfinding in the stationary case.
    players : list of AdvancedPlayer
        The list of all advanced cooperative agents in the grid.
    reservation_table : dict of (int, int, int): int
        A structure used to reserve grid positions at any time.
    counter : int
        The number of players on the grid.

    Notes
    -----
    This space-time version was based on David Silver's algorithm for cooperative
    pathfinding. Particularly, it implements a reservation table for space-time
    coordinates, and interleaves agents' searches.

    """

    timer = 0
    frequence = 0
    players = []
    reservation_table = {}
    counter = 0  # for id's initialisation

    def __init__(self, initial_position, goal_positions, walls, goal_choice=NaiveStrategy):
        super().__init__(initial_position, goal_positions, walls, goal_choice)
        self.current_goal = self.goal_choice.get_next_goal(initial_position)
        self.id = AdvancedPlayer.counter
        self.search_epoch = -1

        # update the list of players
        AdvancedPlayer.counter += 1
        AdvancedPlayer.players.append(self)

    def __set_search_epoch(self, search_epoch):
        """Sets the search epoch of this agent.

        The search epoch corresponds to the time at which the agent will plan
        its path. This method also reserves its current coordinates in the
        reservation table for all the epochs prior to the given search epoch.

        Parameters
        ----------
        search_epoch : int
            The epoch at which this agent will plan its path to its current goal.

        """
        self.search_epoch = search_epoch
        self.a_star = TimeAStar(self.current_position, self.current_goal,
                                self.search_epoch, self.id, self.walls, backwards_search=None)
        for t in range(search_epoch):
            AdvancedPlayer.reservation_table[(
                *self.initial_position, t)] = self.id

    @classmethod
    def set_search_epochs(cls):
        """Initialises the search epoch for each agent.

        Notes
        -----
        This method interleaves the searches, i.e. there are approximately
        :math:`n/f` agents replanning at any epoch, where :math:`n` is the
        number of agents and :math:`f` is the pathfinding frequence.

        """
        n = cls.counter
        d = cls.frequence
        players_per_epoch = n // d
        cnt = 0
        for _ in range(players_per_epoch):
            for t in range(d):
                cls.players[cnt].__set_search_epoch(t)
                cnt += 1
        rem = n - cnt
        for t in range(d):
            if rem == 0:
                break
            cls.players[cnt].__set_search_epoch(t)
            rem -= 1
            cnt += 1
        for player in cls.players:
            print(player.id, player.search_epoch)
        print(cls.reservation_table)

    @classmethod
    def set_pathfinding_frequence(cls, frequence):
        """Sets the pathfinding frequence for the agents.

        Parameters
        ----------
        frequence : int
            The time after which an agent must replan its path.

        """
        cls.frequence = frequence

    def clear_trace(self):
        """Removes any trace of this agent's path from the reservation table."""
        keys_to_delete = [(x, y, t) for (x, y, t), id in AdvancedPlayer.reservation_table.items()
                          if id == self.id]
        for k in keys_to_delete:
            del AdvancedPlayer.reservation_table[k]

    def pathfind(self, resume=True):
        """Finds a path to one of this agent's goals.

        Parameters
        ----------
        resume : bool, optional
            True iff the agent must resume its pursuit of the current goal.

        """
        self.clear_trace()

        if resume is True:  # for continuing the pursuit of current goal
            backwards_search = self.a_star.backwards_search
            last_epoch = self.a_star.last_epoch + AdvancedPlayer.frequence
        else:  # for a new path
            self.current_goal = self.goal_choice.get_next_goal(
                self.current_position)
            last_epoch = self.a_star.last_epoch

            # when replanning time
            if last_epoch == AdvancedPlayer.timer + AdvancedPlayer.frequence:
                last_epoch += AdvancedPlayer.frequence

            backwards_search = None

        self.a_star = TimeAStar(self.current_position, self.current_goal,
                                AdvancedPlayer.timer, self.id, self.walls, backwards_search, last_epoch=last_epoch)
        self.steps = self.a_star.run()

    def is_last(self):
        """Tests whether this agent is the last one in the list of players.

        This method is used only for the purpose of timer update.

        Returns
        -------
        bool
            True iff this agent is the last one in the list of players.

        """
        return self.id == len(AdvancedPlayer.players) - 1

    def next(self):
        """Determines this agent's next position in the grid.

        Returns
        -------
        (int, int)
            The next position of the agent.

        """
        # the agent succeded and wishes to meet another goal
        if self.is_at_goal() and self.has_next_goal():
            self.pathfind(resume=False)
            if AdvancedPlayer.timer == self.search_epoch:
                self.search_epoch += AdvancedPlayer.frequence

        # replanning time
        elif self.search_epoch == AdvancedPlayer.timer:
            self.pathfind(resume=True)
            self.search_epoch += AdvancedPlayer.frequence

        # when along the path
        if self.has_next_step():
            self.get_next_position()

        # the last agent updates the timer
        if self.is_last():
            AdvancedPlayer.timer += 1

        return self.current_position
