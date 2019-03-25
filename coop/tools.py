"""
.. module:: tools
   :synopsis: All the basic functions for cooperative players are defined here.
.. moduleauthor:: Angelo Ortiz <github.com/angelo-ortiz>
"""

import heapq


def distance(point, other):
    """Calculates the Manhattan distance between the given points.

    Parameters
    ----------
    point : tuple of int
        The coordinates of the first point.
    other : tuple of int
        The coordinates of the second point.

    Returns
    -------
    int
        The Manhattan distance between the two points.

    """
    dists = [abs(c1 - c2) for c1, c2 in zip(point, other)]
    return sum(dists)


class Node:
    """A node in A* algorithm's state graph.

    Parameters
    ----------
    a_star : AStar
        This argument points to the A* instance that created this node.
    x : int
        This argument contains the row number of this node.
    y : int
        This argument contains the column number of this node.
    parent : Node or None, optional
        This keyword argument points to the node preceding this node in `a_star`.

    Attributes
    ----------
    coordinates
    a_star : AStar
        The storage location of the associated A* instance.
    x : int
        The storage location of the node's row number.
    y : int
        The storage location of the node's column number.
    parent : Node or None
        The storage location of the node's parent.
    cost : int
        The cost of the path from A*'s root to this node.

    NB_ROWS : int
        The number of rows of the grid.
    NB_COLUMNS : int
        The number of columns of the grid.

    """

    NB_ROWS = 0
    NB_COLUMNS = 0

    def __init__(self, a_star, x, y, parent=None):
        self.a_star = a_star
        self.x = x
        self.y = y
        self.parent = parent
        self.__init_cost()

    @classmethod
    def set_world_dimensions(cls, nb_rows, nb_columns):
        """Sets the dimensions of the grid.

        Parameters
        ----------
        nb_rows : int
            The number of rows.
        nb_columns : int
            The number of columns.

        """
        cls.NB_ROWS = nb_rows
        cls.NB_COLUMNS = nb_columns

    def __init_cost(self):
        if self.has_parent():
            # staying put has cost 0
            if self.get_step() == (0, 0):
                self.cost = self.parent.cost
            else:
                self.cost = self.parent.cost + 1
        else:
            self.cost = 0

    def set_parent(self, parent):
        """Changes the parent of this node.

        Parameters
        ----------
        parent : Node
            The new parent.

        """
        self.parent = parent

    def has_parent(self):
        """Tests whether this node has a parent.

        Returns
        -------
        bool
            True iff this node has a parent node assigned to it.

        """
        return self.parent is not None

    @property
    def coordinates(self):
        """The coordinates of the node.

        Returns
        -------
        int
            The row number of this node.
        int
            The column number of this node.

        """
        return (self.x, self.y)

    def distance(self, other):
        """Calculates the distance between this node and the given one.

        Parameters
        ----------
        other : Node
            The node for which the distance will be calculated.

        Returns
        -------
        int
            The distance between the two nodes.

        See Also
        --------
        distance

        """
        return distance(self.coordinates, other.coordinates)

    def h(self):
        """Calculates the Manhattan heuristic value for this node.

        It corresponds to an estimate of the cheapest path from this node to
        the goal.

        Returns
        -------
        int
            The heuristic value for this node.

        """
        return self.distance(self.a_star.goal_state)

    def f(self):
        """Calculates the f-value of this node.

        The f-value of a node corresponds to an estimate of the full
        path starting from the root and ending at the goal.

        Returns
        -------
        int
            The f-value of this node.

        """
        return self.cost + self.h()

    @classmethod
    def is_valid(cls, x, y):
        """Tests whether the given position is in the grid bounds.

        Parameters
        ----------
        x : int
            The row number.
        y : int
            The column number.

        Returns
        -------
        bool
            True iff the given position is located inside the grid.

        """
        return x >= 0 and x < Node.NB_ROWS and y >= 0 and y < Node.NB_COLUMNS

    def get_valid_neighbours(self):
        """Finds all the valid neighbours of this node.

        A node is said to be valid if it does not enclose a wall and is located
        inside the grid.

        Returns
        -------
        list of Node
            The list of all this node's valid neighbours.

        """
        shifts = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        neighbours = [(self.x + dx, self.y + dy) for dx, dy in shifts]
        return [Node(self.a_star, x, y, parent=self) for x, y in neighbours
                if (x, y) not in self.a_star.walls and Node.is_valid(x, y)]

    def get_step(self):
        """Determines the step taken to come to this node from its parent.

        Returns
        -------
        int
            The step taken along the column axis.
        int
            The step taken along the row axis.

        """
        return self.x - self.parent.x, self.y - self.parent.y

    def __repr__(self):
        """Textual representation of this node."""
        return f'({self.x}, {self.y})'

    def __eq__(self, other):
        """Equality test based on the nodes' coordinates."""
        return self.__class__ == other.__class__ and \
            self.x == other.x and self.y == other.y

    def __lt__(self, other):
        """Greatness test based on the nodes' f-value."""
        return self.f() < other.f()


class AStar:
    """An execution of the A* algorithm.

    Parameters
    ----------
    initial_state : (int, int)
        This argument contains the coordinates of the initial node.
    goal_state : (int, int)
        This argument contains the coordinates of the goal node.
    walls : list of (int, int)
        This argument contains the list of all the obstacles to be avoided.

    Attributes
    ----------
    initial_state : Node
        The initial node.
    goal_state : Node
        The goal node.
    walls : list of (int, int)
        The storage location of the walls position.
    open_set : Heap
        The fringe of the algorithm.
    closed_set : list of Node
        The list of nodes already extended during the execution.

    """

    def __init__(self, initial_state, goal_state, walls):
        self.initial_state = Node(self, *initial_state)
        self.goal_state = Node(self, *goal_state)
        self.walls = walls
        self.open_set = [self.initial_state]
        self.closed_set = []

    def set_new_goal(self, new_goal):
        """Sets the new goal for the A* algorithm.

        Parameters
        ----------
        new_goal : (int, int)
            The coordinates of a new goal to meet.

        """
        self.goal_state = Node(self, *new_goal)

    def open_set_is_empty(self):
        """Tests whether the fringe is empty.

        Returns
        -------
        bool
            True iff the fringe is empty.

        """
        return self.open_set == []

    def select_best(self):
        """Selects the best node in the fringe on the basis of its f-value.

        Returns
        -------
        Node
            The node of the fringe with the lowest f-value.

        """
        return heapq.heappop(self.open_set)

    def get_node_at(self, coordinates):
        """Retrieves the node with the given coordinates in the closed set.

        Parameters
        ----------
        coordinates : (int, int)
            The coordinates of the desired node.

        Returns
        -------
        Node or None
            The node in the closed set whose coordinates are given.

        """
        for node in self.closed_set:
            if coordinates == node.coordinates:
                return node
        return None

    def add_to_open_set(self, states):
        """Appends the given nodes to the fringe.

        Parameters
        ----------
        states : list of Node
            A list of state-wrapping nodes to be added to the fringe.

        """
        for st in states:
            heapq.heappush(self.open_set, st)

    def get_not_extended(self, states):
        """Filters out already extended nodes from the given list of nodes.

        Parameters
        ----------
        states : list[Node]
            A list of state-wrapping nodes.

        Returns
        -------
        list of Node
            The list obtained as a result of removing already extended nodes
            from the given list.

        """
        return [st for st in states if st not in self.closed_set]

    def run(self):
        """Runs this A* instance.

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
        while not self.open_set_is_empty():
            current_state = self.select_best()
            neighbours = current_state.get_valid_neighbours()
            not_extd_neighbours = self.get_not_extended(neighbours)
            self.add_to_open_set(not_extd_neighbours)
            self.closed_set.append(current_state)
            if current_state == self.goal_state:
                self.goal_state.set_parent(current_state.parent)
                return self.__get_reversed_step_sequence()

    def __get_reversed_step_sequence(self):
        """Determines the steps leading to the goal from the initial state.

        Returns
        -------
        list of (int, int)
            The step sequence to take to get to the goal state from the initial
            state.

        """
        current_state = self.goal_state
        steps = []
        while current_state != self.initial_state:
            steps.append(current_state.get_step())
            current_state = current_state.parent
        return steps
