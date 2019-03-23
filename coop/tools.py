"""
.. module:: tools
   :synopsis: All the basic functions for cooperative players are defined here
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
    parent : Node, optional
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
    parent : Node
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

    def __init_cost(self):  # TODO: doc
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
            the row number of this node.
        int
            the column number of this node.

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
        """Textual representation of this node"""
        return f'({self.x}, {self.y})'

    def __eq__(self, other):
        """Equality test based on the nodes' coordinates"""
        return self.__class__ == other.__class__ and \
            self.x == other.x and self.y == other.y

    def __lt__(self, other):
        """Greatness test based on the nodes' f-value"""
        return self.f() < other.f()


class Heap:
    """A priority queue assuring unicity.

    Parameters
    ----------
    elements : list of comparable object, optional
        This argument is used to initialise the heap.

    Attributes
    ----------
    size

    Notes
    -----
    This priority queue implementation uses a list as the container of the
    elements, which have to implement at least one greatness comparison function.

    """

    def __init__(self, elements=[]):
        self.__set = []
        self.__size = 0
        for el in elements:
            self.push(el)

    @property
    def size(self):
        """The size of the heap"""
        return self.__size

    def __content(self, index):
        """The content at the given index in the list.

        Parameters
        ----------
        index : int
            The index of the wanted element.

        Returns
        -------
        comparable object
            The element at `index` in the list.

        """
        return self.__set[index]

    def __left_child(self, node):
        """The index of the left child of the given node.

        Parameters
        ----------
        node : int
            The index of the parent node.

        Returns
        -------
        int
            The index of the left child node.

        """
        return 2 * node + 1

    def __right_child(self, node):
        """The index of the right child of the given node.

        Parameters
        ----------
        node : int
            The index of the parent node.

        Returns
        -------
        int
            The index of the right child node.

        """
        return 2 * node + 2

    def __is_leaf(self, node):
        """Tests whether the given node is a leaf.

        Parameters
        ----------
        node : int
            The node to verify.

        Returns
        -------
        bool
            True iff the node does not have any children.

        """
        return self.__left_child(node) > self.__last()

    def __max_child(self, node):
        """The index of the given node's child with the highest priority.

        Parameters
        ----------
        node : int
            The index of the parent node.

        Returns
        -------
        int
            The index of child node with the highest priority.

        """
        max_child = self.__left_child(node)
        if self.__right_child(node) <= self.__last() and \
                self.__content(self.__right_child(node)) < self.__content(max_child):
            max_child = self.__right_child(node)
        return max_child

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
            max_child = self.__max_child(node)
            if self.__content(node) < self.__content(max_child):
                break
            else:
                self.__swap(node, max_child)
                node = max_child

    def is_empty(self):
        """Tests emptyness
        """
        return self.size == 0

    def push(self, el):
        """
        unicity
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
        """Removes the element with the highest priority.
        """
        self.__swap(self.__root(), self.__last())
        self.__size -= 1
        self.__update_top_down()
        return self.__set.pop()


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
        # the fringe
        self.open_set = Heap([self.initial_state])
        # the set of extended nodes
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
        return self.open_set.is_empty()  # self.open_set == []

    def select_best(self):
        """Selects the best node in the fringe on the basis of its f-value.

        Returns
        -------
        Node
            The node of the fringe with the lowest f-value.

        """
        # return heapq.heappop(self.open_set)
        return self.open_set.pop()

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
            # heapq.heappush(self.open_set, st)
            self.open_set.push(st)

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
        """Runs A* algorithm

        Returns
        -------
        list of (int, int)
            The reversed list of steps to take to get to the goal state from
            the initial state.

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
        """Determines the steps leading to the goal from the initial state.

        Returns
        -------
        list of (int, int)
            The step sequence to take to get to the goal state from the initial
            state.

        Notes
        -----
        The returned step sequence was built as a stack so the agent must use
        `pop()` in order to obtain the immediate next step to take.

        """
        current_state = self.goal_state
        steps = []
        while current_state != self.initial_state:
            steps.append(current_state.get_step())
            current_state = current_state.parent
        return steps
