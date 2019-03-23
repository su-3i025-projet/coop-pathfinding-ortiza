"""
.. module:: planner
   :synopsis: This file contains the planner of cooperative players' turns.
.. moduleauthor:: Angelo Ortiz <github.com/angelo-ortiz>
"""


from .players import CoopPlayer
from .strategies import GroupLengthStrategy


class CoopPlanner:
    """A cooperative entity that defines the action turns of its players.

    This planner plans the path of all of its players and groups them by
    path compatibility, i.e. their paths do not cross each other. It then
    defines the order of passing for the aforementioned groups of players.

    Parameters
    ----------
    initial_positions : list of (int, int)
        This argument contains the initial coordinates of the agents.
    goal_positions : list of list of (int, int)
        This argument contains a list of goals per agent.
    walls : list of (int, int)
        This argument contains the list of all the obstacles to be avoided.
    seq_sorting_choice : SequenceSortingStrategy
        This argument defines the grouping mode.

    Attributes
    ----------
    players : list of CoopPlayer
        The list of cooperative players on the grid.
    walls : list of (int, int)
        The storage location of the walls position.
    seq_sorting_choice : SequenceSortingStrategy
        The storage location of the grouping strategy.
    sequence : list of int
        The sequence in which the groups will pass.
    current_player : int
        The index of the current player in the players list.
    collision_baseline_positions : list of (int, int)
        The list of next positions for the player to be (re)added to a group.

    """

    def __init__(self, initial_positions, goal_positions, walls, seq_sorting_choice=GroupLengthStrategy):
        for pos, goal in zip(initial_positions, goal_positions):
            print(pos, goal)
        self.players = [CoopPlayer(init_pos, goal_pos, walls)
                        for init_pos, goal_pos in zip(initial_positions, goal_positions)]
        self.walls = walls
        self.seq_sorting_choice = seq_sorting_choice
        self.sequence = []
        self.current_player = -1
        self.collision_baseline_positions = []
        self.initialise_sequence()

    def initialise_sequence(self):
        self.find_initial_paths()
        self.update_sequence([i for i, _ in enumerate(self.players)])
        self.seq_sorting_choice = self.seq_sorting_choice(
            self.players, self.sequence)
        # print(self.sequence)
        self.seq_sorting_choice.sort()
        # print(self.sequence)
        self.current_group = self.sequence[0]

    def add_goal(self, player, goal_pos):
        """Adds a new goal to the given player.

        Parameters
        ----------
        player : int
            The index of the agent.
        goal_pos : (int, int)
            The coordinates of a new goal for the given agent.

        """
        self.players[player].add_goal(goal_pos)

    def find_initial_paths(self):
        """
        """
        for player in self.players:
            bef, aft = player.others
            others = [oth.current_goal for oth in bef] + \
                [oth.goal_positions[0] for oth in aft]
            player.find_path_to_goal(others)

    def update_paths(self):
        for player in self.players:
            bef, aft = player.others
            # others = [oth.current_goal for oth in bef + aft]
            others = [oth.current_position for oth in bef + aft]  # +=
            player.find_path_to_goal(others, resume=True)

    def exists_collision(self, player1, player2):
        # for the stationary case
        restore = False

        # save current state
        saved_current_pos2, saved_steps2 = player2.current_position, player2.steps

        if saved_steps2 == []:
            return True
        # try:
        player2.steps = player2.steps[:]
        # except Exception:
        #     return False

        # find all the cells of an already-in-the-sequence agent's path
        other_positions = [saved_current_pos2]
        while player2.has_next_step():
            other_positions.append(player2.get_next_position())

        # find all the cells of the to-be-added agent's path for the first and only time
        if self.collision_baseline_positions == []:
            # save current state
            saved_current_pos1, saved_steps1 = player1.current_position, player1.steps
            try:
                player1.steps = player1.steps[:]
            except Exception:
                player2.steps = saved_steps2
                return False
            restore = True

            self.collision_baseline_positions.append(saved_current_pos1)
            while player1.has_next_step():
                self.collision_baseline_positions.append(
                    player1.get_next_position())

        # there is a collision iff at least one cell belongs to both paths
        collision = False
        for pos1 in self.collision_baseline_positions:
            if pos1 in other_positions:
                collision = True
                break

        # restore initial state
        if restore:
            player1.current_position, player1.steps = saved_current_pos1, saved_steps1
        player2.current_position, player2.steps = saved_current_pos2, saved_steps2

        return collision

    def add_to_sequence(self, player):
        was_dealt_with = False

        # seek a place preferently at the end
        for other_indices in self.sequence[::-1]:
            must_add = True
            for other in other_indices:
                if self.exists_collision(self.players[player], self.players[other]):
                    must_add = False
                    break
            if must_add:
                other_indices.append(player)
                was_dealt_with = True
                break

        # reset path cells storage for <player>
        self.collision_baseline_positions = []

        if not was_dealt_with:
            self.sequence.append([player])

    def update_sequence(self, players):
        import random
        random.shuffle(players)
        for player in players:
            self.add_to_sequence(player)

    def next(self):
        self.current_player = (self.current_player + 1) % len(self.players)

        # calculate a new path for the agent when already met its previous one
        if not self.players[self.current_player].has_next_step():
            bef, aft = self.players[self.current_player].others
            placed = [oth.current_position for oth in bef + aft]
            placed += [oth.current_goal for oth in bef + aft]
            self.players[self.current_player].find_path_to_goal(
                placed=placed)
            print(self.players[self.current_player].steps)
            self.add_to_sequence(self.current_player)
            print(self.sequence)

        # change the current active group when empty
        if self.current_group == []:
            # self.sequence.pop(0)
            # self.current_group = self.sequence[0]
            # self.update_paths()
            self.sequence.clear()  # TODO: replan A* paths instead of just updating the turns list
            self.update_sequence(
                [i for i, p in enumerate(self.players) if p.has_next_step()])
            self.seq_sorting_choice.sort()
            self.current_group = self.sequence[0]

        print(self.current_group)

        # the current agent will move if being part of the current active group
        if self.current_player in self.current_group:
            try:
                next_position = self.players[self.current_player].get_next_position(
                )
            except Exception:
                while True:
                    pass
            if self.players[self.current_player].is_at_goal():
                self.current_group.remove(self.current_player)
            return next_position

        return self.players[self.current_player].current_position
