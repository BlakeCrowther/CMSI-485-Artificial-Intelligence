import numpy
import time
import json
from queue import Queue

# Agent constants
MOVE_DIRS = {"U": (0, 1), "D": (0, -1), "L": (-1, 0), "R": (1, 0)}

class MazeAgent:
    
    def __init__ (self, env):
        self.env = env
        self.malmo = env.malmo
        self.loc = env.player_pos
        self.facing = (0, 1)
        self.maze = env.ag_maze
        self.plan = Queue()
        # [!] TODO: Initialize any other knowledge-related attributes for
        # agent here
    
    # [!] TODO! Agent currently just runs straight up
    def think(self, perception):
        """
        think is parameterized by the agent's perception of the tile type
        on which it is now standing, and is called during the environment's
        action loop. This method is the chief workhorse of your MazeAgent
        such that it must then generate a plan of action from its current
        knowledge about the environment.
        """
        # Agent simply plans to move forward at the moment
        # Do something that thinks about the perception!
        self.plan.put("U")
    
    
    # [!] TODO: Any record-keeping when bot dies
    def die(self):
        """
        die is called whenever BlindBot has stepped into a lava pit. Whoops!
        In this bot's mistakes, however, there is information to inform its
        successors. The information that the bot's current location was a pit
        is useful, after which the bot is reincarnated back at the initial state
        """
        # Reset player position back to start
        self.loc = self.env.player_pos
    
    def act(self):
        """
        Called at each of the environment's ticks, the act method gives the
        BlindBot the chance to perceive the tile it is currently standing
        upon, think about what it should do next, and then enact that plan
        one step at a time
        """
        # BlindBot perceives what it is standing on...
        perception = self.perceive()
        # ...BlindBot thinks...
        self.think(perception)
        # ...and then moves according to its plan, if it has one
        if (self.plan.empty()):
            return
        self.move(self.plan.get())
        return
    
    # -----------------------------------------------------------------------
    # YOU MAY NOT MODIFY ANYTHING BELOW THIS LINE
    # -----------------------------------------------------------------------
    
    def move(self, dir):
        """
        Instructs the Malmo agent to move one tile (discretely, shh) in the
        given dir amongst legal actions: {"U", "D", "L", "R"}
        """
        move_dir = MOVE_DIRS[dir]
        face_dir = tuple(numpy.subtract(self.facing, move_dir))
        turnVal = -0.5
        
        if (abs(face_dir[0]) == 2 or abs(face_dir[1]) == 2):
            turnVal = 1
        else:
            turnVal = turnVal * face_dir[0] * face_dir[1]
        
        self.facing = move_dir
        self.loc = self.loc + move_dir
        self.malmo.sendCommand("turn " + str(turnVal))
        self.malmo.sendCommand("move 1")
    
    def perceive(self):
        """
        Receives the currently stood-upon tile type from the environment, and
        also tests if that tile is a goal in order to end the game
        """
        # Information comes in as a TimestampedString
        msg = self.malmo.getWorldState().observations[-1].text
        observations = json.loads(msg)
        grid = observations.get(u'floor3x3', 0)
        # Since our agent is blind, it only knows what it's standing on:
        standingOn = grid[4]
        # If we're standing on a goal, huzzah! We win!
        self.env.goal_test(standingOn)
        return standingOn
    