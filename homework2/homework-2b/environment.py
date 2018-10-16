import MalmoPython
import os
import sys
import time
from maze_agent import MazeAgent

# Environment Configuration Constants:
FLOOR_LEVEL  = 5
MISSION_TIME = 120000
MAX_RETRIES  = 3
TICK_LENGTH  = 1 # seconds

class Environment:
    
    # Problem-specific constants
    GOAL_BLOCK = "diamond_block"
    WARN_BLOCK = "obsidian"
    SAFE_BLOCK = "cobblestone"
    
    def __init__ (self, maze):
        """
        Initializes the environment from a given maze, specified as an
        array of strings like in previous assignments
        """
        self.maze = maze
        self.ag_maze = [r.replace('P', '.') for r in maze]
        self.mission_xml = self.get_mission_xml(maze)
        self.mission = MalmoPython.MissionSpec(self.mission_xml, True)
        self.mission_record = MalmoPython.MissionRecordSpec()
        self.malmo = MalmoPython.AgentHost()
        self.agent = MazeAgent(self)
        self.attempt = 1
        self.goal_reached = False
    
    def __translate_coord (self, c, r):
        """
        Malmo's coordinate plane is reversed from ours, so this function
        provides a translation based on this environment's maze
        """
        return (len(self.maze[0]) -1 - c, len(self.maze) - 1 - r)
    
    def __generate_pit (self, c, r):
        """
        For any given pits at position (c, r), this method will populate
        the adjacent obsidian tiles
        """
        maze = self.maze
        
        # Generate pit
        result = '<DrawBlock x="{}" y="5" z="{}" type="lava"/>'.format(c, r)
        
        # Generate obsidian warning blocks, as long as there isn't another tile already
        # there, and the tile would not be generated outside of the maze's bounds
        # (probably a better way to do this, but meh)
        obsidians = list(filter(lambda b: b[0] != 0 and b[1] != 0 and b[0] < len(maze[0]) and b[1] < len(maze) and maze[len(maze) - b[1] - 1][len(maze[0]) - b[0] - 1] == ".",\
            [(c-1, r), (c+1, r), (c, r-1), (c, r+1)]))
        for coord in obsidians:
            result = result + '<DrawBlock x="{}" y="{}" z="{}" type="{}"/>'.format(coord[0], FLOOR_LEVEL, coord[1], Environment.WARN_BLOCK)
        
        return result
    
    def __generate_maze (self):
        """
        Generates the Malmo blocks from the given maze representation
        """
        # maze grid is rectangular
        maze = self.maze
        rows = len(maze)
        cols = len(maze[0])
        
        # Base grid at 0,0
        result = '<DrawCuboid x1="0" y1="{}" z1="0" x2="{}" y2="{}" z2="{}" type="{}"/>'.format(FLOOR_LEVEL, cols-1, FLOOR_LEVEL, rows-1, Environment.SAFE_BLOCK)
        
        # Parse special cell contents
        for r, row in enumerate(maze):
            for c, cell in enumerate(row):
                tcoord = self.__translate_coord(c, r)
                if cell == "*":
                    self.player_pos = tcoord
                elif cell == "X":
                    result = result + '<DrawBlock x="{}" y="{}" z="{}" type="{}"/>'.format(tcoord[0], FLOOR_LEVEL + 1, tcoord[1], Environment.SAFE_BLOCK)
                elif cell == "G":
                    result = result + '<DrawBlock x="{}" y="{}" z="{}" type="{}"/>'.format(tcoord[0], FLOOR_LEVEL, tcoord[1], Environment.GOAL_BLOCK)
                elif cell == "P":
                    result = result + self.__generate_pit(tcoord[0], tcoord[1])
                    
        return result
    
    def get_mission_xml (self, maze):
        """
        Provides the Malmo environment XML from the given maze
        """
        mazeXML = self.__generate_maze()
        
        return '''<?xml version="1.0" encoding="UTF-8" standalone="no" ?>
            <Mission xmlns="http://ProjectMalmo.microsoft.com" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
            
                <About>
                    <Summary>Maze Pitfalls!</Summary>
                </About>
              
                <ServerSection>
                    <ServerInitialConditions>
                        <Time>
                            <StartTime>1000</StartTime>
                            <AllowPassageOfTime>false</AllowPassageOfTime>
                        </Time>
                        <Weather>clear</Weather>
                    </ServerInitialConditions>
                    <ServerHandlers>
                        <FlatWorldGenerator generatorString="3;7,2*3,11;8;village"/>
                        <DrawingDecorator>
                            <DrawCuboid x1="-50" y1="''' + str(FLOOR_LEVEL) + '''" z1="-50" x2="50" y2="''' + str(FLOOR_LEVEL) + '''" z2="50" type="air"/>
                            ''' + mazeXML + '''
                        </DrawingDecorator>
                        <ServerQuitFromTimeUp timeLimitMs="''' + str(MISSION_TIME) + '''"/>
                        <ServerQuitWhenAnyAgentFinishes/>
                    </ServerHandlers>
                </ServerSection>
                  
                <AgentSection mode="Survival">
                    <Name>BlindBot</Name>
                    <AgentStart>
                        <Placement x="''' + str(self.player_pos[0] + 0.5) + '''" y="''' + str(FLOOR_LEVEL + 1.0) + '''" z="''' + str(self.player_pos[1] + 0.5) + '''"/>
                    </AgentStart>
                    <AgentHandlers>
                        <DiscreteMovementCommands/>
                        <AgentQuitFromTouchingBlockType>
                            <Block type="lava"/>
                        </AgentQuitFromTouchingBlockType>
                        <ObservationFromGrid>
                            <Grid name="floor3x3">
                                <min x="-1" y="-1" z="-1"/>
                                <max x="1" y="-1" z="1"/>
                            </Grid>
                        </ObservationFromGrid>
                    </AgentHandlers>
                </AgentSection>
            </Mission>'''
    
    def goal_test (self, block):
        """
        Used to determine if the goal has been reached by comparing the environment's
        goal block type to the given
        """
        if block == Environment.GOAL_BLOCK:
            self.goal_reached = True
            print("[$] Mission Successful! Deaths: " + str(self.attempt - 1))
    
    def start_mission (self):
        """
        Manages the agent's action loop and the environment's record-keeping
        mechanics, including how to restart if the agent dies or time expires
        """
        if (self.attempt <= MAX_RETRIES):
            print("[~] Starting mission - Attempt #" + str(self.attempt))
            self.malmo.startMission( self.mission, self.mission_record )
            world_state = self.malmo.getWorldState()
            while not world_state.has_mission_begun:
                sys.stdout.write(".")
                time.sleep(0.1)
                world_state = self.malmo.getWorldState()
            
            print("[!] Mission started!")
            while world_state.is_mission_running and not self.goal_reached:
                time.sleep(TICK_LENGTH)
                self.agent.act()
                world_state = self.malmo.getWorldState()
            
            if not self.goal_reached:
                self.attempt += 1
                self.start_mission()
        else:
            print("[X] GAME OVER!")
            