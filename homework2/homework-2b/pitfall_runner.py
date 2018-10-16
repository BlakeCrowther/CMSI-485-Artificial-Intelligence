from environment import Environment

if __name__ == "__main__":
    mazes = [\
        # Easy difficulty: Need only 1 life to solve
        ["XXXXXX",
         "X...GX",\
         "X..P.X",\
         "X....X",\
         "X..P.X",\
         "X*...X",\
         "XXXXXX"],
        
        # Medium difficulty: Need only 1 life to solve
        ["XXXXXXXXX",
         "X..PGP..X",\
         "X.......X",\
         "X..P.P..X",\
         "X.......X",\
         "X..*....X",\
         "XXXXXXXXX"],
        
        # Hard difficulty: Needs a good heuristic and 2-3 lives
        ["XXXXXXXXX",
         "X..PGP..X",\
         "X.......X",\
         "X..PPPP.X",\
         "XP.....PX",\
         "X...*...X",\
         "XXXXXXXXX"],
    ]
    
    # Pick your difficulty!
    env = Environment(mazes[2])
    env.start_mission()
