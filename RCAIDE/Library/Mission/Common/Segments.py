# RCAIDE/Library/Missions/Common/helper_functions.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
# RCAIDE imports 
import RCAIDE 
from tqdm import tqdm
import  numpy as  np

def sequential_segments(mission):

    print('\n Mission Solver Initiated')    
    pbar = tqdm(total=100)
    progress_interval = round(np.ceil(100/ len(mission.segments)))
    last_tag = None
    for tag,segment in mission.segments.items():
        segment.mission_tag = mission.tag
        if last_tag:
            segment.state.initials = mission.segments[last_tag].state
        last_tag = tag        
        
        segment.process.initialize.expand_state(segment) 
        segment.process.initialize.expand_state = RCAIDE.Library.Methods.skip        
        segment.evaluate()
        print('\n Solving', segment.tag, 'segment. Mission simulation completion status: ')
        pbar.update(progress_interval)
        print('\n')
    pbar.close()
            
            