# RCAIDE/Library/Missions/Common/helper_functions.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
# RCAIDE imports 
import RCAIDE 
from RCAIDE.Framework.Core  import Data 

def pre_process(mission): 
    for tag,segment in mission.segments.items():     
        segment.pre_process()

def sequential_segments(mission):  
    
    last_tag = None
    for tag,segment in mission.segments.items():
        segment.mission_tag = mission.tag
        if last_tag:
            segment.state.initials = mission.segments[last_tag].state
        last_tag = tag        
        
        segment.process.initialize.expand_state(segment) 
        segment.process.initialize.expand_state = RCAIDE.Library.Methods.skip        
        segment.evaluate()
        
def update_segments(mission):   
    for tag,segment in mission.segments.items():
        segment.post_process() 
        
def merge_segment_states(mission): 
    mission.state.update(mission.merged())
    
def unpack_segments(mission): 
    # Build a dict with the sections, sections start at 0
    counter = Data()
    
    for key in mission.state.unknowns.keys():
        counter[key] = 0

    for i, segment in enumerate(mission.segments):
        for key in segment.state.unknowns.keys():
            if key=='tag':
                continue
            points = segment.state.unknowns[key].size
            segment.state.unknowns[key] = mission.state.unknowns[key][counter[key]:counter[key]+points]
            counter[key] = counter[key]+points
            
    return
            
            