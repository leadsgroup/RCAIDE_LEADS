
# RCAIDE/Core/Container.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------        
import RCAIDE
from .               import Data
from warnings        import warn
import random

import string
chars = string.punctuation + string.whitespace
t_table = str.maketrans( chars          + string.ascii_uppercase , 
                            '_'*len(chars) + string.ascii_lowercase )

# ----------------------------------------------------------------------------------------------------------------------
#  Container
# ----------------------------------------------------------------------------------------------------------------------   

class Container(Data):
    """ A dict-type container with attribute, item and index style access
        intended to hold a attribute-accessible list of Data(). This is unordered.
        
        Assumptions:
        N/A
        
        Source:
        N/A
        
    """
            
        
    def __defaults__(self):
        """ Defaults function
    
            Assumptions:
            None
        
            Source:
            N/A
        
            Inputs:
            N/A
        
            Outputs:
            N/A
            
            Properties Used:
            N/A
        """          
        pass
    
    def __init__(self,*args,**kwarg):
        """ Initialization that builds the container
        
            Assumptions:
            None
        
            Source:
            N/A
        
            Inputs:
            self
        
            Outputs:
            N/A
            
            Properties Used:
            N/A
        """          
        super(Container,self).__init__(*args,**kwarg)
        self.__defaults__()
    
    def append(self,val):
        """ Appends the value to the containers
            This overrides the Data class append by allowing for duplicate named components
            The following components will get new names.
        
            Assumptions:
            None
        
            Source:
            N/A
        
            Inputs:
            self
        
            Outputs:
            N/A
            
            Properties Used:
            N/A
        """           
        
        old_tags = []
        old_tags = get_tags(self,old_tags) 
        check_tags(val,old_tags)     
        Data.append(self,val) 
         
        return
    
        
    def extend(self,vals):
        # """ Append things regressively depending on what is inside.
    
        #     Assumptions:
        #     None
        
        #     Source:
        #     N/A
        
        #     Inputs:
        #     self
        
        #     Outputs:
        #     N/A
            
        #     Properties Used:
        #     N/A
        # """         
        if isinstance(vals,(list,tuple)):
            for v in vals: self.append(v) #originally val
        elif isinstance(vals,dict):
            self.update(vals)
        else:
            raise Exception('unrecognized data type') 

def get_tags(item,tag_list):
        
    if isinstance(item, RCAIDE.Library.Components.Component) or isinstance(item, dict):
        for s_tag, s_item in item.items():
            if 'tag' == s_tag:
                item.tag = str.lower(s_item.translate(t_table))
                tag_list.append(item.tag)
            if isinstance(s_item, RCAIDE.Library.Components.Component): 
                for ss_tag, ss_item in s_item.items(): 
                    if 'tag' == ss_tag:
                        s_item.tag = str.lower(ss_item.translate(t_table))
                        tag_list.append(s_item.tag)
                    if isinstance(ss_item, RCAIDE.Library.Components.Component):
                        for sss_tag, sss_item in ss_item.items():
                            if 'tag' == sss_tag:
                                ss_item.tag = str.lower(sss_item.translate(t_table))
                                tag_list.append(ss_item.tag) 
                            if isinstance(sss_item, RCAIDE.Library.Components.Component):
                                for ssss_tag, ssss_item in sss_item.items():
                                    if 'tag' == ssss_tag:
                                        sss_item.tag = str.lower(ssss_item.translate(t_table))
                                        tag_list.append(sss_item.tag)                                
                            
    return tag_list
                            
def check_tags(item,tag_list):
    if isinstance(item, RCAIDE.Library.Components.Component) or isinstance(item, dict):
        for s_tag, s_item in item.items():
            if 'tag' == s_tag:
                if s_item in tag_list:
                    unmodified_tag = str.lower(s_item.translate(t_table))
                    string_of_keys = "".join(tag_list)
                    n_comps        = string_of_keys.count(unmodified_tag)
                    item.tag       = unmodified_tag + str(n_comps+1)
                else:
                    item.tag = str.lower(s_item.translate(t_table))
                tag_list.append(item.tag)
            if isinstance(s_item, RCAIDE.Library.Components.Component): 
                for ss_tag, ss_item in s_item.items(): 
                    if 'tag' == ss_tag: 
                        if ss_item in tag_list:
                            unmodified_tag = str.lower(ss_item.translate(t_table))  
                            string_of_keys = "".join(tag_list)
                            n_comps        = string_of_keys.count(unmodified_tag)
                            s_item.tag     = unmodified_tag + str(n_comps+1)
                        else:
                            s_item.tag = str.lower(ss_item.translate(t_table))
                        tag_list.append(s_item.tag) 
                    if isinstance(ss_item, RCAIDE.Library.Components.Component):
                        for sss_tag, sss_item in ss_item.items():
                            if 'tag' == sss_tag: 
                                if sss_item in tag_list:
                                    unmodified_tag = str.lower(sss_item.translate(t_table))  
                                    string_of_keys = "".join(tag_list)
                                    n_comps        = string_of_keys.count(unmodified_tag)
                                    ss_item.tag    = unmodified_tag + str(n_comps+1)
                                else:
                                    ss_item.tag = str.lower(sss_item.translate(t_table))  
                                tag_list.append(ss_item.tag) 
                            if isinstance(sss_item, RCAIDE.Library.Components.Component):
                                for ssss_tag, ssss_item in sss_item.items():
                                    if 'tag' == ssss_tag: 
                                        if ssss_item in tag_list:
                                            unmodified_tag = str.lower(ssss_item.translate(t_table))  
                                            string_of_keys = "".join(tag_list)
                                            n_comps        = string_of_keys.count(unmodified_tag)
                                            sss_item.tag   = unmodified_tag + str(n_comps+1)
                                        else:
                                            sss_item.tag = str.lower(ssss_item.translate(t_table))  
                                        tag_list.append(sss_item.tag)                                   
    return  
