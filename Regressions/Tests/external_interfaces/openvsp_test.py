# weights.py
import  RCAIDE
import sys
import os

sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
# the analysis functions

from Boeing_737             import vehicle_setup as transport_setup
from Cessna_172             import vehicle_setup as general_aviation_setup
from Boeing_BWB_450         import vehicle_setup as bwb_setup
from Stopped_Rotor_EVTOL    import vehicle_setup as evtol_setup

from RCAIDE.Framework.External_Interfaces.OpenVSP.export_vsp_vehicle import export_vsp_vehicle 
from RCAIDE.Framework.External_Interfaces.OpenVSP.import_vsp_vehicle import import_vsp_vehicle
from RCAIDE.Library.Plots import  *  

def main():
    try:
        import vsp as vsp
        Transport_Aircraft_Test()
        BWB_Aircraft_Test()
        General_Aviation_Test() 
        EVTOL_Aircraft_Test() 
    except ImportError:
        pass
           
    return


def Transport_Aircraft_Test():
    
    vehicle   = transport_setup()  
    export_vsp_vehicle(vehicle, 'Boeing_737')
    
    propulsor_type = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan()
    network_type   = RCAIDE.Framework.Networks.Fuel()
    
    vsp_vehicle  = import_vsp_vehicle('Boeing_737.vsp3',network_type=network_type,propulsor_type=propulsor_type) 
    return

        
def BWB_Aircraft_Test(): 
    vehicle  = bwb_setup() 
    
    export_vsp_vehicle(vehicle, 'BWB')

    propulsor_type = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan()
    network_type   = RCAIDE.Framework.Networks.Fuel()
    vsp_vehicle  = import_vsp_vehicle('BWB.vsp3',network_type=network_type,propulsor_type=propulsor_type)
    
    return


def General_Aviation_Test():
      
    vehicle  = general_aviation_setup() 
    export_vsp_vehicle(vehicle, 'Cessna_172')

    propulsor_type = RCAIDE.Library.Components.Powertrain.Propulsors.ICE_Propeller()
    network_type   = RCAIDE.Framework.Networks.Fuel()
    vsp_vehicle  = import_vsp_vehicle('Cessna_172.vsp3',network_type=network_type,propulsor_type=propulsor_type)     
    return

def EVTOL_Aircraft_Test():  
    vehicle  = evtol_setup(new_regression = False) 
    export_vsp_vehicle(vehicle, 'Stopped_Rotor_EVTOL')  


    propulsor_type = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()
    network_type   = RCAIDE.Framework.Networks.Electric()
    vsp_vehicle  = import_vsp_vehicle('Stopped_Rotor_EVTOL.vsp3',network_type=network_type,propulsor_type=propulsor_type)      
    return
 
 
if __name__ == '__main__':
    main()
