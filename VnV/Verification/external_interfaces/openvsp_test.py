# weights.py
import  RCAIDE
import sys
import os

base_dir = os.path.dirname(os.path.abspath(__file__))

vehicles_path = os.path.abspath(
    os.path.join(base_dir, "..", "..", "Vehicles")
)

if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)
# the analysis functions

from Boeing_737             import vehicle_setup as transport_setup
from Cessna_172             import vehicle_setup as general_aviation_setup
from BWB                    import vehicle_setup as bwb_setup
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
    vsp_vehicle    = import_vsp_vehicle('BWB.vsp3',network_type=network_type,propulsor_type=propulsor_type)
    
    return


def General_Aviation_Test():
      
    vehicle  = general_aviation_setup() 
    export_vsp_vehicle(vehicle, 'Cessna_172')

    propulsor_type = RCAIDE.Library.Components.Powertrain.Propulsors.Internal_Combustion_Engine()
    network_type   = RCAIDE.Framework.Networks.Fuel()
    vsp_vehicle    = import_vsp_vehicle('Cessna_172.vsp3',network_type=network_type,propulsor_type=propulsor_type)     
    return

def EVTOL_Aircraft_Test():  
    vehicle  = evtol_setup(new_regression = False) 
    export_vsp_vehicle(vehicle, 'Stopped_Rotor_EVTOL')  


    propulsor_type = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()
    network_type   = RCAIDE.Framework.Networks.Electric()
    vsp_vehicle    = import_vsp_vehicle('Stopped_Rotor_EVTOL.vsp3',network_type=network_type,propulsor_type=propulsor_type)      
    return
 
 
if __name__ == '__main__':
    main()
