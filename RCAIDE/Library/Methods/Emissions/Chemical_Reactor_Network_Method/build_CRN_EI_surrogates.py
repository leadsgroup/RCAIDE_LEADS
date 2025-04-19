# RCAIDE/Library/Methods/Emissions/Chemical_Reactor_Network_Method/build_CRN_EI_surrogates.py
#  
# Created: June 2024, M. Clarke, M. Guidotti 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from RCAIDE.Framework.Core import  Data 

# package imports 
from scipy.interpolate  import RegularGridInterpolator

# ----------------------------------------------------------------------------------------------------------------------
#  build_CRN_EI_surrogates
# ---------------------------------------------------------------------------------------------------------------------- 
def build_CRN_EI_surrogates(emissions):

    """
    Builds surrogate models for emission indices using Chemical Reactor Network (CRN) training data.

    Parameters
    ----------
    emissions : Data 
        Container of emission data and surrogate models
        
        - surrogates : Data
            Container for surrogate models
            
        - training : Data
            Training data container
            
            - pressure : ndarray
                Array of pressure values [Pa]
            - temperature : ndarray
                Array of temperature values [K]
            - air_mass_flowrate : ndarray
                Array of air mass flow rates [kg/s]
            - fuel_to_air_ratio : ndarray
                Array of fuel-to-air ratios [-]
            - EI_CO2 : ndarray
                CO2 emission index training data [kg_CO2/kg_fuel]
            - EI_CO : ndarray
                CO emission index training data [kg_CO/kg_fuel]
            - EI_H2O : ndarray
                H2O emission index training data [kg_H2O/kg_fuel]
            - EI_NO : ndarray
                NO emission index training data [kg_NO/kg_fuel]
            - EI_NO2 : ndarray
                NO2 emission index training data [kg_NO2/kg_fuel]

    Returns
    -------
    Updates the emissions.surrogates Data structure with interpolation functions

    Notes
    -----
    Creates RegularGridInterpolator objects for each emission species using the training data.
    The interpolators are stored in the emissions.surrogates Data structure.

    The surrogate models interpolate emission indices based on:

    - Pressure
    - Temperature 
    - Air mass flow rate
    - Fuel-to-air ratio

    **Extra modules required**

    * scipy.interpolate

    **Major Assumptions**

    * Linear interpolation is sufficient between training points

    See Also
    --------
    RCAIDE.Library.Methods.Emissions.Chemical_Reactor_Network_Method.train_CRN_EI_surrogates
    RCAIDE.Library.Methods.Emissions.Chemical_Reactor_Network_Method.evaluate_CRN_emission_indices
    """
     
    surrogates                            = emissions.surrogates
    training                              = emissions.training  
    pressure_data                         = training.pressure         
    temperature_data                      = training.temperature      
    air_mass_flowrate_data                = training.air_mass_flowrate
    fuel_to_air_ratio_data                = training.fuel_to_air_ratio

    surrogates.EI_CO2                     = RegularGridInterpolator((pressure_data ,temperature_data, air_mass_flowrate_data, fuel_to_air_ratio_data),training.EI_CO2 ,method = 'linear',   bounds_error=False, fill_value=None) 
    surrogates.EI_CO                      = RegularGridInterpolator((pressure_data ,temperature_data, air_mass_flowrate_data, fuel_to_air_ratio_data),training.EI_CO  ,method = 'linear',   bounds_error=False, fill_value=None) 
    surrogates.EI_H2O                     = RegularGridInterpolator((pressure_data ,temperature_data, air_mass_flowrate_data, fuel_to_air_ratio_data),training.EI_H2O ,method = 'linear',   bounds_error=False, fill_value=None) 
    surrogates.EI_NO                      = RegularGridInterpolator((pressure_data ,temperature_data, air_mass_flowrate_data, fuel_to_air_ratio_data),training.EI_NO  ,method = 'linear',   bounds_error=False, fill_value=None) 
    surrogates.EI_NO2                     = RegularGridInterpolator((pressure_data ,temperature_data, air_mass_flowrate_data, fuel_to_air_ratio_data),training.EI_NO2 ,method = 'linear',   bounds_error=False, fill_value=None) 
   
    return