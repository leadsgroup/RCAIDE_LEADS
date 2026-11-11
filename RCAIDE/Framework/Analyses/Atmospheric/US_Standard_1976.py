#RCAIDE/Frameworks/Analyses/Atmospheric/US_Standard_1976.py
#
# Created: Dec 2024, M. Clarke
# Modified: Nov 2026, M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Analyses.Atmospheric      import Atmospheric
from RCAIDE.Framework.Mission.Common.Conditions import Conditions 
from RCAIDE.Framework.Core.Arrays               import atleast_2d_col, array_namespace
from RCAIDE.Library.Attributes.Gases            import Air
from RCAIDE.Library.Attributes.Planets          import Earth

# pthon imports 
import numpy as np
from warnings import warn 

# ----------------------------------------------------------------------
#  Classes
# ----------------------------------------------------------------------

class US_Standard_1976(Atmospheric):

    """ Implements the U.S. Standard Atmosphere (1976 version)
        
    Assumptions:
    None
    
    Source:
    U.S. Standard Atmosphere, 1976, U.S. Government Printing Office, Washington, D.C., 1976
    """
    
    def __defaults__(self):
        """This sets the default values for the analysis to function.

        Assumptions:
        None

        Source:
        N/A

        Inputs:
        None

        Output:
        None

        Properties Used:
        None
        """      
        atmo_data = RCAIDE.Library.Attributes.Atmospheres.Earth.US_Standard_1976()
        self.update(atmo_data)         
        planet = RCAIDE.Framework.Analyses.Planets.Earth()
        self.features.planet = planet.features
    
    def compute_values(self,altitude,temperature_deviation=0.0,var_gamma=False):

        """Computes atmospheric values.

        Assumptions:
        US 1976 Standard Atmosphere

        Source:
        U.S. Standard Atmosphere, 1976, U.S. Government Printing Office, Washington, D.C., 1976

        Inputs:
        altitude                                 [m]
        temperature_deviation                    [K]

        Output:
        atmo_data.
          pressure                               [Pa]
          temperature                            [K]
          speed_of_sound                         [m/s]
          dynamic_viscosity                      [kg/(m*s)]
          kinematic_viscosity                    [m^2/s]
          thermal_conductivity                   [W/(m*K)]
          prandtl_number                         [-]
           
        Properties Used:
        self.
          fluid_properties.gas_specific_constant [J/(kg*K)]
          planet.sea_level_gravity               [m/s^2]
          planet.mean_radius                     [m]
          breaks.
            altitude                             [m]
            temperature                          [K]
            pressure                             [Pa]
        """

        # unpack
        zs        = altitude
        gas       = self.fluid_properties
        planet    = self.planet
        grav      = self.planet.sea_level_gravity        
        Rad       = self.planet.mean_radius
        R         = gas.gas_specific_constant
        delta_isa = temperature_deviation
        
        # check properties
        if not gas == Air():
            warn('US Standard Atmosphere not using Air fluid properties')
        if not planet == Earth():
            warn('US Standard Atmosphere not using Earth planet properties')          
        
        # convert input if necessary
        zs = atleast_2d_col(zs)
        xp = array_namespace(zs)

        # get model altitude bounds
        zmin = self.breaks.altitude[0]
        zmax = self.breaks.altitude[-1]

        # convert geometric to geopotential altitude
        zs = zs/(1 + zs/Rad)

        # check ranges; the warning needs concrete values, so it is skipped while JAX traces
        if xp is np:
            if np.amin(zs) < zmin:
                print("Warning: altitude requested below minimum for this atmospheric model; returning values for h = -2.0 km")
            if np.amax(zs) > zmax:
                print("Warning: altitude requested above maximum for this atmospheric model; returning values for h = 86.0 km")
        zs = xp.clip(zs, zmin, zmax)

        # find the layer each altitude is in, and that layer's base values and lapse rate
        z0    = zs * 0.0
        T0    = zs * 0.0
        p0    = zs * 0.0
        alpha = zs * 0.0
        for i in range( len(self.breaks.altitude)-1 ):
            i_inside = (zs >= self.breaks.altitude[i]) & (zs <= self.breaks.altitude[i+1])
            z0    = xp.where(i_inside, self.breaks.altitude[i], z0)
            T0    = xp.where(i_inside, self.breaks.temperature[i], T0)
            p0    = xp.where(i_inside, self.breaks.pressure[i], p0)
            alpha = xp.where(i_inside, -(self.breaks.temperature[i+1] - self.breaks.temperature[i])/ \
                                        (self.breaks.altitude[i+1]    - self.breaks.altitude[i]), alpha)

        # pressure: isothermal layers use the exponential, the others the power law. Both formulas are evaluated
        # everywhere, so the unused power law must stay finite for gradients: in isothermal layers its base is
        # exactly 1 (zero lapse rate) and its exponent uses a lapse rate of 1 instead of dividing by zero
        dz          = zs-z0
        i_isoth     = (alpha == 0.)
        alpha_safe  = xp.where(i_isoth, 1., alpha)
        p_isoth     = p0 * xp.exp(-1.*dz*grav/(R*T0))
        p_adiab     = p0 * ( (1.-alpha*dz/T0) **(1.*grav/(alpha_safe*R)) )
        p           = xp.where(i_isoth, p_isoth, p_adiab)

        T     = T0 - dz*alpha + delta_isa
        rho   = gas.compute_density(T,p)
        a     = gas.compute_speed_of_sound(T,p,var_gamma)
        mu    = gas.compute_absolute_viscosity(T)
        K     = gas.compute_thermal_conductivity(T)  
        Pr    = gas.compute_prandtl_number(T)
        Cp    = gas.compute_cp(T,p)
        gamma = gas.compute_gamma(T,p)
        
        atmo_data = Conditions()
        atmo_data.expand_rows(zs.shape[0])
        atmo_data.pressure                        = p
        atmo_data.temperature                     = T
        atmo_data.density                         = rho
        atmo_data.speed_of_sound                  = a
        atmo_data.dynamic_viscosity               = mu
        atmo_data.kinematic_viscosity             = mu/rho
        atmo_data.thermal_conductivity            = K
        atmo_data.prandtl_number                  = Pr 
        atmo_data.constant_pressure_specific_heat = Cp
        atmo_data.specific_heat                   = gamma
        
        return atmo_data