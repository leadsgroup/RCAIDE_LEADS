# RCAIDE/Library/Components/Powertrain/Energy/Sources/Solar_Panels/Solar_Panel.py
# 
# 
# Created:  Oct 2024, M. Clarke 
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
from RCAIDE.Library.Components    import Component    

# ----------------------------------------------------------------------------------------------------------------------
#  Solar_Panel
# ----------------------------------------------------------------------------------------------------------------------
class Solar_Panel(Component):
    """
    Class for modeling photovoltaic solar panels in aircraft applications
    
    Attributes
    ----------
    area : float
        Solar panel surface area [m^2] (default: 0.0)
        
    efficiency : float
        Solar energy conversion efficiency [-] (default: 0.0)

    Notes
    -----
    The solar panel model provides basic functionality for converting solar
    radiation into electrical power based on panel area and efficiency.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Modulators.Solar_Logic
        Solar power management and control
    """       
    def __defaults__(self):
        """
        Sets default values for solar panel attributes
        """        
        self.area       = 0.0
        self.efficiency = 0.0
    #def solar_radiation(self,conditions):  
        #"""Computes the adjusted solar flux

        #Assumptions:
        #Solar intensity =1305 W/m^2
        #Includes a diffuse component of 0% of the direct component
        #Altitudes are not excessive 

        #Source:
        #N/A

        #Inputs:
        #conditions.frames.
          #planet.start_time        [s]
          #planet.latitude          [degrees]
          #planet.longitude         [degrees]
          #body.inertial_rotations  [radians]
          #inertial.time            [s]
        #conditions.freestream.
          #altitude                 [m]

        #Outputs:
        #self.outputs.flux          [W/m^2]
        #flux                       [W/m^2]

        #Properties Used:
        #N/A
        #"""            
        
        ## Unpack
        #timedate  = conditions.frames.planet.start_time
        #latitude  = conditions.frames.planet.latitude
        #longitude = conditions.frames.planet.longitude
        #phip      = conditions.frames.body.inertial_rotations[:,0,None]
        #thetap    = conditions.frames.body.inertial_rotations[:,1,None]
        #psip      = conditions.frames.body.inertial_rotations[:,2,None]
        #altitude  = conditions.freestream.altitude
        #times     = conditions.frames.inertial.time
        
        ## Figure out the date and time
        #day       = timedate.tm_yday + np.floor_divide(times, 24.*60.*60.)
        #TUTC      = timedate.tm_sec + 60.*timedate.tm_min+ 60.*60.*timedate.tm_hour + np.mod(times,24.*60.*60.)
        
        ## Gamma is defined to be due south, so
        #gamma = psip - np.pi
        
        ## Solar intensity external to the Earths atmosphere
        #Io = 1367.0
        
        ## Indirect component adjustment
        #Ind = 1.0
        
        ## B
        #B = (360./365.0)*(day-81.)*np.pi/180.0
        
        ## Equation of Time
        #EoT = 9.87*np.sin(2*B)-7.53*np.cos(B)-1.5*np.sin(B)
        
        ## Time Correction factor
        #TC = 4*longitude+EoT
        
        ## Local Solar Time
        #LST = TUTC/3600.0+TC/60.0
        
        ## Hour Angle   
        #HRA = (15.0*(LST-12.0))*np.pi/180.0
        
        ## Declination angle (rad)
        #delta = -23.44*np.cos((360./365.)*(day+10.)*np.pi/180.)*np.pi/180.
        
        ## Zenith angle (rad)
        #psi = np.arccos(np.sin(delta)*np.sin(latitude*np.pi/180.0)+np.cos(delta)*np.cos(latitude*np.pi/180.0)*np.cos(HRA))
        
        ## Solar Azimuth angle, Duffie/Beckman 1.6.6
        #gammas = np.sign(HRA)*np.abs((np.cos(psi)*np.sin(latitude*np.pi/180.)-np.sin(delta))/(np.sin(psi)*np.cos(latitude*np.pi/180)))
        
        ## Slope of the solar panel, Bower AIAA 2011-7072 EQN 15
        #beta = np.reshape(np.arccos(np.cos(thetap)*np.cos(phip)),np.shape(gammas))
        
        ## Angle of incidence, Duffie/Beckman 1.6.3
        #theta = np.arccos(np.cos(psi)*np.cos(beta)+np.sin(psi)*np.sin(beta)*np.cos(gammas-gamma))
        
        #flux = np.zeros_like(psi)
        
        #earth = RCADE.Attributes.Planets.Earth()
        #Re = earth.mean_radius
        
        ## Atmospheric properties
        #Yatm = 9. # The atmospheres thickness in km
        #r    = Re/Yatm
        #c    = altitude/9000.
        
        ## Air mass
        #AM = np.zeros_like(psi)
        #AM[altitude<9000.] = (((r+c[altitude<9000.])*(r+c[altitude<9000.]))*(np.cos(psi[altitude<9000.])*np.cos(psi[altitude<9000.]))+2.*r*(1.-c[altitude<9000.])-c[altitude<9000.]*c[altitude<9000.] +1.)**(0.5)-(r+c[altitude<9000.])*np.cos(psi[altitude<9000.])
        
        ## Direct component 
        #Id = Ind*Io*(0.7**(AM**0.678))
        
        ## Horizontal flux
        #Ih = Id*(np.cos(latitude*np.pi/180.)*np.cos(delta)*np.cos(HRA)+np.sin(latitude*np.pi/180.)*np.sin(delta))              
        
        ## Flux on the inclined panel, if the altitude is less than 9000 meters
        #I = Ih*np.cos(theta)/np.cos(psi)
        
        ## Now update if the plane is outside the majority of the atmosphere, (9km)
        #Id = Ind*Io
        #Ih = Id*(np.cos(latitude*np.pi/180.)*np.cos(delta)*np.cos(HRA)+np.sin(latitude*np.pi/180.)*np.sin(delta))           
        #I[altitude>9000.] = Ih[altitude>9000.]*np.cos(theta[altitude>9000.])/np.cos(psi[altitude>9000.])
        
        ## Now if the sun is on the other side of the Earth...
        #I[((psi<-np.pi/2.)|(psi>96.70995*np.pi/180.))] = 0
        
        #flux = np.maximum(0.0,I)        
        
        ## Store to outputs
        #self.outputs.flux = flux      
        
        ## Return result for fun/convenience
        #return flux
    
    #def power(self):
        #"""This determines the power output of the solar cell.

        #Assumptions:
        #None

        #Source:
        #None

        #Inputs:
        #self.inputs.flux   [W/m^2]

        #Outputs:
        #self.outputs.power [W]
        #power              [W]

        #Properties Used:
        #self.efficiency    [-]
        #self.area          [m^2]
        #"""        
        ## Unpack
        #flux       = self.inputs.flux
        #efficiency = self.efficiency
        #area       = self.area
        
        #p = flux*area*efficiency
        
        ## Store to outputs
        #self.outputs.power = p
    
        #return p
    
    
    
    