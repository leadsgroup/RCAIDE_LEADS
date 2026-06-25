import numpy as np

class LandingGearNoiseModel:
    def __init__(self, gear_params, flight_params):
        """
        
        gear_params: dict
            - num_wheels (Nw): Number of wheels
            - wheel_diam (d): Wheel diameter [inches]
            - wheel_width (w): Wheel width [inches]
            - strut_lengths (L_j): List of lengths of struts [inches]
            - strut_dims (dim_j): List of diameters/widths of struts [inches]
            - aircraft_weight (W_ac): Max Takeoff Weight [lbs]
            - track_angle (gamma): Wheel track alignment angle [degrees]
            
        flight_params: dict
            - M_flight: Flight Mach number
            - theta: Emission angle [degrees] (90 is overhead)
            - R: Distance to observer [ft]
            - c0: Speed of sound [ft/s] (default 1116)
            - rho0: Air density [slugs/ft^3] (default 0.00237)
        """

        self.gp = gear_params
        self.fp = flight_params
        
        # Environmental Constants
        self.c0 = flight_params.get('c0', 1116.4) # speed of found [ft/s.]
        self.rho0 = flight_params.get('rho0', 0.00237) # air density in slugs
        # Reference pressure 2e-5 Pa converted to psf approx 4.177e-7
        self.p_ref_val = 4.177e-7 

        # Flow Parameters
        # Local Mach number is typically 0.75 * M_flight (Eq. 58) [cite: 796]
        self.M_local = 0.75 * self.fp['M_flight']
        
        # Doppler Factor (Eq. 8) [cite: 208]
        self.theta_rad = np.radians(self.fp['theta'])
        self.Doppler = 1 - self.M_local * np.cos(self.theta_rad)
        
        # Component Coefficients (Table 2 & 3) [cite: 365, 488]
        self.params = {
            'Low': {
                'beta': 4.5e-8, 'St0': 1.0, 'sigma': 4.0, 'mu': 2.5, 'q': 2.6, 
                'h': 0.2, 'A': 3.53, 'B': 0.62
            },
            'Mid': {
                'beta': 1.5e-8, 'St0': 0.3, 'sigma': 3.0, 'mu': 1.5, 'q': 4.2, 
                'h': 0.6, 'A': 0.42, 'B': 0.18
            },
            'High': {
                'beta': 3.2e-5, 'St0': 0.1, 'sigma': 2.0, 'mu': 1.1, 'q': 4.2, 
                'h': 1.0, 'A': 0.08, 'B': 0.10
            }
        }

        self._calculate_geometry()

    def _calculate_geometry(self):
        """Calculates S, l0, and complexity factor."""
        # --- LOW FREQ (Wheels) ---
        # S_L = pi * Nw * w * d (Eq. 33)
        Nw = self.gp['num_wheels']
        w_ft = self.gp['wheel_width']
        d_ft = self.gp['wheel_diam']
        self.S_L = np.pi * Nw * w_ft * d_ft
        self.l0_L = d_ft # Length scale is diameter

        # --- MID FREQ (Struts) ---
        # S_M = sum(perimeter_j * L_j) (Eq. 35)
        L_struts_ft = np.array(self.gp['strut_lengths'])
        D_struts_ft = np.array(self.gp['strut_dims'])
        
        perimeters = np.pi * D_struts_ft # Assuming circular approx
        self.S_M = np.sum(perimeters * L_struts_ft)
        
        self.L_total_ft = np.sum(L_struts_ft)
        self.L_total_in = self.L_total_ft * 12.0
        
        # Average dimension a = S_M / (pi * L_total) (Eq. 38)
        if self.L_total_ft > 0:
            self.a_ft = self.S_M / (np.pi * self.L_total_ft) #cross section dimension
        else:
            self.a_ft = 1.0 # Avoid div/0
        self.l0_M = self.a_ft

        # --- HIGH FREQ (Complexity) ---
        # Refs: N_ref=2, W_ref=150,000lb, L_ref=300in
        N_ref, W_ref, L_ref = 2.0, 150000.0, 300.0
        W_ac = self.gp['aircraft_weight']
        gamma = np.radians(self.gp.get('track_angle', 0.0)) #local get of 0
        print('gamma',gamma)
        
        term1 = 1 + 0.028 * ((Nw / N_ref) * (self.L_total_in / L_ref) * (W_ac / W_ref) - 1)
        term2 = 1 + 2 * ((Nw - 2) / Nw) * np.sin(2 * gamma) if Nw > 0 else 1 #at least 1 no. wheel on MLG
        
        # Guard for cases where Nw=2 (term2 becomes 1) or Nw < 2
        if Nw <= 2: term2 = 1.0
        
        self.eta = term1 * term2 #complexity factor
        
        # Length scale l = 0.15 * a (Eq. 42)
        self.l0_H = 0.15 * self.l0_M
        
        # S_H = eta * l^2 (Eq. 40)
        self.S_H = self.eta * (self.l0_H**2)

    def _normalized_spectrum(self, St, comp_type):
        """Calculates F(St) using Eq. 43[cite: 445]."""
        p = self.params[comp_type]

        num = p['A'] * (St**p['sigma'])
        den = (p['B'] + St**p['mu'])**p['q']
        return num / den

    def _directivity_component(self, theta_deg, comp_type):
        """Calculates D(theta) using Eq. 51."""
        h = self.params[comp_type]['h']
        theta_rad = np.radians(theta_deg)
        return (1 + h * np.cos(theta_rad)**2)**2

    def _installation_directivity(self, theta_deg):
        """Calculates D0(theta) using Eq. 52."""
        theta_rad = np.radians(theta_deg)
        return 1.2 * (1 - 0.9 * np.cos(theta_rad)**2)**2

    def predict_spectrum(self, frequencies):
        """Calculates SPL for a list of frequencies."""
        R_ft = self.fp['R']
        M = self.M_local
        
        # Base Amplitude Term (Eq 31) 
        # (rho0 * c0^2)^2 * M^6 / (R^2 * (1-M cos)^4)
        # amb_term = (self.rho0 * self.c0**2)**2
        amb_term = (self.rho0 * self.c0**2)**2
        #e_term = np.e**(-)
        #conv_term = (1 - M * np.cos(self.theta_rad))**4
        conv_term = 1.0 # convective amplification should be removed (set to 1.0) because there is no relative motion between the gear and the microphone.
        #according to paper
        spread_term = R_ft**2
        
        P_base = (amb_term * M**6) / (spread_term * conv_term)
        
        # Installation Effect
        #D0 = self._installation_directivity(self.fp['theta'])
        D0 = 1.0
        
        results = {'Freq': frequencies, 'Total': [], 'Low': [], 'Mid': [], 'High': []}
        
        for f in frequencies:
            U = M * self.c0 
            
            # --- Low ---
            St_L = f * self.l0_L / U
            val_L = P_base * D0 * self.params['Low']['beta'] * self.S_L * \
                    self._directivity_component(self.fp['theta'], 'Low') * \
                    self._normalized_spectrum(St_L, 'Low')
            
            # --- Mid ---
            St_M = f * self.l0_M / U
            val_M = P_base * D0 * self.params['Mid']['beta'] * self.S_M * \
                    self._directivity_component(self.fp['theta'], 'Mid') * \
                    self._normalized_spectrum(St_M, 'Mid')
            
            # --- High ---
            St_H = f * self.l0_H / U
            val_H = P_base * D0 * self.params['High']['beta'] * self.S_H * \
                    self._directivity_component(self.fp['theta'], 'High') * \
                    self._normalized_spectrum(St_H, 'High')
            
            # total of freqs
            val_total = val_L + val_M + val_H
            
            # Convert to dB
            def to_db(p2):
                if p2 <= 1e-20: return 0
                return 10 * np.log10(p2 / (self.p_ref_val**2))

            results['Low'].append(to_db(val_L))
            results['Mid'].append(to_db(val_M))
            results['High'].append(to_db(val_H))
            results['Total'].append(to_db(val_total))
            
        return results

def compute_landing_gear_noise(D, H, W, wheels, M, Weight, strut_diameter, theta, distance, frequency, segment):
    gear_params = {
        'num_wheels': wheels,
        'wheel_diam': D/12,  # Approximate
        'wheel_width': W/12, # Approximate
        'strut_lengths': [H/12], # Total length L=317 in
        'strut_dims': [strut_diameter/12],     # Average dimension a=4.65 in
        'aircraft_weight': Weight, # Reference weight (lbs)
        'track_angle': 0.0 #assume track angle of zero
    }


    target_M_local = M #mach number from flight segment needed.
    flight_cond = {
        'M_flight': target_M_local / 0.75, 
        'theta': theta,
        'R': distance, 
        'c0': segment.conditions.freestream.speed_of_sound, #sound speed
        'rho0': segment.conditions.freestream.density  
    }

    model = LandingGearNoiseModel(gear_params, flight_cond)
    freqs = frequency # 30Hz to 10kHz
    SPL = model.predict_spectrum(freqs)
    return SPL