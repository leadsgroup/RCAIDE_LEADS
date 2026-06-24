import numpy as np
import math
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.interpolate_core_noise import get_spl


def compute_core_noise():

    core_noise= Data()

    return core_noise 
# Standard 1/3-octave-band center frequencies (Hz)
standard_freqs = [50, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500, 
                  630, 800, 1000, 1250, 1600, 2000, 2500, 3150, 4000, 
                  5000, 6300]


model_inputs = {
    "W1": 21.1,      # Total core mass flow rate (lbm/sec) -> v001t01a006-85-igt-139.pdf
    "T_C_o": 1800,   # Combustor outlet total temperature (deg R) -> Assume this paper is accurate https://depozit.isae.fr/theses/2005/2005_Roux_Elodie.pdf (pg. 198)
    "T_C_i": 1121,   # Combustor inlet total temperature (deg R) -> T3 of entropy temp chart
    "P_amb": 14.6488, # Ambient pressure (psia)
    "T_amb": 518.4,  # Ambient temperature (deg R)
    "n_f":  18,       # UPDATED: NASA used 18 for JT15D fuel nozzle number (count) (cite: 2969)
    "R": 100,         # UPDATED: Fig 19 is scaled to 100 ft (cite: 4861)
    "D_h_1": 1.2506,   #used to be 2, shift LF to the right.
    "c_amb": 1115,   # Ambient sonic velocity (ft/sec)
    "D_C": 2.25,     # Combustor diameter (ft)
    "c_C_o": 2362.2,  # Combustor exit sonic velocity (ft/sec)
    "f": standard_freqs,# Frequency (Hz) -> injected list
    "theta_c": 120,   #theta
    "pressure_ratio": 13.1 #T/O Pressure ratio-> https://engineering.purdue.edu/~propulsi/propulsion/jets/tfans/jt15d.html
}

def calc_core_param(W1, T_C_o, T_C_i, pressure_ratio, T_amb):
    temp_ratio_diff = T_C_o - T_C_i
    temp_ratio_amb = T_amb / T_C_i
    inner_term = W1 * ((temp_ratio_diff * pressure_ratio * temp_ratio_amb) ** 2)
    return math.log10(inner_term)

def calc_uol_c1(R, n_f, core_param_log):
    C_C1, N_C1, F_C1 = 78.0, 7.0, 14.0
    return C_C1 - (20.0 * math.log10(R)) + (N_C1 * core_param_log) - (F_C1 * math.log10(n_f))

def calc_uol_c2(R, n_f, core_param_log):
    C_C2, N_C2, F_C2 = 60.3, 10.0, 18.0
    return C_C2 - (20.0 * math.log10(R)) + (N_C2 * core_param_log) - (F_C2 * math.log10(n_f))

def calc_uol_c3(R, core_param_log):
    C_C3, N_C3 = 42.5, 9.0
    return C_C3 - (20.0 * math.log10(R)) + (N_C3 * core_param_log)

def calc_strouhal_c1(f, D_h_1, c_amb):
    return (f * D_h_1) / c_amb

def calc_strouhal_c2_c3(f, D_C, c_C_o):
    return (f * D_C) / c_C_o

# 1. Calculate Base Parameters
core_param_log = calc_core_param(
    model_inputs["W1"], model_inputs["T_C_o"], model_inputs["T_C_i"], 
    model_inputs["pressure_ratio"], model_inputs["T_amb"]
)

uol_c1 = calc_uol_c1(model_inputs["R"], model_inputs["n_f"], core_param_log)
uol_c2 = calc_uol_c2(model_inputs["R"], model_inputs["n_f"], core_param_log)
uol_c3 = calc_uol_c3(model_inputs["R"], core_param_log)

# 2. Initialize output arrays and tables
spl_c1_list, spl_c2_list, spl_c3_list = [], [], []

theta_c = model_inputs["theta_c"]

# calculate log_S before get_spl
def get_normalized_spl(interpolator_obj, strouhal_num, theta_c):
    if strouhal_num <= 0:
        return 0
    log_S = math.log10(strouhal_num)
    return get_spl(interpolator_obj, theta_c, log_S)


