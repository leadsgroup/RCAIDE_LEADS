# aerodynamic_crm_validation.py
# Aerodynamic validation of the Common Research Model (CRM) using VLM
# against experimental wind tunnel data at Mach 0.85.

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import RCAIDE
from RCAIDE.Framework.Core                                            import Units, Data
from RCAIDE.Framework.External_Interfaces.OpenVSP.import_vsp_vehicle import import_vsp_vehicle
from RCAIDE.Library.Methods.Performance                               import aircraft_aerodynamic_analysis
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan            import design_turbofan
from RCAIDE.Library.Plots   import plot_aircraft_aerodynamics

from copy   import deepcopy
from io     import StringIO
import pickle
import os
import sys
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from RCAIDE.Library.Plots import *

base_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.abspath(os.path.join(os.path.join(sys.path[0]), "../../Vehicles"))) 
from CRM import vehicle_setup as CRM_setup

# ----------------------------------------------------------------------
#   Experimental Reference Data (Mach ~0.85, Re ~5e7)
#   Source: CRM wind tunnel campaign
# ----------------------------------------------------------------------

raw_data_92 = """\
AoA (deg)\tMach\tRE_ref\tTemperature (F)\tCL\tCD\tCM
-2.83992\t0.849892\t5.0033698\t119.5240021\t-0.290069\t0.0254703\t0.296253
-1.80837\t0.850047\t5.0030799\t119.5780029\t-0.152055\t0.0180762\t0.243471
-1.31433\t0.849711\t5.00035\t119.6259995\t-0.0854695\t0.0164904\t0.218967
-0.820749\t0.849448\t5.0002298\t119.6289978\t-0.0205704\t0.0157003\t0.195744
-0.312387\t0.850052\t5.0008998\t119.7030029\t0.0466929\t0.0155093\t0.172404
0.229618\t0.850203\t5.0014701\t119.6900024\t0.116596\t0.0157593\t0.148772
0.668474\t0.850193\t4.9999299\t119.7979965\t0.174459\t0.0162941\t0.130109
1.15677\t0.849777\t4.9980602\t119.8730011\t0.239466\t0.0172128\t0.110187
1.6593699\t0.850219\t4.9994798\t119.9169998\t0.307011\t0.0186479\t0.0912732
1.9300801\t0.850103\t4.9969702\t120.0759964\t0.344069\t0.0196483\t0.0816128
2.1863999\t0.850218\t4.9969802\t120.1190033\t0.379054\t0.0208287\t0.0724932
2.4152701\t0.850923\t4.9980202\t120.1460037\t0.415042\t0.0220803\t0.0629309
2.65643\t0.850738\t4.9979\t120.1750031\t0.453787\t0.0237041\t0.0519673
2.9351599\t0.850357\t4.9967399\t120.1679993\t0.492912\t0.0260218\t0.0400815
3.16377\t0.849942\t4.99582\t120.1269989\t0.525882\t0.0282517\t0.0304636
3.4015601\t0.850893\t4.9960299\t120.2770004\t0.556315\t0.0312678\t0.0243156
3.6719799\t0.850949\t4.9979801\t120.1760025\t0.580371\t0.0351033\t0.0252739
3.9244299\t0.850595\t4.9980898\t120.1719971\t0.599312\t0.0389397\t0.0290647
4.21771\t0.849216\t4.9941802\t120.0790024\t0.61983\t0.043213\t0.0326177
4.42237\t0.850891\t4.9988899\t120.1719971\t0.630447\t0.047249\t0.0391618
4.6530199\t0.850985\t4.9976001\t120.125\t0.645526\t0.0512715\t0.0432939
4.9145002\t0.850835\t4.9985499\t120.112999\t0.661954\t0.0556064\t0.0461898
5.1538601\t0.850516\t4.9973698\t120.1039963\t0.678735\t0.0596733\t0.0475863
5.6085401\t0.849873\t4.9970002\t120.0230026\t0.701978\t0.0662093\t0.050425
6.21034\t0.84843\t4.9924302\t120.1029968\t0.724428\t0.0748343\t0.0603281
6.7125502\t0.849285\t4.9938598\t120.1750031\t0.744763\t0.0841753\t0.0662167
7.1406398\t0.848729\t4.9906602\t120.2979965\t0.761114\t0.0921219\t0.0616133
7.5858102\t0.849791\t4.9962702\t120.0770035\t0.785667\t0.10216\t0.037346
8.0290203\t0.850005\t4.9960999\t120.1070023\t0.803596\t0.111904\t0.0019705
8.6389103\t0.850149\t4.9965901\t120.1179962\t0.832168\t0.126354\t-0.038431
9.0966501\t0.84901\t4.9932199\t120.0630035\t0.838642\t0.13557\t-0.0567955
9.4848003\t0.849567\t4.99472\t120.1500015\t0.847626\t0.14586\t-0.0794967
10.2521\t0.850526\t4.9964099\t120.2220001\t0.854065\t0.164255\t-0.0851677
"""

# Drag breakdown from paper (drag counts = CD * 1e4)
raw_data_paper = """\
Lift\tCD Profile Drag Counts\tCD Induced Drag Counts\tCD Wave Drag Counts
0.3769\t166.2675\t50.2994\t1.3972
0.4518\t167.6647\t73.3533\t5.5888
0.5047\t170.4591\t90.8184\t9.7804
0.5289\t171.8563\t99.2016\t13.9721
0.5972\t180.9381\t126.4471\t40.5190
0.6501\t250.0998\t152.9940\t72.6547
"""

# Regression truth values — first 5 data points per quantity (snapshot 2026-04-22)
TRUTH_VALUES = {
    'Lift':         np.array([-2.5013857199e-01, -1.1054613425e-01, -4.3671358935e-02,  2.3110966590e-02,  9.1938472921e-02]),
    'Total Drag':   np.array([ 3.6586579212e-02,  2.7351617523e-02,  2.2997951739e-02,  2.0512203247e-02,  1.8012978407e-02]),
    'Induced Drag': np.array([ 1.1078637930e-02,  7.7852513243e-03,  6.2646857681e-03,  4.7869310241e-03,  3.3107393207e-03]),
    'Wave Drag':    np.array([ 9.4800170926e-06,  2.8489918747e-05,  4.7017947099e-05,  7.6005111330e-05,  1.2185721190e-04]),
    'Profile Drag': np.array([ 0.023290438340032797,0.017328324290169197,0.014475576482664931, 0.013437450527902036,0.012367293114379625]),
}

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    update_regression = False
    vehicle    = CRM_setup()
    results    = run_aero_analysis(vehicle)
    paper_data = pd.read_csv(StringIO(raw_data_paper), sep='\t')
    plot_drag_validation(results, paper_data)
    check_truth_values(results)
    return


# ----------------------------------------------------------------------
#   Regression Check
# ----------------------------------------------------------------------

def check_truth_values(results, tol=1e-2):
    print('\n--- Regression Check ---')
    n = len(next(iter(TRUTH_VALUES.values())))   # number of truth points (5)
    passed = True
    for key, truth in TRUTH_VALUES.items():
        computed = results[key].to_numpy()[:n]
        if not np.allclose(computed, truth, rtol=tol, atol=tol):
            max_err = np.max(np.abs(computed - truth))
            print(f'  FAIL  {key:<14}  max deviation = {max_err:.3e}')
            passed = False
        else:
            print(f'  PASS  {key}')
    if passed:
        print('All checks passed.\n')
    else:
        raise AssertionError('Regression check failed — see output above.')


# ----------------------------------------------------------------------
#   Plotting
# ----------------------------------------------------------------------

def plot_drag_validation(results, paper_data):
    paper_data['Total Drag Counts'] = (paper_data['CD Profile Drag Counts']
                                       + paper_data['CD Induced Drag Counts']
                                       + paper_data['CD Wave Drag Counts'])

    plot_configs = [
        ('Induced Drag',  'CD Induced Drag Counts', 'Induced Drag'),
        ('Wave Drag',     'CD Wave Drag Counts',    'Compressibility Drag'),
        ('Profile Drag',  'CD Profile Drag Counts', 'Profile Drag'),
        ('Total Drag',    'Total Drag Counts',      'Total Drag'),
    ]

    fig, axes = plt.subplots(1, 4, figsize=(20, 5))
    fig.suptitle('CRM Drag Validation — VLM vs. Experiment (Mach 0.85)',
                 fontsize=13, fontweight='bold')

    for ax, (results_key, paper_key, title) in zip(axes, plot_configs):
        ax.plot(results[results_key], results['Lift'],
                color='steelblue', linewidth=1.8, label='RCAIDE VLM')
        ax.plot(paper_data[paper_key] / 1e4, paper_data['Lift'],
                'o--', color='firebrick', markersize=5, linewidth=1.4, label='Experiment')
        ax.set_xlabel('Drag Coefficient $C_D$', fontsize=11)
        ax.set_ylabel('Lift Coefficient $C_L$', fontsize=11)
        ax.set_title(title, fontsize=11)
        ax.legend(fontsize=10)
        ax.grid(True, linestyle='--', alpha=0.5)
        ax.tick_params(labelsize=10)

    fig.tight_layout()

    # ------------------------------------------------------------------
    #   Wind tunnel comparison — raw_data_92 (CL, CD vs AoA; polar)
    # ------------------------------------------------------------------
    exp = pd.read_csv(StringIO(raw_data_92), sep='\t')

    fig2, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 5))
    fig2.suptitle('CRM Aerodynamic Validation — VLM vs. Wind Tunnel (Mach 0.85)',
                  fontsize=13, fontweight='bold')

    alpha_vlm = np.degrees(results['Alpha'])
    alpha_exp = exp['AoA (deg)']

    # CL vs AoA
    ax1.plot(alpha_vlm, results['Lift'],
             color='steelblue', linewidth=1.8, label='RCAIDE VLM')
    ax1.plot(alpha_exp, exp['CL'],
             'o--', color='firebrick', markersize=5, linewidth=1.4, label='Wind Tunnel')
    ax1.set_xlabel('Angle of Attack (deg)', fontsize=11)
    ax1.set_ylabel('$C_L$', fontsize=11)
    ax1.set_title('$C_L$ vs AoA', fontsize=11)
    ax1.legend(fontsize=10)
    ax1.grid(True, linestyle='--', alpha=0.5)

    # CD vs AoA
    ax2.plot(alpha_vlm, results['Total Drag'],
             color='steelblue', linewidth=1.8, label='RCAIDE VLM')
    ax2.plot(alpha_exp, exp['CD'],
             'o--', color='firebrick', markersize=5, linewidth=1.4, label='Wind Tunnel')
    ax2.set_xlabel('Angle of Attack (deg)', fontsize=11)
    ax2.set_ylabel('$C_D$', fontsize=11)
    ax2.set_title('$C_D$ vs AoA', fontsize=11)
    ax2.legend(fontsize=10)
    ax2.grid(True, linestyle='--', alpha=0.5)

    # Drag polar — CL vs CD
    ax3.plot(results['Total Drag'], results['Lift'],
             color='steelblue', linewidth=1.8, label='RCAIDE VLM')
    ax3.plot(exp['CD'], exp['CL'],
             'o--', color='firebrick', markersize=5, linewidth=1.4, label='Wind Tunnel')
    ax3.set_xlabel('$C_D$', fontsize=11)
    ax3.set_ylabel('$C_L$', fontsize=11)
    ax3.set_title('Drag Polar', fontsize=11)
    ax3.legend(fontsize=10)
    ax3.grid(True, linestyle='--', alpha=0.5)

    for ax in (ax1, ax2, ax3):
        ax.tick_params(labelsize=10)

    fig2.tight_layout()
    return fig, fig2


# ----------------------------------------------------------------------
#   Aerodynamic Analysis
# ----------------------------------------------------------------------

def run_aero_analysis(vehicle):

    # Build analysis routine
    analyses = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle

    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.settings.compute_fuel_volume = False
    analyses.append(geometry)

    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.vehicle                               = vehicle 
    analyses.append(aerodynamics)

    # Parse wind tunnel conditions from embedded data
    data        = pd.read_csv(StringIO(raw_data_92), sep='\t')
    AoAs        = np.atleast_2d(np.array(data['AoA (deg)']) * Units.degrees).T
    Machs       = np.atleast_2d(np.array(data['Mach'])).T
    RE_refs     = np.atleast_2d(np.array(data['RE_ref'])).T * 1e7
    Ts          = (np.atleast_2d(np.array(data['Temperature (F)'])).T - 32) * 5/9 + 273.15
    Cref        = 275.80 * Units.inches
    Non_Dim_Res = RE_refs / Cref

    # Run VLM 
    results = aircraft_aerodynamic_analysis(
        analyses                         = analyses,
        angle_of_attacks                 = AoAs,
        mach_numbers                     = Machs,
        non_dimensional_reynolds_numbers = Non_Dim_Res,
        temperatures                     = Ts,
    )
    plot_aircraft_aerodynamics(results) 

    results_data = pd.DataFrame({
        'Mach':         results.freestream.mach_number.flatten(),
        'Alpha':        results.aerodynamics.angles.alpha.flatten(),
        'Lift':         results.aerodynamics.coefficients.lift.total.flatten(),
        'Total Drag':   results.aerodynamics.coefficients.drag.total.flatten(),
        'Parasite Drag':results.aerodynamics.coefficients.drag.parasite.total.flatten(),
        'Wave Drag':    results.aerodynamics.coefficients.drag.compressible.total.flatten(),
        'Induced Drag': results.aerodynamics.coefficients.drag.induced.total.flatten(),
        'Form Drag':    results.aerodynamics.coefficients.drag.form.total.flatten(),
    })
     
    results_data['Profile Drag'] = (results_data['Total Drag']
                                    - results_data['Induced Drag']
                                    - results_data['Wave Drag'])


    # Run VLM - Non-Surrogate
    analyses.aerodynamics.settings.use_surrogate =  False
    results_non_surrogate = aircraft_aerodynamic_analysis(
        analyses                         = analyses,
        angle_of_attacks                 = AoAs,
        mach_numbers                     = Machs,
        non_dimensional_reynolds_numbers = Non_Dim_Res,
        temperatures                     = Ts,
    )
    
    # Plot surface pressure coefficient
    plot_pressure_coefficient_distribution(results_non_surrogate)
    
    return results_data



# ----------------------------------------------------------------------
#   Entry Point
# ----------------------------------------------------------------------

if __name__ == '__main__':
    main()
    plt.show()
