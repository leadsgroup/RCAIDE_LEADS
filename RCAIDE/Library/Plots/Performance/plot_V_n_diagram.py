# RCAIDE/Library/Plots/Performance/plot_V_n_diagram.py
# 
# 
# Created: Feb 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
from RCAIDE.Framework.Core import Units   
from RCAIDE.Library.Plots.Common import set_axes, plot_style     

# Pacakge imports  
from matplotlib import pyplot as plt  

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------   
def plot_V_n_diagram(V_n_data,
                     vehicle         = None, 
                     save_figure     = False,
                     show_legend     = True,
                     save_filename   = "V_n_Diagram",
                     file_type       =".png",
                     generate_report = False,
                     width           = 8,
                     height          = 6):    
    """ Plot graph, save the final figure, and create results output file

    Source:

    Inputs:
    V_n_data.
        airspeeds.positive                  [kts]
        airspeeds.negative                  [kts]
        Vc                                  [kts]
        Vd                                  [kts]
        Vs1.positive                        [kts]
            negative                        [kts]
        Va.positive                         [kts]
            negative                        [kts]
        load_factors.positive               [Unitless]
            negative                        [Unitless]
        gust_load_factors.positive          [Unitless]
            negative                        [Unitless]
        weight                              [lb]
        altitude                            [ft]
    vehicle._base.tag                       [Unitless]
    Uref_rough                              [ft/s]
    Uref_cruise                             [ft/s]
    Uref_dive                               [ft/s]

    Outputs:

    Properties Used:
    N/A

    Description:
    """

    # Unpack
    load_factors_pos        = V_n_data.load_factors.positive
    load_factors_neg        = V_n_data.load_factors.negative
    airspeeds_pos           = V_n_data.airspeeds.positive
    airspeeds_neg           = V_n_data.airspeeds.negative
    Vc                      = V_n_data.Vc
    Vd                      = V_n_data.Vd
    Vs1_pos                 = V_n_data.Vs1.positive
    Vs1_neg                 = V_n_data.Vs1.negative
    Va_pos                  = V_n_data.Va.positive
    Va_neg                  = V_n_data.Va.negative
    gust_load_factors_pos   = V_n_data.gust_load_factors.positive
    gust_load_factors_neg   = V_n_data.gust_load_factors.negative
    weight                  = V_n_data.weight
    altitude                = V_n_data.altitude 
    Uref_rough              = V_n_data.gust_data.airspeeds.rough_gust   
    Uref_cruise             = V_n_data.gust_data.airspeeds.cruise_gust  
    Uref_dive               = V_n_data.gust_data.airspeeds.dive_gust
    category_tag            = V_n_data.category_tag


    #-----------------------------
    # Plotting the V-n diagram
    #-----------------------------  
    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)


    fig   = plt.figure(save_filename)
    fig.set_size_inches(width,height) 

    ax = fig.add_subplot(1,1,1)
    ax.fill(airspeeds_pos, load_factors_pos, c='b', alpha=0.3)
    ax.fill(airspeeds_neg, load_factors_neg, c='b', alpha=0.3)
    ax.plot(airspeeds_pos, load_factors_pos, c='b')
    ax.plot(airspeeds_neg, load_factors_neg, c='b')

    # Plotting gust lines
    ax.plot([0, Vc,1.05*Vd],[1,gust_load_factors_pos[2],gust_load_factors_pos[len(gust_load_factors_pos)-3]],'--', c='r', label = ('Gust ' + str(round(Uref_cruise)) + 'fps'))
    ax.plot([0, Vd,1.05*Vd],[1,gust_load_factors_pos[3],gust_load_factors_pos[len(gust_load_factors_pos)-2]],'--', c='g', label = ('Gust ' + str(round(Uref_dive)) + 'fps'))
    ax.plot([0, Vc,1.05*Vd],[1,gust_load_factors_neg[2],gust_load_factors_neg[len(gust_load_factors_neg)-3]],'--', c='r')
    ax.plot([0, Vd,1.05*Vd],[1,gust_load_factors_neg[3],gust_load_factors_neg[len(gust_load_factors_neg)-2]],'--', c='g')

    if category_tag == 'commuter':
        ax.plot([0, 1.05*Vd],[1,gust_load_factors_pos[len(gust_load_factors_pos)-1]],'--', c='m', label = ('Gust ' + str(round(Uref_rough)) + 'fps'))
        ax.plot([0, 1.05*Vd],[1,gust_load_factors_neg[len(gust_load_factors_neg)-1]],'--', c='m')

    # Formating the plot
    ax.set_xlabel('Airspeed, KEAS')
    ax.set_ylabel('Load Factor')
    ax.set_title(vehicle.tag + '  Weight=' + str(round(weight)) + 'lb  ' + ' Altitude=' + str(round(altitude)) + 'ft ')
    ax.legend()
    ax.grid() 

    #---------------------------------
    # Creating results output file
    #---------------------------------
    if generate_report: 
        fres = open("V_n_diagram_results_" + vehicle.tag +".dat","w")
        fres.write('V-n diagram summary\n')
        fres.write('-------------------\n')
        fres.write('Aircraft: ' + vehicle.tag + '\n')
        fres.write('category: ' + vehicle.flight_envelope.category + '\n')
        fres.write('FAR certification: Part ' +  vehicle.flight_envelope.FAR_part_number  + '\n')
        fres.write('Weight = ' + str(round(weight)) + ' lb\n')
        fres.write('Altitude = ' + str(round(altitude)) + ' ft\n')
        fres.write('---------------------------------------------------------------\n\n')
        fres.write('Airspeeds: \n')
        fres.write('    Positive stall speed (Vs1)   = ' + str(round(Vs1_pos,1)) + ' KEAS\n')
        fres.write('    Negative stall speed (Vs1)   = ' + str(round(Vs1_neg,1)) + ' KEAS\n')
        fres.write('    Positive maneuver speed (Va) = ' + str(round(Va_pos,1))  + ' KEAS\n')
        fres.write('    Negative maneuver speed (Va) = ' + str(round(Va_neg,1))  + ' KEAS\n')
        fres.write('    Cruise speed (Vc)            = ' + str(round(Vc,1))      + ' KEAS\n')
        fres.write('    Dive speed (Vd)              = ' + str(round(Vd,1))      + ' KEAS\n')
        fres.write('Load factors: \n')
        fres.write('    Positive limit load factor (n+) = ' + str(round(max(load_factors_pos),2)) + '\n')
        fres.write('    Negative limit load factor (n-) = ' + str(round(min(load_factors_neg),2)) + '\n')
        fres.write('    Positive load factor at Vd      = ' + str(round(V_n_data.limit_loads.dive.positive,2)) + '\n')
        fres.write('    Negative load factor at Vd      = ' + str(round(V_n_data.limit_loads.dive.negative,2)) + '\n')

    # Adjusting the sub-plots for legend
    fig.tight_layout()   
    if save_figure:
        fig.savefig(save_filename   + file_type)  
    return fig 
