# RCAIDE/Library/Plots/Emissions/plot_contrails_appleman_chart
# 
# 
# Created:  Jul 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Common import set_axes, plot_style 
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------   
## @ingroup Library-Plots-Performance-Emissions 
def plot_contrails_appleman_chart(results,
                    save_figure = False,
                    show_legend = True,
                    save_filename = "Appleman_Chart" ,
                    file_type = ".png",
                    width = 8, height = 6):
    """
    Generate plots showing CO2-equivalent emissions and emission indexes for various fuel species over mission segments.

    Parameters
    ----------
    results : Data
        Mission results data structure containing:
        results.segments[i].conditions.emissions.total with fields:
            - CO2 : array
                Carbon dioxide emissions [kg]
            - NOx : array
                Nitrogen oxide emissions [kg]
            - H2O : array
                Water vapor emissions [kg]
            - Contrails : array
                Contrail formation impact [kg CO2e]
            - Soot : array
                Particulate emissions [kg]
            - SO2 : array
                Sulfur dioxide emissions [kg]

    save_figure : bool, optional
        Save figure to file if True, default False

    show_legend : bool, optional
        Display segment legend if True, default True

    save_filename : str, optional
        Name for saved figure file, default "CO2e_Emissions"

    file_type : str, optional
        File extension for saved figure, default ".png"

    width : float, optional
        Figure width in inches, default 11

    height : float, optional
        Figure height in inches, default 7

    Returns
    -------
    fig : matplotlib.figure.Figure
        Figure showing stacked emissions contributions

    Notes
    -----
    Creates a stacked area plot showing:
        - Individual contributions from each emission type
        - Cumulative total CO2-equivalent impact
        - Breakdown by mission segment
        - Time history of emissions

    Different emission types are distinguished by fill colors
    and segments use different shades from the inferno colormap.

    **Definitions**

    'CO2-equivalent (CO2e)'
        Combined climate impact normalized to CO2
    
    'Global Warming Potential (GWP)'
        Relative impact factor for different emissions
    
    'Contrail Impact'
        Climate forcing from aviation-induced cloudiness
    """
 
    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)
      
    fig   = plt.figure(save_filename)
    fig.set_size_inches(width,height)  

    line_colors   = cm.inferno(np.linspace(0,0.9,len(results.segments)))
    
  
    # always contrains left of line
    always_contrails_line = np.array([[-59.43367168358419, 101.16672500291688],
                                    [-55.971298564928254, 149.10512192276286],
                                    [-53.23976198809942, 202.20977715552448],
                                    [-51.07478707268698, 254.72173608680438],
                                    [-48.994516392486304, 316.4531559911329],
                                    [-46.60133006650334, 405.2829308132073],
                                    [-44.84984249212462, 479.10862209777156],
                                    [-44.453622681134064, 500.4456889511142]])

    maximum_T_for_persistence_line = np.array([[-56.842608797106536, 101.24139540310355],
                                                [-54.4251545910629, 130.12950647532384],
                                                [-52.33321666083305, 163.0428188076071],
                                                [-50.644965581612425, 193.0626531326567],
                                                [-49.120289347800735, 227.11235561778093],
                                                [-47.59607980399021, 262.314782405787],
                                                [-45.83479173958699, 311.93326332983315],
                                                [-44.31361568078405, 354.628398086571],
                                                [-43.59584645898963, 381.73842025434607],
                                                [-41.52140940380353, 457.87889394469727],
                                                [-40.9651149224128, 483.83152490957883],
                                                [-40.64753237661884, 499.40263679850665]]) 
    # no contrails right of line
    no_contrails_line = np.array([[-50.76957181192394, 100.84004200210016],
                                [-48.83514175708787, 122.79780655699457],
                                [-45.85649282464124, 165.53494341383742],
                                [-43.44603896861511, 211.71391902928485],
                                [-41.35970131839927, 258.4599229961499],
                                [-40.39878660599698, 285.0005833625015],
                                [-38.638431921596094, 336.9245128923113],
                                [-36.80200676700503, 400.95438105238594],
                                [-35.366468323416186, 455.17442538793614],
                                [-34.2513125656283, 500.73970365184925]]) 
    

    #  An Appleman chart can be used to identify more and less favorable conditions. Using the vehicle's mission, estimate the
    # time the vehicle will spend at a lower temperature and/or higher pressure than the 0% relative humidity on the Appleman chart.
    # Also use the vehicle's mission, estimate the time the vehicle will spend at temperatures and pressures 
    # between the 0% relative humidity and maximum temperature for persistence on the Appleman chart. 

    # plot bounds of Appleman chart   
    axis = fig.add_subplot(1, 1, 1)
    axis.plot(always_contrails_line[:,0], always_contrails_line[:,1], label = "Always Contrails", linewidth =  ps.line_width, linestyle = '-', color = 'black')
    axis.plot(maximum_T_for_persistence_line[:,0], maximum_T_for_persistence_line[:,1], label = "Maximum T for Persistence", linewidth =  ps.line_width, color = 'black', linestyle = '--')
    axis.plot(no_contrails_line[:,0], no_contrails_line[:,1], label = "No Contrails", linewidth =  ps.line_width, color = 'black', linestyle = ':')
    axis.set_xlabel('Temperature (°C)')
    axis.set_ylabel('Pressure (hPa)') 
    axis.set_ylim(100,500)
    axis.set_xlim(-60,-30)
    axis.invert_yaxis()
    
    # start with 0 time in each region
    always_contrails_time_sec  =0
    persistence_time_sec = 0 
    # get line colors for plots 
    line_colors   = cm.inferno(np.linspace(0,0.9,len(results.segments)))     

    # loop through mission segments and plot conditions on Appleman chart
    for i in range(len(results.segments)): 

        always_contrails_time_stamps = []
        persistence_time_stamps = []
        no_contrails_time_stamps = []

        T = results.segments[i].conditions.freestream.temperature[:,0] - 273.15
        P = results.segments[i].conditions.freestream.pressure[:,0]/100
        t = results.segments[i].conditions.frames.inertial.time[:,0] 
         
        # rediscretize temperature pressure and time to ensure points are captured within appleman chart
        time        = np.linspace(t[0], t[-1], 200) 
        temperature = np.interp(time, t, T)
        pressure    = np.interp(time, t, P)

        segment_tag  =  results.segments[i].tag
        segment_name = segment_tag.replace('_', ' ') 
        
        # if any of the pressure values are above 500 hPa, print a warning that the conditions are outside the bounds of the Appleman chart
        if np.any(pressure < 500): 
            axis.scatter(temperature, pressure, label =segment_name, color = line_colors[i], s = 50)
            
            # compute time spent in each region based in temperature and pressure conditions
            for j in range(len(temperature)):
                if temperature[j] < np.interp(pressure[j], always_contrails_line[:,1], always_contrails_line[:,0]):
                    always_contrails_time_stamps.append(time[j])
                    
                if temperature[j] > np.interp(pressure[j], no_contrails_line[:,1], no_contrails_line[:,0]):
                    no_contrails_time_stamps.append(time[j])

                # compute time spent between 0% relative humidity (always contrails) and maximum temperature for persistence on the Appleman chart
                if temperature[j] > np.interp(pressure[j], always_contrails_line[:,1], always_contrails_line[:,0]) and temperature[j] < np.interp(pressure[j], maximum_T_for_persistence_line[:,1], maximum_T_for_persistence_line[:,0]):
                    persistence_time_stamps.append(time[j])
 
            # convert time stamps into numpy arrays and compute total time spent in each region
            always_contrails_time_stamps = np.array(always_contrails_time_stamps)
            persistence_time_stamps = np.array(persistence_time_stamps)
            no_contrails_time_stamps = np.array(no_contrails_time_stamps) 

            # use differences between time stamps to compute total time spent in each region
            always_contrails_time_sec += np.sum(np.diff(always_contrails_time_stamps))
            persistence_time_sec += np.sum(np.diff(persistence_time_stamps)) 


        else:
            print(segment_name + "Segment has pressure values outside the bounds of the Appleman chart. Time spent in each region may be inaccurate.")

    # add annotations for each region of the Appleman chart
    axis.text(-58, 400, 'Always Contrails', fontsize = 12
                , color = 'black')
    axis.text(-50, 220, 'Maybe Contrails', fontsize = 12
                , color = 'black')
    axis.text(-38, 150, 'No Contrails', fontsize = 12
                , color = 'black')

    
    t_fin = results.segments[-1].conditions.frames.inertial.time[-1,0]    

    no_contrails_time_sec = t_fin - always_contrails_time_sec - persistence_time_sec        
    # percent of flight in each regine 
    always_contrails_time_percent = (always_contrails_time_sec/t_fin)*100
    persistence_time_percent = (persistence_time_sec/t_fin)*100
    no_contrails_time_percent = (no_contrails_time_sec/t_fin)*100 
    print(f"Percent of flight in Always Contrails region: {always_contrails_time_percent:.2f}%")
    print(f"Percent of flight in Persistence region: {persistence_time_percent:.2f}%")
    print(f"Percent of flight in No Contrails region: {no_contrails_time_percent:.2f}%")
    
    always_contrails_time_hr = always_contrails_time_sec/3600
    persistence_time_hr = persistence_time_sec/3600
    no_contrails_time_hr = no_contrails_time_sec/3600

    # time spend in each region 
    print(f"Time spent in Always Contrails region: {always_contrails_time_hr:.2f} hours")
    print(f"Time spent in Persistence region: {persistence_time_hr:.2f} hours")
    print(f"Time spent in No Contrails region: {no_contrails_time_hr:.2f} hours") 
                
    if show_legend:
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4) 
        leg.set_title('Flight Segment', prop={'size': ps.legend_font_size, 'weight': 'heavy'})    
    
    # Adjusting the sub-plots for legend 
    
    # set title of plot 
    title_text    = 'Appleman Chart'     
    fig.tight_layout() 
    fig.suptitle(title_text)
    
    fig.subplots_adjust(top=0.8)
    
    if save_figure:
        plt.savefig(save_filename + file_type)   
    return fig 