from types import SimpleNamespace

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from RCAIDE.Library.Plots.Emissions.plot_CO2e_emissions import plot_CO2e_emissions


def test_plot_CO2e_emissions_ignores_non_finite_values():
    results = SimpleNamespace(segments=[
        SimpleNamespace(
            tag='mission',
            conditions=SimpleNamespace(
                frames=SimpleNamespace(
                    inertial=SimpleNamespace(time=np.array([[0.], [1.], [2.]]))
                ),
                emissions=SimpleNamespace(
                    cumulative_gCO2e=np.array([[np.nan], [1e6], [3e6]])
                ),
            ),
        ),
    ])

    figure = plot_CO2e_emissions(results, show_legend=False)
    try:
        assert np.all(np.isfinite(figure.axes[0].get_ylim()))
        assert np.allclose(figure.axes[0].get_ylim(), [0, 3.3])
    finally:
        plt.close(figure)


def test_plot_CO2e_emissions_handles_segment_with_no_finite_values():
    results = SimpleNamespace(segments=[
        SimpleNamespace(
            tag='mission',
            conditions=SimpleNamespace(
                frames=SimpleNamespace(
                    inertial=SimpleNamespace(time=np.array([[0.], [1.]]))
                ),
                emissions=SimpleNamespace(
                    cumulative_gCO2e=np.array([[np.nan], [np.inf]])
                ),
            ),
        ),
    ])

    figure = plot_CO2e_emissions(results, show_legend=False)
    try:
        assert np.all(np.isfinite(figure.axes[0].get_ylim()))
    finally:
        plt.close(figure)
