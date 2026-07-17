## @ingroup Library-Plots-Common
# RCAIDE/Library/Plots/Common/rcaide_colormap.py
#
# Created: Jun 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
import matplotlib.colors as mcolors

# ----------------------------------------------------------------------------------------------------------------------
#  RCAIDE colormap — parula
# ----------------------------------------------------------------------------------------------------------------------
# Five anchor points sampled from MATLAB's parula palette.  The map runs from
# deep blue (cool) through cyan/green to bright yellow (warm), giving high
# contrast on both white and dark backgrounds while remaining perceptually
# uniform and colorblind-friendly.
_RCAIDE_CMAP = mcolors.LinearSegmentedColormap.from_list(
    "rcaide_parula",
    [
        [0.2081, 0.1663, 0.5292],   # deep blue
        [0.0196, 0.4061, 0.8754],   # royal blue
        [0.0762, 0.6631, 0.7859],   # cyan
        [0.3672, 0.7897, 0.3832],   # green
        [0.9764, 0.8431, 0.1255],   # yellow
    ],
)


def segment_colors(n):
    """
    Return *n* RGBA colours sampled uniformly from the RCAIDE parula colormap.

    Parameters
    ----------
    n : int
        Number of colours to generate (one per mission segment, case, etc.).

    Returns
    -------
    colors : np.ndarray, shape (n, 4)
        Array of RGBA values in [0, 1].  Matches the shape returned by
        ``matplotlib.segment_colors(n)``.

    Examples
    --------
    >>> line_colors = segment_colors(len(results.segments))
    >>> ax.plot(x, y, color=line_colors[i])
    """
    n = max(int(n), 1)
    t = np.linspace(0.0, 1.0, n)
    return _RCAIDE_CMAP(t)
