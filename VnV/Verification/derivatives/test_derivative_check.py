# VnV/Verification/derivatives/test_derivative_check.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
import torch

from RCAIDE.Library.Methods.Utilities.derivative_check import check_derivatives

N = 16
X = np.linspace(-1.0, 1.5, N)[:, None]
Y = np.linspace(0.5, 2.0, N)[:, None]


def smooth_kernel(x, y, xp=np):
    """Two outputs with known derivatives, on RCAIDE's (n_control_points, 1) column shape."""
    return xp.sin(x) * y**2 + xp.exp(0.1 * x * y), xp.tanh(x - y)


def detached_kernel(x, xp=np):
    """Right values, wrong Torch derivative: the gradient is cut on the Torch path."""
    if xp is np:
        return np.sin(x)
    return torch.sin(x.detach()) + 0.0 * x


def offset_kernel(x, xp=np):
    """Right derivative, wrong Torch value."""
    return xp.sin(x) + (1e-6 if xp is torch else 0.0)


def breaking_kernel(x, xp=np):
    """Right values and derivatives, but the Torch path cannot compile as one graph."""
    if xp is torch:
        torch._dynamo.graph_break()
    return x**2


def test_smooth_kernel_passes():
    report = check_derivatives(smooth_kernel, (X, Y))
    assert report.passed
    assert report.value_error <= 1e-15
    assert report.jvp_error <= 1e-8
    assert report.vjp_consistency_error <= 1e-12
    assert report.graph_breaks == 0


def test_jvp_matches_analytic_derivative():
    # d/dx and d/dy of the first output, against the harness's own Torch JVP path
    x, y   = torch.as_tensor(X), torch.as_tensor(Y)
    dx, dy = torch.ones_like(x), torch.zeros_like(y)
    jv     = torch.func.jvp(lambda a, b: smooth_kernel(a, b, xp=torch)[0], (x, y), (dx, dy))[1].numpy()
    analytic = np.cos(X) * Y**2 + 0.1 * Y * np.exp(0.1 * X * Y)
    assert np.allclose(jv, analytic, rtol=1e-14, atol=0)


def test_truncation_and_round_off_visible_over_steps():
    report = check_derivatives(smooth_kernel, (X, Y), steps=(1e-2, 1e-5, 1e-11), check_graph_breaks=False)
    large, middle, small = np.array(report.jvp_errors).T
    assert np.all(middle < large)      # truncation shrinks with the step
    assert np.all(middle < small)      # round-off grows below it


def test_wrong_derivative_is_flagged():
    report = check_derivatives(detached_kernel, (X,), check_graph_breaks=False)
    assert report.value_error <= 1e-15
    assert report.jvp_error > 0.5
    assert not report.passed


def test_wrong_value_is_flagged():
    report = check_derivatives(offset_kernel, (X,), check_graph_breaks=False)
    assert report.value_error > 1e-7
    assert report.jvp_error <= 1e-8
    assert not report.passed


def test_graph_break_is_flagged():
    report = check_derivatives(breaking_kernel, (X,))
    assert report.jvp_error <= 1e-8
    assert report.graph_breaks >= 1
    assert not report.passed
