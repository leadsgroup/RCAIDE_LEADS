# Installing pyoptsparse + IPOPT

`numerics.mission_solver.package = "pyopt"` (used when `type = "optimize"`)
needs `pyoptsparse` installed, plus IPOPT's native library if
`method = "IPOPT"`. Neither is a `pip install RCAIDE_LEADS` dependency --
`pyoptsparse` has no PyPI wheel at all (confirmed: not on PyPI under any
name), so it can't be listed in `pyproject.toml` without either breaking a
plain `pip install RCAIDE_LEADS` for everyone (no Fortran toolchain) or
breaking RCAIDE_LEADS's own PyPI upload (PyPI rejects packages whose
metadata contains a direct git URL dependency). It has to be installed
separately, once, per machine.

Supported method: **IPOPT** only. CONMIN and SLSQP were both tried and
dropped -- see `pyopt_setup.py`'s `Pyoptsparse_Solve` docstring/`ValueError`
for why: CONMIN's pyoptsparse wrapper reports no exit status at all and was
observed reporting false convergence on RCAIDE's mission segments; SLSQP
ran 3.5-6+ hours without converging on a case scipy's SLSQP solves in
minutes and pyopt's IPOPT solves in seconds. SNOPT isn't supported:
commercial license, despite pyoptsparse itself being open source.

## macOS (Homebrew), verified working

```bash
# 1. IPOPT itself -- required, this is the only supported pyopt method
brew install ipopt

# 2. cyipopt -- the Python binding pyoptsparse's IPOPT wrapper uses.
#    Needs pkg-config to find the just-installed ipopt.
export PKG_CONFIG_PATH="/opt/homebrew/opt/ipopt/lib/pkgconfig:$PKG_CONFIG_PATH"
pip install cyipopt

# 3. pyoptsparse itself, built from source (no PyPI wheel).
#    Needs a working Fortran compiler for its own bundled SLSQP/CONMIN
#    extensions, even if you only ever call IPOPT.
export SDKROOT=$(xcrun --show-sdk-path)
pip install git+https://github.com/mdolab/pyoptsparse.git
```

Step 3's `SDKROOT` line matters even if `gfortran --version` already runs
fine: a stale/incomplete Command Line Tools registration can leave gfortran
able to report its version but unable to actually link anything, failing
meson's build-time compiler check with `Compiler gfortran cannot compile
programs.` / `ld: library not found for -lSystem`. Confirm before assuming
you need a full CLT reinstall:

```bash
gfortran -x f90 -o /tmp/t - <<< 'print *, "ok"' && /tmp/t   # fails without SDKROOT
export SDKROOT=$(xcrun --show-sdk-path)
gfortran -x f90 -o /tmp/t - <<< 'print *, "ok"' && /tmp/t   # should print "ok"
```

If that fixes it, `export SDKROOT=...` (put it in your shell profile) is
sufficient -- no `xcode-select --install` / CLT reinstall needed.

## Verify

```python
import pyoptsparse
pyoptsparse.IPOPT()   # raises "No module named 'cyipopt'" if step 2 was skipped/failed
```

## Other platforms

Not verified in this pass (this was done on Apple Silicon macOS 26). The
same three steps apply in shape -- a native IPOPT install findable via
pkg-config, `cyipopt`, then `pyoptsparse` from source with a working
Fortran/C toolchain -- but package manager commands differ (e.g.
`conda install -c conda-forge ipopt` is the standard route on Linux/other
platforms and bundles its own toolchain, likely with less friction than
the Homebrew route above).
