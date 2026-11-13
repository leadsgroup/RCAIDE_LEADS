# RCAIDE/Library/Methods/Powertrain/Propulsors/Common/solve_offdesign_matching_jax.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  solve_offdesign_matching_jax
# ----------------------------------------------------------------------------------------------------------------------
def solve_offdesign_matching_jax(step, residual, initial_state, robust_states, inputs, state_size, n, tolerance=1e-12,
                                 max_iterations=400, matched_tolerance=1e-9):
    """
    Matched state of a gas turbine at every control point at once on JAX, differentiable through the implicit
    function theorem. Shared by the turbofan, turbojet and turboprop off-design models.

    The state is flat, state_size quantities by n points (quantity-major). A JAX iteration of step finds the matched
    state of every point; points it does not converge are solved by robust_states, the engine's NumPy robust solver
    with continuation, through a callback, and then refined by the JAX iteration. The callback only runs when a point
    needs it. Derivatives come from residual at the matched state (jax.lax.custom_root): the points are independent,
    so its Jacobian is block diagonal and state_size products give every state_size x state_size block.

    Inputs:
       step                flat -> (flat, collapsed): one forward iteration; collapsed marks points with no flow
       residual            flat -> flat: differentiable, zero at the matched state (step(x) - x for a fixed point)
       initial_state       flat starting state (the engine's reference point at every control point)
       robust_states       NumPy function (flat, done, *inputs) -> flat with the matched states of the points not done
       inputs              tuple of the 1-D arrays the engine is solved at, passed concrete to robust_states
       state_size, n       quantities per point and number of points
       tolerance           relative change of every state entry that ends the JAX iteration at a point   [-]
       matched_tolerance   relative residual below which a point counts as matched                       [-]

    Outputs:
       flat                matched state, differentiable with respect to everything step and residual close over
       matched             True where the point is matched
    """
    import jax
    import jax.numpy as jnp

    def iterate(flat):
        """Fixed-point iteration; each point stops when it converges, collapses or leaves the finite range."""
        def condition(carry):
            flat, active, done, i = carry
            return jnp.any(active) & (i < max_iterations)
        def body(carry):
            flat, active, done, i = carry
            new, collapsed = step(flat)
            old       = flat.reshape(state_size, n)
            new       = new.reshape(state_size, n)
            settled   = jnp.all(jnp.abs(new - old) <= tolerance * jnp.maximum(1.0, jnp.abs(old)), axis=0) & (i > 0)
            finite    = jnp.all(jnp.isfinite(new), axis=0)
            failed    = active & (collapsed | ~finite)
            converged = active & settled & ~failed
            new       = jnp.where(active & ~failed, new, old)
            return new.ravel(), active & ~converged & ~failed, done | converged, i + 1
        start = (flat, jnp.ones(n, dtype=bool), jnp.zeros(n, dtype=bool), 0)
        flat, active, done, i = jax.lax.while_loop(condition, body, start)
        return flat, done

    def forward(residual, guess):
        flat, done = iterate(guess)
        def rescue(flat):
            shape  = jax.ShapeDtypeStruct(flat.shape, jnp.float64)
            states = jax.pure_callback(robust_states, shape, flat, done, *inputs)
            return iterate(states)[0]
        return jax.lax.cond(jnp.all(done), lambda flat: flat, rescue, flat)

    def tangent_solve(linear_map, y):
        columns = [linear_map(jnp.zeros((state_size, n)).at[k].set(1.0).ravel()).reshape(state_size, n) for k in range(state_size)]
        blocks  = jnp.stack(columns, axis=-1).transpose(1, 0, 2)
        x       = jnp.linalg.solve(blocks, y.reshape(state_size, n).T[:, :, None])[:, :, 0]
        return x.T.ravel()

    flat    = jax.lax.custom_root(residual, initial_state, forward, tangent_solve)

    # a point is matched where the residual vanishes
    state   = jax.lax.stop_gradient(flat).reshape(state_size, n)
    error   = jax.lax.stop_gradient(residual(flat)).reshape(state_size, n)
    matched = jnp.all(jnp.abs(error) <= matched_tolerance * jnp.maximum(1.0, jnp.abs(state)), axis=0)
    return flat, matched


def robust_states_from_points(solve_point, state_fields, state_size, n):
    """
    robust_states function for solve_offdesign_matching_jax: solve_point(i, inputs) returns the engine's robust NumPy
    result at point i (a Data with the fields in state_fields) or raises the engine's matching error; points it cannot
    solve keep the state of the JAX iteration and stay unmatched.
    """
    from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.Turbofan_OffDesign_Matching import OffDesignMatchingError
    def robust_states(flat, done, *inputs):
        inputs = [np.asarray(value) for value in inputs]
        states = np.array(flat, dtype=np.float64).reshape(state_size, n)
        for i in np.flatnonzero(~np.asarray(done)):
            try:
                result = solve_point(i, inputs)
            except OffDesignMatchingError:
                continue
            states[:, i] = [result[field] for field in state_fields]
        return states.ravel()
    return robust_states
