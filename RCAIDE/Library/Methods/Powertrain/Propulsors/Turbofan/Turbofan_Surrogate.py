# RCAIDE/Library/Methods/Powertrain/Propulsors/Turbofan/Turbofan_Surrogate.py
#
# Created:  Sep 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Data, Units 
import numpy as np
import pandas as pd
from scipy.interpolate import RBFInterpolator


# ----------------------------------------------------------------------------------------------------------------------
#  Turbofan_Surrogate
# ----------------------------------------------------------------------------------------------------------------------
# Engine_Rating_Codes.png -- string abbreviation -> numeric RC used by the deck.
# 0 ("not a rating" / part-power) has no abbreviation; it's the implicit fallback.
RATING_CODE_ABBREVIATIONS = {
    "MTO": 50,   # Max Takeoff       -- Takeoff, TakeoffOEI, MissedApproach
    "MCO": 45,   # Max Continuous    -- OEI, OEIClimb
    "MCL": 40,   # Max Climb         -- Climb
    "MCR": 35,   # Max Cruise        -- Cruise, Hold, Loiter, PickUpDropOff, Refuel, Turn, Acceleration
    "FID": 20,   # Flight Idle       -- Descent, Landing, Taxi
}


class _RatingCodeInterpolator(Data):
    """Smooth interpolators (scipy RBFInterpolator, thin-plate-spline kernel)
    for one rating code's data. Rated codes (RC>0) are single-valued at fixed
    (altitude, Mach[, ISA]) by definition (one fixed power setting), so they
    interpolate over (altitude_ft, Mach[, ISA_k]) -> (FN_norm, FF_norm)
    directly, and the caller applies throttle as a post-hoc multiplier (same
    convention compute_thrust.py uses for the analytical cycle model).

    RC=0 (the general part-power map) is NOT single-valued at fixed
    (altitude, Mach, ISA) -- see THROTTLE_AXIS note on Turbofan_Surrogate.
    When use_throttle_axis is set, this instead interpolates over
    (altitude_ft, Mach[, ISA_k], throttle_fraction) -> (FN_norm, FF_norm), and
    the caller's throttle is consumed as an interpolation coordinate, not a
    post-hoc multiplier.

    Uses RBFInterpolator rather than a Delaunay-triangulation-based linear
    interpolator (the previous approach): scattered altitude/Mach/throttle
    points from an algorithmically-generated deck aren't a clean simplex
    mesh, and LinearNDInterpolator's triangulation went visibly singular/
    noisy on exactly that kind of data (non-monotonic thrust vs. altitude and
    failed mission-segment convergence, found running a full Boeing 737
    mission through a generate_turbofan_offdesign_deck-built surrogate). RBF
    has no triangulation to go singular and is globally smooth by
    construction. Coordinates are min-max scaled to [0, 1] per axis before
    fitting -- altitude (ft, O(10^4)) and Mach/throttle (O(1)) would otherwise
    make the interpolator's distance metric almost entirely altitude, since
    RBFInterpolator's kernel is isotropic."""

    # Above this many points, fit uses only each query's nearest RBF_NEIGHBORS training
    # points (scipy's neighbors= option) instead of the full dense kernel matrix -- RBF's
    # exact solve is O(n^3), impractical much beyond a few thousand points. Keep this high:
    # a local neighbor subset can be locally coplanar even when the full dataset isn't (e.g.
    # near a Mach=0 boundary where many points share that one value), which makes the
    # thin-plate-spline's degree-1 polynomial term rank-deficient -- seen in practice on a
    # 400-point deck at the previous threshold of 300.
    RBF_NEIGHBORS_THRESHOLD = 2000
    RBF_NEIGHBORS           = 100

    def __defaults__(self):
        self.has_isa_variation  = False
        self.use_throttle_axis  = False
        self._rbf       = {}
        self._col_min   = None
        self._col_range = None

    def _scale(self, points):
        return (points - self._col_min) / self._col_range

    def build(self, df_rc, use_throttle_axis=False):
        self.has_isa_variation = df_rc["ISA k"].nunique() > 1
        self.use_throttle_axis = use_throttle_axis

        cols = ["ALT ft", "XM"]
        if self.has_isa_variation:
            cols.append("ISA k")
        if use_throttle_axis:
            cols.append("throttle_fraction")
        points = df_rc[cols].to_numpy(dtype=float)

        col_max          = points.max(axis=0)
        self._col_min    = points.min(axis=0)
        self._col_range  = np.where(col_max > self._col_min, col_max - self._col_min, 1.0)
        scaled_points    = self._scale(points)

        n_points  = len(scaled_points)
        neighbors = min(self.RBF_NEIGHBORS, n_points - 1) if n_points > self.RBF_NEIGHBORS_THRESHOLD else None

        for col, key in (("FN_norm", "FN"), ("FF_norm", "FF")):
            values          = df_rc[col].to_numpy(dtype=float)
            self._rbf[key] = RBFInterpolator(scaled_points, values, kernel='thin_plate_spline',
                                              neighbors=neighbors)

    def evaluate(self, alt_ft, mach, isa_dev, throttle_fraction=None):
        """alt_ft, mach, isa_dev[, throttle_fraction]: 1-D arrays of equal
        length. throttle_fraction is required iff use_throttle_axis is set.
        Returns (FN_norm, FF_norm)."""
        point = [alt_ft, mach]
        if self.has_isa_variation:
            point.append(isa_dev)
        if self.use_throttle_axis:
            point.append(throttle_fraction)
        query_points = self._scale(np.column_stack(point))

        return self._rbf["FN"](query_points), self._rbf["FF"](query_points)


class Turbofan_Surrogate(Data):
    """
    Table-driven turbofan performance surrogate, built from an engine deck in
    the format of Example_Engine_Deck_for_RCAIDE.xlsx. Assign an instance of
    this class to turbofan.surrogate to have compute_turbofan_performance
    dispatch to compute_turbofan_performance_surrogate instead of the
    analytical cycle model.

    Required deck schema (case-sensitive column names; one row per flight
    condition/rating point -- order doesn't matter, but every column below
    must be present):

        ALT ft   XM    FN lbf    FF lb/h   ISA k   RC
        0        0.0   26338.1   6176.1    0       50
        0        0.2   20335.1   6268.6    0       50
        5000     0.4   18500.0   5900.0    0       0
        5000     0.6   15200.0   5100.0    0       0
        ...

    where, per Engine_Rating_Codes.png:
        ALT ft   pressure altitude [ft]
        XM       flight Mach number
        FN lbf   net thrust [lbf]           (at that ALT/XM/ISA/RC)
        FF lb/h  fuel flow [lb/h]            (at that ALT/XM/ISA/RC)
        ISA k    ISA temperature deviation [K]
        RC       rating code: 50 MTO, 45 MCO, 40 MCL, 35 MCR, 20 FID, 0 = not
                 a rating (general part-power map -- the RC=0 fallback query()
                 uses when rating_code=None or an unrecognized abbreviation)

    A deck must include at least one ALT=0, XM=0, ISA=0 row (the sea-level-
    static reference used to normalize thrust and fuel flow -- see below);
    build() raises with a specific, actionable message via validate_deck()
    if the schema doesn't hold, rather than a bare pandas KeyError.

    Thrust and fuel flow are normalized by that reference row (highest rating
    code present at ALT=0/XM=0/ISA=0 -- typically MTO) before being stored, so
    the same deck can later be "rubberized" to any target engine's actual SLS
    thrust/fuel flow via query()'s target_SLS_thrust_N/target_SLS_fuel_flow_kg_s
    arguments (default: the design_thrust of the turbofan this surrogate is
    attached to).

    Rating-code fallback: rating_code=None, or an abbreviation with no data in
    this deck, falls back to RC=0 -- the dense, general part-power map.

    RC=0 typically has only ISA=0 data (a single ISA level can't support 3-D
    interpolation -- it's a degenerate flat sheet), so it's interpolated in
    (altitude, Mach[, throttle]) and corrected for nonzero ISA deviation using
    a sensitivity factor fit from whichever rated codes in the deck do have
    multiple ISA levels. This is a deck-derived default, not independently
    validated -- override isa_sensitivity_per_K explicitly if you have better
    data.

    THROTTLE AXIS (RC=0 only): unlike the rated codes (each a single fixed
    power setting by definition -- single-valued at fixed altitude/Mach/ISA),
    RC=0 is a full part-power sweep and is genuinely multi-valued at fixed
    (altitude, Mach, ISA): e.g. the SAFAM deck has exactly 12 different FN
    values at ALT=0/XM=0.2/ISA=0/RC=0, ranging 34,609 down to -88 lbf (a
    windmilling/below-idle point). The deck has no explicit throttle/N1
    column to disambiguate these, so one is constructed: within each
    (altitude, Mach, ISA) group, rows are ranked by descending FN and mapped
    to throttle_fraction = 1 - (rank-1)/(group_size-1), i.e. 1.0 at the
    group's highest thrust and 0.0 at its lowest. RC=0 is then interpolated
    over (altitude, Mach[, ISA], throttle_fraction) -> (FN_norm, FF_norm), and
    query()'s throttle argument is used directly as that 4th coordinate for
    RC=0 -- NOT applied as a post-hoc multiplier the way it is for rated
    codes. This is an approximation (throttle position is assumed
    proportional to the sweep's thrust ranking, since the deck doesn't say
    what the actual commanded variable was) but is well-defined and
    order-independent (rank is computed from FN's value, not row position in
    the file). See Example_Engine_Deck_for_RCAIDE.xlsx investigation notes in
    RESEARCH/22_ATI/Engine_Validation/ for the data that motivated this.

    Component-level station data (e.g. compressor exit temperature) doesn't
    exist for a surrogate -- only thrust and fuel flow are produced. Anything
    reading individual converter conditions (e.g. the aeroacoustics noise
    model) is not supported when turbofan.surrogate is set.
    """

    REQUIRED_COLUMNS = ("ALT ft", "XM", "FN lbf", "FF lb/h", "ISA k", "RC")
    MIN_POINTS_2D = 4    # LinearNDInterpolator needs enough points to triangulate
    MIN_POINTS_3D = 5
    MIN_POINTS_4D = 6    # RC=0 with ISA variation: (altitude, Mach, ISA, throttle)

    def __defaults__(self):
        self.tag                           = 'turbofan_surrogate'
        self.deck_path                      = None
        self.sheet_name                     = 'Thrust_Table_Data'
        self.isa_sensitivity_per_K         = None    # d(FN_norm)/d(ISA dev, K); auto-fit from the deck if None
        self._rc_models                     = {}
        self._SLS_reference_thrust_N        = None
        self._SLS_reference_fuel_flow_kg_s  = None

    @classmethod
    def validate_deck(cls, df):
        """Checks df against the schema documented on this class and raises
        ValueError with a specific, actionable message on the first problem
        found, rather than letting a downstream pandas/scipy call fail with a
        confusing KeyError or Qhull error. Called automatically by build();
        safe to call standalone to check a deck before using it."""
        if not isinstance(df, pd.DataFrame):
            raise ValueError(f"expected a pandas DataFrame, got {type(df).__name__}")

        missing = [c for c in cls.REQUIRED_COLUMNS if c not in df.columns]
        if missing:
            raise ValueError(
                f"deck is missing required column(s) {missing}. "
                f"Required columns (case-sensitive): {list(cls.REQUIRED_COLUMNS)}. "
                f"Found: {list(df.columns)}")

        for col in cls.REQUIRED_COLUMNS:
            non_numeric = df[col].apply(lambda v: not isinstance(v, (int, float, np.integer, np.floating)))
            if non_numeric.any():
                bad_rows = df.index[non_numeric].tolist()[:5]
                raise ValueError(
                    f"column '{col}' has non-numeric value(s) at row(s) {bad_rows} "
                    f"(e.g. {df.loc[bad_rows[0], col]!r}) -- every column must be numeric")
            if df[col].isna().any():
                bad_rows = df.index[df[col].isna()].tolist()[:5]
                raise ValueError(f"column '{col}' has missing (NaN) value(s) at row(s) {bad_rows}")

        # RC=0 is expected to have repeated (ALT ft, XM, ISA k) combinations -- that's the
        # part-power sweep the throttle axis is built from (see THROTTLE_AXIS in the class
        # docstring). Rated codes (RC>0) are single-valued by definition, so duplicates there
        # are a real data problem.
        rated = df[df["RC"] != 0]
        duplicates = rated.duplicated(subset=["ALT ft", "XM", "ISA k", "RC"], keep=False)
        if duplicates.any():
            dup_rows = rated.index[duplicates].tolist()[:5]
            raise ValueError(
                f"deck has duplicate (ALT ft, XM, ISA k, RC) combinations at row(s) {dup_rows} "
                f"for a rated code (RC != 0) -- each rated flight-condition combination must "
                f"appear at most once (RC=0 is the exception; see THROTTLE_AXIS docstring)")

        ref = df[(df["ALT ft"] == 0) & (df["XM"] == 0) & (df["ISA k"] == 0)]
        if len(ref) == 0:
            raise ValueError(
                "deck has no sea-level-static reference row (ALT ft=0, XM=0, ISA k=0) -- "
                "this row is required to normalize thrust and fuel flow (see class docstring)")

        for rc, df_rc in df.groupby("RC"):
            has_isa_variation = df_rc["ISA k"].nunique() > 1
            if rc == 0:
                min_points = cls.MIN_POINTS_4D if has_isa_variation else cls.MIN_POINTS_3D
                dims = "(altitude, Mach, ISA, throttle)" if has_isa_variation else "(altitude, Mach, throttle)"
            else:
                min_points = cls.MIN_POINTS_3D if has_isa_variation else cls.MIN_POINTS_2D
                dims = "(altitude, Mach, ISA)" if has_isa_variation else "(altitude, Mach)"
            if len(df_rc) < min_points:
                raise ValueError(
                    f"RC={rc} has only {len(df_rc)} row(s), too few to interpolate over {dims} "
                    f"(need at least {min_points}). Add more points for this rating code, or "
                    f"remove it from the deck so queries fall back to another rating code.")
            if rc == 0:
                sweep_sizes = df_rc.groupby(["ALT ft", "XM", "ISA k"]).size()
                if (sweep_sizes < 2).any():
                    bad = sweep_sizes[sweep_sizes < 2].index.tolist()[:5]
                    raise ValueError(
                        f"RC=0 has only a single row (no throttle sweep) at (ALT ft, XM, ISA k) "
                        f"{bad} -- every RC=0 flight condition needs at least 2 rows at different "
                        f"power settings to build the throttle axis")

    # Tolerance for treating duplicate (ALT ft, XM, ISA k) rows within one rated code as
    # regeneration/precision noise (auto-averaged) vs. a genuine data inconsistency (raised).
    # The SAFAM deck's RC=20 (Flight Idle) has 16 such rows differing by 0.01-0.1% -- this
    # tolerance is set an order of magnitude above that.
    RATED_DUPLICATE_NOISE_TOLERANCE = 0.01   # 1% relative

    @classmethod
    def _resolve_near_duplicate_rated_rows(cls, df):
        """Averages duplicate (ALT ft, XM, ISA k, RC) rows for rated codes
        (RC>0) when they agree within RATED_DUPLICATE_NOISE_TOLERANCE (a data
        quirk, not a real distinct engine state -- see class docstring),
        printing what was collapsed. Raises if any duplicate group disagrees
        by more than that -- that's a real inconsistency, not noise, and
        shouldn't be silently averaged away. Leaves RC=0 untouched entirely:
        its repeated (ALT ft, XM, ISA k) rows are the throttle sweep, not
        duplicates (see THROTTLE_AXIS in the class docstring)."""
        rated = df[df["RC"] != 0]
        group_keys = ["ALT ft", "XM", "ISA k", "RC"]
        dup_mask = rated.duplicated(subset=group_keys, keep=False)
        if not dup_mask.any():
            return df

        resolved_groups = []
        n_collapsed = 0
        for key, group in rated[dup_mask].groupby(group_keys):
            spread = (group["FN lbf"].max() - group["FN lbf"].min()) / abs(group["FN lbf"].mean())
            if abs(spread) > cls.RATED_DUPLICATE_NOISE_TOLERANCE:
                raise ValueError(
                    f"rated code RC={key[3]} has duplicate rows at (ALT ft, XM, ISA k)="
                    f"{key[:3]} whose FN lbf values disagree by {spread*100:.2f}% "
                    f"(more than the {cls.RATED_DUPLICATE_NOISE_TOLERANCE*100:.0f}% noise "
                    f"tolerance) -- this looks like a real data inconsistency, not "
                    f"regeneration noise, and needs to be resolved in the source deck: "
                    f"{group[['FN lbf', 'FF lb/h']].to_dict('records')}")
            averaged = group.iloc[[0]].copy()
            averaged["FN lbf"]  = group["FN lbf"].mean()
            averaged["FF lb/h"] = group["FF lb/h"].mean()
            resolved_groups.append(averaged)
            n_collapsed += len(group)

        print(f"Turbofan_Surrogate: averaged {n_collapsed} near-duplicate rated-code rows "
              f"(within {cls.RATED_DUPLICATE_NOISE_TOLERANCE*100:.0f}% noise tolerance) into "
              f"{len(resolved_groups)} row(s) -- see RATED_DUPLICATE_NOISE_TOLERANCE docstring.")

        unresolved = df.drop(rated[dup_mask].index)
        return pd.concat([unresolved] + resolved_groups, ignore_index=True)

    @staticmethod
    def deck_to_dataframe(deck):
        """Converts a generate_turbofan_offdesign_deck() result (or any Data/
        object with the same fields: altitude_m, mach_number, thrust_N,
        fuel_mass_flow_rate, isa_deviation_k, rating_code, each a 1-D array)
        into the pandas.DataFrame schema this class requires (REQUIRED_COLUMNS
        -- see the class docstring). Used by build()'s deck= argument; exposed
        standalone for callers that just want the DataFrame (e.g. to inspect
        or export it without building a surrogate)."""
        return pd.DataFrame({
            "ALT ft":  deck.altitude_m / Units.ft,
            "XM":      deck.mach_number,
            "FN lbf":  deck.thrust_N / Units.lbf,
            "FF lb/h": deck.fuel_mass_flow_rate / (Units['lbm'] / Units.hour),
            "ISA k":   deck.isa_deviation_k,
            "RC":      deck.rating_code,
        })

    def build(self, deck_path=None, sheet_name=None, dataframe=None, deck=None, save_path=None):
        """Loads (or accepts, via dataframe= or deck=) and normalizes the deck,
        and fits per-rating-code interpolators. Must be called once before
        query(). Returns self, so it can be chained:
            turbofan.surrogate = Turbofan_Surrogate().build(deck_path)
        Pass an in-memory DataFrame directly (matching REQUIRED_COLUMNS) via
        dataframe=, or a generate_turbofan_offdesign_deck() result via deck=
        (converted internally by deck_to_dataframe()), instead of deck_path=
        when the deck isn't coming from a file. save_path=, if given, writes
        the resolved deck (post near-duplicate-row averaging, pre
        normalization -- i.e. the same REQUIRED_COLUMNS schema, reusable as a
        deck_path= later) to that path via pandas.DataFrame.to_excel()."""
        if deck is not None:
            dataframe = self.deck_to_dataframe(deck)

        if dataframe is not None:
            df = dataframe
        else:
            deck_path  = deck_path or self.deck_path
            sheet_name = sheet_name or self.sheet_name
            if deck_path is None:
                raise ValueError("build() needs one of deck_path (or self.deck_path), dataframe=, or deck=")
            df = pd.read_excel(deck_path, sheet_name=sheet_name)

        df = self._resolve_near_duplicate_rated_rows(df)
        self.validate_deck(df)

        if save_path is not None:
            df.to_excel(save_path, sheet_name=sheet_name or self.sheet_name, index=False)

        ref = df[(df["ALT ft"] == 0) & (df["XM"] == 0) & (df["ISA k"] == 0)]
        ref_row = ref.loc[ref["RC"].idxmax()]   # highest-rated code available at that point (typically MTO)
        self._SLS_reference_thrust_N       = ref_row["FN lbf"] * Units.lbf
        self._SLS_reference_fuel_flow_kg_s = ref_row["FF lb/h"] * (Units['lbm'] / Units.hour)

        df = df.copy()
        df["FN_norm"] = (df["FN lbf"] * Units.lbf) / self._SLS_reference_thrust_N
        df["FF_norm"] = (df["FF lb/h"] * (Units['lbm'] / Units.hour)) / self._SLS_reference_fuel_flow_kg_s

        # RC=0 throttle axis -- see THROTTLE_AXIS in the class docstring. Rank (not row
        # position) within each (altitude, Mach, ISA) group, descending FN, mapped to
        # 1.0 (highest thrust in the group) .. 0.0 (lowest).
        is_rc0 = df["RC"] == 0
        if is_rc0.any():
            group_keys   = ["ALT ft", "XM", "ISA k"]
            rank          = df.loc[is_rc0].groupby(group_keys)["FN lbf"].rank(method="first", ascending=False)
            group_size    = df.loc[is_rc0].groupby(group_keys)["FN lbf"].transform("size")
            df.loc[is_rc0, "throttle_fraction"] = 1.0 - (rank - 1.0) / (group_size - 1.0)

        self._rc_models = {}
        for rc, df_rc in df.groupby("RC"):
            model = _RatingCodeInterpolator()
            model.build(df_rc, use_throttle_axis=(rc == 0))
            self._rc_models[int(rc)] = model

        if self.isa_sensitivity_per_K is None:
            self.isa_sensitivity_per_K = self._fit_isa_sensitivity(df)

        return self

    @staticmethod
    def _fit_isa_sensitivity(df):
        """Average d(FN_norm)/d(ISA dev) across rated codes (RC>0) that have at
        least two ISA levels, matched on (altitude, Mach)."""
        slopes = []
        for rc, df_rc in df[df["RC"] > 0].groupby("RC"):
            isa_levels = sorted(df_rc["ISA k"].unique())
            if len(isa_levels) < 2:
                continue
            base = df_rc[df_rc["ISA k"] == isa_levels[0]].set_index(["ALT ft", "XM"])["FN_norm"]
            for isa in isa_levels[1:]:
                other  = df_rc[df_rc["ISA k"] == isa].set_index(["ALT ft", "XM"])["FN_norm"]
                common = base.index.intersection(other.index)
                if len(common) == 0:
                    continue
                ratio = (other[common] / base[common]).mean()
                slopes.append((ratio - 1.0) / (isa - isa_levels[0]))
        return float(np.mean(slopes)) if slopes else 0.0

    def query(self, altitude_m, mach, isa_dev=0.0, rating_code=None, throttle=1.0,
              target_SLS_thrust_N=None, target_SLS_fuel_flow_kg_s=None):
        """
        Vectorized: altitude_m, mach may be scalars or 1-D arrays of equal
        length; isa_dev and throttle may additionally be scalars even when the
        others are arrays. rating_code: str or None, one of
        RATING_CODE_ABBREVIATIONS (None, or an abbreviation not in this deck,
        falls back to RC=0).

        throttle in [0, 1] is used two different ways depending on rc, per
        THROTTLE_AXIS in the class docstring:
          * rated codes (RC>0, single-valued at fixed altitude/Mach/ISA):
            applied as a post-hoc multiplier on the rating's 100%-power value
            (same convention as compute_thrust.py's analytical cycle model).
          * RC=0 (the multi-valued part-power fallback): consumed directly as
            the 4th interpolation coordinate (throttle_fraction), since the
            deck itself has no single "100% RC=0" value to scale.

        Returns (thrust_N, fuel_flow_kg_s), arrays the same length as mach.
        """
        if not self._rc_models:
            raise RuntimeError("Turbofan_Surrogate.build() must be called before query()")

        rc = RATING_CODE_ABBREVIATIONS.get(rating_code, 0)
        if rc not in self._rc_models:
            rc = 0

        altitude_m = np.atleast_1d(np.asarray(altitude_m, dtype=float))
        mach       = np.atleast_1d(np.asarray(mach, dtype=float))
        isa_dev    = np.atleast_1d(np.asarray(isa_dev, dtype=float))
        throttle   = np.atleast_1d(np.asarray(throttle, dtype=float))
        if isa_dev.size == 1 and mach.size > 1:
            isa_dev = np.full(mach.shape, isa_dev[0])
        if throttle.size == 1 and mach.size > 1:
            throttle = np.full(mach.shape, throttle[0])

        alt_ft = altitude_m / Units.ft
        model  = self._rc_models[rc]
        if model.use_throttle_axis:
            FN_norm, FF_norm = model.evaluate(alt_ft, mach, isa_dev, throttle_fraction=throttle)
        else:
            FN_norm, FF_norm = model.evaluate(alt_ft, mach, isa_dev)

        if not model.has_isa_variation:
            FN_norm = FN_norm * (1.0 + self.isa_sensitivity_per_K * isa_dev)

        if not model.use_throttle_axis:
            FN_norm = FN_norm * throttle
            FF_norm = FF_norm * throttle

        FN_ref = target_SLS_thrust_N if target_SLS_thrust_N is not None else self._SLS_reference_thrust_N
        FF_ref = target_SLS_fuel_flow_kg_s if target_SLS_fuel_flow_kg_s is not None else self._SLS_reference_fuel_flow_kg_s

        # thrust may legitimately be negative (windmilling/idle drag -- see RC=20 Flight
        # Idle data), but fuel flow can't be: floors an occasional RBF interpolation
        # overshoot near a sparse region, not a real physical value from the training data
        return FN_norm * FN_ref, np.maximum(FF_norm * FF_ref, 0.0)
