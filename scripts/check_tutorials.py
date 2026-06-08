"""
Static and import-resolution checks for all tutorial .py files and Jupyter notebooks.

Two modes
---------
Default (no flags)
    Fast, no install needed. Checks syntax (ast.parse) and known banned import
    patterns. Suitable for a no-dependency pre-check step.

--check-imports
    Requires RCAIDE to be installed. Extracts every import statement from every
    tutorial file / notebook cell and executes them in a fresh subprocess.
    Catches ModuleNotFoundError, renamed symbols, and other import-time failures
    without running any simulations (all computation lives inside functions).

Exit codes
----------
0  all files pass
1  one or more failures
"""

import ast
import glob
import json
import os
import subprocess
import sys
import tempfile
import textwrap

# ---------------------------------------------------------------------------
# Banned import substrings
# Add a new entry here whenever an import path is renamed or removed.
# ---------------------------------------------------------------------------
BANNED = [
    # Old top-level IO shims — use RCAIDE.Input_Output instead
    ("from RCAIDE.load import",
     "use 'from RCAIDE.Input_Output import load'"),
    ("from RCAIDE.save import",
     "use 'from RCAIDE.Input_Output import save'"),
    ("from RCAIDE.export_rcaide_data import",
     "use 'from RCAIDE.Input_Output import export'"),
    ("from RCAIDE.import_rcaide_data import",
     "use 'from RCAIDE.Input_Output import import_data'"),
    # Case-sensitivity bug — file on disk is _tofl (lowercase)
    ("from RCAIDE.Library.Methods.Performance.estimate_take_off_weight_given_TOFL import",
     "use '...estimate_take_off_weight_given_tofl' (lowercase tofl)"),
]

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SEARCH_DIRS = [
    os.path.join(ROOT, "Tutorials"),
    os.path.join(ROOT, "docs", "source", "tutorials"),
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _all_py_files() -> list[str]:
    paths = []
    for d in SEARCH_DIRS:
        if os.path.isdir(d):
            paths.extend(glob.glob(d + "/**/*.py", recursive=True))
    return paths


def _all_notebooks() -> list[str]:
    paths = []
    for d in SEARCH_DIRS:
        if os.path.isdir(d):
            paths.extend(glob.glob(d + "/**/*.ipynb", recursive=True))
    return paths


def _extract_imports(source: str) -> list[str]:
    """Return lines that are import statements (import … / from … import …)."""
    try:
        tree = ast.parse(source)
    except SyntaxError:
        return []
    lines = source.splitlines()
    imports = []
    for node in ast.walk(tree):
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            # Reconstruct the source line(s); use ast.get_source_segment if
            # available (3.8+), otherwise fall back to the raw line.
            try:
                seg = ast.get_source_segment(source, node)
                if seg:
                    imports.append(seg)
                    continue
            except Exception:
                pass
            if hasattr(node, "lineno"):
                imports.append(lines[node.lineno - 1].strip())
    return imports


# ---------------------------------------------------------------------------
# Mode 1: fast static checks
# ---------------------------------------------------------------------------

def static_check_source(source: str, path: str, cell_label: str = "") -> list[str]:
    errors = []
    label = f"{path}{cell_label}"

    try:
        ast.parse(source)
    except SyntaxError as exc:
        errors.append(f"SYNTAX  {label}:{exc.lineno}: {exc.msg}")
        return errors  # No point checking patterns if syntax is broken

    for lineno, line in enumerate(source.splitlines(), 1):
        stripped = line.strip()
        for pattern, suggestion in BANNED:
            if pattern in stripped:
                errors.append(
                    f"IMPORT  {label}:{lineno}: '{pattern}' — {suggestion}"
                )
    return errors


def run_static_checks() -> list[str]:
    errors = []
    for path in _all_py_files():
        with open(path, encoding="utf-8") as fh:
            errors.extend(static_check_source(fh.read(), path))
    for path in _all_notebooks():
        with open(path, encoding="utf-8") as fh:
            nb = json.load(fh)
        for idx, cell in enumerate(nb.get("cells", [])):
            if cell.get("cell_type") != "code":
                continue
            source = "".join(cell.get("source", []))
            cell_id = cell.get("id", str(idx))
            errors.extend(static_check_source(source, path, f" [cell {cell_id}]"))
    return errors


# ---------------------------------------------------------------------------
# Mode 2: import-resolution checks (requires RCAIDE installed)
# ---------------------------------------------------------------------------

def _collect_all_imports() -> tuple[list[str], set[str]]:
    """
    Collect every unique import statement found across all tutorial files and
    notebooks. Returns (sorted_unique_imports, skipped_non_rcaide_set).
    """
    all_imports: set[str] = set()

    for path in _all_py_files():
        with open(path, encoding="utf-8") as fh:
            all_imports.update(_extract_imports(fh.read()))

    for path in _all_notebooks():
        with open(path, encoding="utf-8") as fh:
            nb = json.load(fh)
        for cell in nb.get("cells", []):
            if cell.get("cell_type") != "code":
                continue
            source = "".join(cell.get("source", []))
            all_imports.update(_extract_imports(source))

    # Filter to RCAIDE-related imports only; third-party and stdlib imports
    # may not be installed in the lint environment, and we only care about
    # RCAIDE API correctness here.
    rcaide_imports = sorted(
        imp for imp in all_imports
        if "RCAIDE" in imp
    )
    return rcaide_imports


def run_import_checks() -> list[str]:
    """
    Execute all collected RCAIDE import statements in a subprocess.
    Returns a list of error strings.
    """
    imports = _collect_all_imports()
    if not imports:
        return []

    script = textwrap.dedent("""\
        import sys
        errors = []
        imports = {imports!r}
        for stmt in imports:
            try:
                exec(stmt, {{}})
            except Exception as exc:
                errors.append(f"{{stmt!r}}  ->  {{type(exc).__name__}}: {{exc}}")
        if errors:
            for e in errors:
                print(e)
            sys.exit(1)
    """).format(imports=imports)

    with tempfile.NamedTemporaryFile(mode="w", suffix=".py",
                                     delete=False, encoding="utf-8") as tmp:
        tmp.write(script)
        tmp_path = tmp.name

    try:
        result = subprocess.run(
            [sys.executable, tmp_path],
            capture_output=True, text=True
        )
    finally:
        os.unlink(tmp_path)

    errors = []
    if result.returncode != 0:
        for line in (result.stdout + result.stderr).splitlines():
            if line.strip():
                errors.append(f"IMPORT-RESOLVE  {line}")
    return errors


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> int:
    check_imports = "--check-imports" in sys.argv

    errors: list[str] = []

    # Always run static checks
    errors.extend(run_static_checks())

    # Optionally run import-resolution checks
    if check_imports:
        print("Running import-resolution checks (requires RCAIDE installed)…")
        errors.extend(run_import_checks())

    if errors:
        print(f"Tutorial check FAILED — {len(errors)} issue(s):\n")
        for err in errors:
            rel = err.replace(ROOT + os.sep, "")
            print(f"  {rel}")
        return 1

    total_py = len(_all_py_files())
    total_nb = len(_all_notebooks())
    mode = " + import resolution" if check_imports else ""
    print(
        f"Tutorial check passed{mode} — "
        f"{total_py} .py files, {total_nb} notebooks, 0 issues."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
