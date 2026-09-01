"""URP <-> URScript conversion, re-exported for library use.

The implementation lives in ``scripts/urp_convert.py`` (it predates the
package and is the source of truth used by the CLI, pre-commit hooks, and the
existing test suite). This module makes those functions importable as
``urctl.urp`` without duplicating the logic, bootstrapping ``scripts/`` onto
the path when the package is run straight from a checkout.

    from urctl.urp import script_to_urp, urp_to_script
"""

from __future__ import annotations

import sys
from pathlib import Path

try:  # scripts/ already on the path
    from urp_convert import script_to_urp, urp_to_script
except ImportError:
    _scripts = Path(__file__).resolve().parent.parent / "scripts"
    if _scripts.is_dir():  # running from a source checkout
        if str(_scripts) not in sys.path:
            sys.path.insert(0, str(_scripts))
        from urp_convert import script_to_urp, urp_to_script
    else:  # installed wheel: the build vendored a copy as urctl._urp_convert
        from urctl._urp_convert import script_to_urp, urp_to_script

__all__ = ["script_to_urp", "urp_to_script"]
