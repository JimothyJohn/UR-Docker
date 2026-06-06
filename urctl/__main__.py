"""Enable ``python -m urctl ...`` as an alias for the ``urctl`` console script."""

from __future__ import annotations

import sys

from .cli import main

if __name__ == "__main__":
    sys.exit(main())
