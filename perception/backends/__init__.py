"""Optional, heavyweight model backends.

Nothing here is imported at package import time — the factories in
:mod:`perception.factory` import these modules lazily, only when their backend
name is selected, so the dependency-free core never drags in torch or OpenCV.
"""
