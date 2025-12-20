"""Design code modules (DNV, Eurocode, AISC, etc.).

This package provides a pluggable architecture for implementing design code
checks. Each design code is implemented as a separate module that inherits
from the base classes defined here.

Example usage:
    >>> from grillex.design_codes import DesignCode, DesignCheck, CheckResult
    >>> # Implement your own design code
    >>> class MyCode(DesignCode):
    ...     pass

Available base classes:
    - DesignCheck: Abstract base for individual checks (axial, bending, etc.)
    - DesignCode: Abstract base for complete design standards
    - CheckResult: Dataclass for check results

Design codes will be added as submodules:
    - grillex.design_codes.eurocode3 (EN 1993-1-1)
    - grillex.design_codes.dnv (DNV-RP-C201)
    - grillex.design_codes.aisc (AISC 360)
"""

from .base import CheckResult, DesignCheck, DesignCode

__all__ = [
    "CheckResult",
    "DesignCheck",
    "DesignCode",
]
