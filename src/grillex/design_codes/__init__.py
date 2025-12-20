"""Design code modules (DNV, Eurocode, AISC, etc.).

This package provides a pluggable architecture for implementing design code
checks. Each design code is implemented as a separate module that inherits
from the base classes defined here.

Example usage:
    >>> from grillex.design_codes import Eurocode3
    >>> code = Eurocode3()
    >>> results = code.check_beam(beam, result_case, combination)

Available base classes:
    - DesignCheck: Abstract base for individual checks (axial, bending, etc.)
    - DesignCode: Abstract base for complete design standards
    - CheckResult: Dataclass for check results

Available design codes:
    - Eurocode3: EN 1993-1-1 (basic cross-section resistance checks)

Future design codes:
    - grillex.design_codes.dnv (DNV-RP-C201)
    - grillex.design_codes.aisc (AISC 360)
"""

from .base import CheckResult, DesignCheck, DesignCode
from .eurocode3 import (
    EC3AxialCheck,
    EC3BendingYCheck,
    EC3BendingZCheck,
    EC3CombinedCheck,
    EC3ShearYCheck,
    EC3ShearZCheck,
    Eurocode3,
)

__all__ = [
    # Base classes
    "CheckResult",
    "DesignCheck",
    "DesignCode",
    # Eurocode 3
    "Eurocode3",
    "EC3AxialCheck",
    "EC3BendingYCheck",
    "EC3BendingZCheck",
    "EC3ShearYCheck",
    "EC3ShearZCheck",
    "EC3CombinedCheck",
]
