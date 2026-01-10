## Phase 22: Vessel Geometry - Profile Library

### Overview

This phase implements a profile library for defining stiffener and girder cross-sections from geometric input parameters. Section properties (A, Iy, Iz, J, etc.) are calculated automatically.

**Key Concepts:**
- **Profile**: Geometric definition of a cross-section (HP, T, L, I, flat bar)
- **ProfileLibrary**: Collection of profiles for a vessel
- **Automatic calculation**: A, Iy, Iz, J, Cw from dimensions
- **Standard profiles**: Lookup tables for common HP series

**Dependencies:** Phase 21 (Core Data Structures)

**Directory Structure:**
```
src/grillex/vessel/
└── profiles/
    ├── __init__.py
    ├── library.py          # ProfileLibrary class
    ├── sections.py         # Profile type classes
    └── standard_hp.py      # Standard HP bulb flat database
```

---

### Task 22.1: Create Profile Base Class and Types

**Requirements:** New feature
**Dependencies:** Phase 21 complete
**Difficulty:** Medium

**Description:**
Create the profile type classes for common structural sections.

**Steps:**

1. Create `src/grillex/vessel/profiles/__init__.py`:
   ```python
   from .library import ProfileLibrary
   from .sections import (
       Profile,
       FlatBar,
       HPProfile,
       AngleProfile,
       TeeProfile,
       IProfile,
   )

   __all__ = [
       "ProfileLibrary",
       "Profile",
       "FlatBar",
       "HPProfile",
       "AngleProfile",
       "TeeProfile",
       "IProfile",
   ]
   ```

2. Create `src/grillex/vessel/profiles/sections.py`:
   ```python
   from dataclasses import dataclass, field
   from typing import Optional
   from abc import ABC, abstractmethod
   import math

   @dataclass
   class SectionProperties:
       """Calculated section properties."""
       A: float      # Area [m²]
       Iy: float     # Second moment about local y-axis [m⁴]
       Iz: float     # Second moment about local z-axis [m⁴]
       J: float      # Torsional constant [m⁴]
       Cw: float     # Warping constant [m⁶]
       y_cg: float   # Centroid y-offset from reference [m]
       z_cg: float   # Centroid z-offset from reference [m]
       y_sc: float   # Shear center y-offset [m]
       z_sc: float   # Shear center z-offset [m]

   @dataclass
   class Profile(ABC):
       """Base class for structural profiles."""
       name: str

       @abstractmethod
       def calculate_properties(self) -> SectionProperties:
           """Calculate section properties from geometry."""
           pass

       @property
       def properties(self) -> SectionProperties:
           """Get section properties (cached)."""
           if not hasattr(self, "_properties"):
               self._properties = self.calculate_properties()
           return self._properties

       @property
       def A(self) -> float:
           """Cross-sectional area [m²]."""
           return self.properties.A

       @property
       def Iy(self) -> float:
           """Second moment of area about y-axis [m⁴]."""
           return self.properties.Iy

       @property
       def Iz(self) -> float:
           """Second moment of area about z-axis [m⁴]."""
           return self.properties.Iz

       @property
       def J(self) -> float:
           """Torsional constant [m⁴]."""
           return self.properties.J


   @dataclass
   class FlatBar(Profile):
       """
       Flat bar profile.

       Reference point: bottom center of bar.

           ┌─────────┐
           │         │ height
           └─────────┘
            thickness
       """
       height: float      # Height in meters
       thickness: float   # Thickness in meters

       def calculate_properties(self) -> SectionProperties:
           h, t = self.height, self.thickness
           A = h * t
           # Centroid at half height
           z_cg = h / 2
           # Second moments about centroid
           Iy = t * h**3 / 12  # About horizontal axis through centroid
           Iz = h * t**3 / 12  # About vertical axis
           # Torsion (thin rectangle)
           J = h * t**3 / 3
           # Warping negligible for flat bar
           Cw = 0.0

           return SectionProperties(
               A=A, Iy=Iy, Iz=Iz, J=J, Cw=Cw,
               y_cg=0.0, z_cg=z_cg,
               y_sc=0.0, z_sc=z_cg
           )


   @dataclass
   class HPProfile(Profile):
       """
       HP Bulb Flat profile (Holland Profile).

       Approximated as flat bar with bulb. The bulb adds area and
       shifts the centroid.

       Reference point: bottom of web (plate attachment point).

           ●───●  <- bulb
           │   │
           │   │  height (total)
           │   │
           └───┘
          thickness

       Note: For accurate HP properties, use standard_hp lookup.
       """
       height: float      # Total height including bulb [m]
       thickness: float   # Web thickness [m]

       def calculate_properties(self) -> SectionProperties:
           h, t = self.height, self.thickness
           # Approximate bulb as adding 40% to web area
           # and 15% extra to Iy
           bulb_factor_area = 1.4
           bulb_factor_iy = 1.15

           A = h * t * bulb_factor_area
           z_cg = h / 2 * 1.05  # Bulb shifts centroid up slightly

           Iy = (t * h**3 / 12) * bulb_factor_iy
           Iz = h * t**3 / 12 * bulb_factor_area
           J = h * t**3 / 3
           Cw = 0.0  # HP has minimal warping resistance

           return SectionProperties(
               A=A, Iy=Iy, Iz=Iz, J=J, Cw=Cw,
               y_cg=0.0, z_cg=z_cg,
               y_sc=0.0, z_sc=z_cg
           )


   @dataclass
   class AngleProfile(Profile):
       """
       Angle (L) profile.

       Reference point: corner (intersection of legs).

             │
             │ height
             │
           ──┴────
            width
       """
       height: float      # Vertical leg height [m]
       width: float       # Horizontal leg width [m]
       thickness: float   # Leg thickness [m]

       def calculate_properties(self) -> SectionProperties:
           h, w, t = self.height, self.width, self.thickness

           # Areas of legs
           A_vert = (h - t) * t
           A_horiz = w * t
           A = A_vert + A_horiz

           # Centroids of each leg
           z_vert = t + (h - t) / 2
           y_vert = t / 2
           z_horiz = t / 2
           y_horiz = w / 2

           # Combined centroid
           y_cg = (A_vert * y_vert + A_horiz * y_horiz) / A
           z_cg = (A_vert * z_vert + A_horiz * z_horiz) / A

           # Second moments about centroid (approximate)
           Iy_vert = t * (h - t)**3 / 12 + A_vert * (z_vert - z_cg)**2
           Iy_horiz = w * t**3 / 12 + A_horiz * (z_horiz - z_cg)**2
           Iy = Iy_vert + Iy_horiz

           Iz_vert = (h - t) * t**3 / 12 + A_vert * (y_vert - y_cg)**2
           Iz_horiz = t * w**3 / 12 + A_horiz * (y_horiz - y_cg)**2
           Iz = Iz_vert + Iz_horiz

           # Torsion (sum of thin rectangles)
           J = ((h - t) * t**3 + w * t**3) / 3

           # Warping (approximate)
           Cw = 0.0  # Angles have minimal warping resistance

           # Shear center (approximate, at corner for equal legs)
           y_sc = t / 2
           z_sc = t / 2

           return SectionProperties(
               A=A, Iy=Iy, Iz=Iz, J=J, Cw=Cw,
               y_cg=y_cg, z_cg=z_cg,
               y_sc=y_sc, z_sc=z_sc
           )


   @dataclass
   class TeeProfile(Profile):
       """
       T-profile (web + flange).

       Reference point: bottom of web (plate attachment point).

           ┌─────────────┐
           │   flange    │ flange_thickness
           └──────┬──────┘
                  │
                  │ web    web_height
                  │
                  └
              web_thickness
       """
       web_height: float        # Web height [m]
       web_thickness: float     # Web thickness [m]
       flange_width: float      # Flange width [m]
       flange_thickness: float  # Flange thickness [m]

       def calculate_properties(self) -> SectionProperties:
           hw, tw = self.web_height, self.web_thickness
           bf, tf = self.flange_width, self.flange_thickness

           # Total height
           h_total = hw + tf

           # Areas
           A_web = hw * tw
           A_flange = bf * tf
           A = A_web + A_flange

           # Centroids (from bottom of web)
           z_web = hw / 2
           z_flange = hw + tf / 2

           # Combined centroid
           z_cg = (A_web * z_web + A_flange * z_flange) / A
           y_cg = 0.0  # Symmetric about y

           # Second moments about centroid
           Iy_web = tw * hw**3 / 12 + A_web * (z_web - z_cg)**2
           Iy_flange = bf * tf**3 / 12 + A_flange * (z_flange - z_cg)**2
           Iy = Iy_web + Iy_flange

           Iz_web = hw * tw**3 / 12
           Iz_flange = tf * bf**3 / 12
           Iz = Iz_web + Iz_flange

           # Torsion
           J = (hw * tw**3 + bf * tf**3) / 3

           # Warping constant (approximate for T)
           # T-sections have significant warping for thin-walled analysis
           Cw = (tf * bf**3 * hw**2) / 12

           # Shear center (at flange-web junction for T)
           z_sc = hw + tf / 2

           return SectionProperties(
               A=A, Iy=Iy, Iz=Iz, J=J, Cw=Cw,
               y_cg=y_cg, z_cg=z_cg,
               y_sc=0.0, z_sc=z_sc
           )


   @dataclass
   class IProfile(Profile):
       """
       I-profile (symmetric I-beam).

       Reference point: bottom of bottom flange.

           ┌─────────────┐
           │   flange    │
           └──────┬──────┘
                  │
                  │ web
                  │
           ┌──────┴──────┐
           │   flange    │
           └─────────────┘
       """
       height: float           # Total height [m]
       flange_width: float     # Flange width [m]
       web_thickness: float    # Web thickness [m]
       flange_thickness: float # Flange thickness [m]

       def calculate_properties(self) -> SectionProperties:
           h = self.height
           bf = self.flange_width
           tw = self.web_thickness
           tf = self.flange_thickness

           hw = h - 2 * tf  # Web height

           # Areas
           A_web = hw * tw
           A_flange = bf * tf
           A = A_web + 2 * A_flange

           # Centroid at mid-height (symmetric)
           z_cg = h / 2
           y_cg = 0.0

           # Second moment about centroid (strong axis)
           Iy_web = tw * hw**3 / 12
           Iy_flange = bf * tf**3 / 12 + A_flange * (h/2 - tf/2)**2
           Iy = Iy_web + 2 * Iy_flange

           # Weak axis
           Iz_web = hw * tw**3 / 12
           Iz_flange = tf * bf**3 / 12
           Iz = Iz_web + 2 * Iz_flange

           # Torsion
           J = (hw * tw**3 + 2 * bf * tf**3) / 3

           # Warping constant
           Cw = (tf * bf**3 * (h - tf)**2) / 24

           # Shear center at centroid (symmetric section)
           z_sc = z_cg

           return SectionProperties(
               A=A, Iy=Iy, Iz=Iz, J=J, Cw=Cw,
               y_cg=y_cg, z_cg=z_cg,
               y_sc=0.0, z_sc=z_sc
           )
   ```

**Acceptance Criteria:**
- [ ] Profile base class with abstract calculate_properties()
- [ ] SectionProperties dataclass with A, Iy, Iz, J, Cw, centroids, shear center
- [ ] FlatBar profile with height and thickness
- [ ] HPProfile with height and thickness (approximated)
- [ ] AngleProfile with height, width, thickness
- [ ] TeeProfile with web and flange dimensions
- [ ] IProfile with height, flange width, web/flange thickness
- [ ] All profiles calculate properties correctly
- [ ] Properties are cached after first calculation

---

### Task 22.2: Create ProfileLibrary Class

**Requirements:** New feature
**Dependencies:** Task 22.1
**Difficulty:** Medium

**Description:**
Create the ProfileLibrary class to manage profile definitions.

**Steps:**

1. Create `src/grillex/vessel/profiles/library.py`:
   ```python
   from typing import Dict, Optional, Union
   from .sections import (
       Profile, FlatBar, HPProfile, AngleProfile, TeeProfile, IProfile
   )
   from .standard_hp import STANDARD_HP_PROFILES

   class ProfileLibrary:
       """
       Library of structural profiles for vessel geometry.

       Profiles can be added manually or looked up from standard databases.

       Example:
           >>> lib = ProfileLibrary()
           >>> lib.add_flat_bar("FB100x10", height=0.100, thickness=0.010)
           >>> lib.add_hp("HP200x10", height=0.200, thickness=0.010)
           >>> lib.add_tee("T400x150x12x20",
           ...             web_height=0.400, web_thickness=0.012,
           ...             flange_width=0.150, flange_thickness=0.020)

           >>> # Lookup from standard database
           >>> lib.add_standard_hp("HP 200x10")

           >>> # Get profile
           >>> profile = lib.get("HP200x10")
           >>> print(f"Area: {profile.A * 1e4:.1f} cm²")
       """

       def __init__(self):
           self._profiles: Dict[str, Profile] = {}

       def add(self, profile: Profile) -> "ProfileLibrary":
           """Add a profile to the library."""
           self._profiles[profile.name] = profile
           return self

       def get(self, name: str) -> Profile:
           """Get profile by name."""
           if name not in self._profiles:
               # Try to auto-resolve from standard HP database
               if self._try_add_standard_hp(name):
                   return self._profiles[name]
               raise KeyError(f"Profile '{name}' not found in library")
           return self._profiles[name]

       def __contains__(self, name: str) -> bool:
           return name in self._profiles

       def __getitem__(self, name: str) -> Profile:
           return self.get(name)

       def list_profiles(self) -> list:
           """List all profile names."""
           return list(self._profiles.keys())

       def add_flat_bar(
           self,
           name: str,
           height: float,
           thickness: float
       ) -> "ProfileLibrary":
           """
           Add a flat bar profile.

           Args:
               name: Profile name (e.g., "FB100x10")
               height: Height in meters
               thickness: Thickness in meters
           """
           profile = FlatBar(name=name, height=height, thickness=thickness)
           return self.add(profile)

       def add_hp(
           self,
           name: str,
           height: float,
           thickness: float
       ) -> "ProfileLibrary":
           """
           Add an HP bulb flat profile.

           Args:
               name: Profile name (e.g., "HP200x10")
               height: Total height in meters
               thickness: Web thickness in meters
           """
           profile = HPProfile(name=name, height=height, thickness=thickness)
           return self.add(profile)

       def add_angle(
           self,
           name: str,
           height: float,
           width: float,
           thickness: float
       ) -> "ProfileLibrary":
           """
           Add an angle (L) profile.

           Args:
               name: Profile name (e.g., "L150x100x10")
               height: Vertical leg height in meters
               width: Horizontal leg width in meters
               thickness: Leg thickness in meters
           """
           profile = AngleProfile(
               name=name, height=height, width=width, thickness=thickness
           )
           return self.add(profile)

       def add_tee(
           self,
           name: str,
           web_height: float,
           web_thickness: float,
           flange_width: float,
           flange_thickness: float
       ) -> "ProfileLibrary":
           """
           Add a T-profile.

           Args:
               name: Profile name (e.g., "T400x150x12x20")
               web_height: Web height in meters
               web_thickness: Web thickness in meters
               flange_width: Flange width in meters
               flange_thickness: Flange thickness in meters
           """
           profile = TeeProfile(
               name=name,
               web_height=web_height,
               web_thickness=web_thickness,
               flange_width=flange_width,
               flange_thickness=flange_thickness
           )
           return self.add(profile)

       def add_i_section(
           self,
           name: str,
           height: float,
           flange_width: float,
           web_thickness: float,
           flange_thickness: float
       ) -> "ProfileLibrary":
           """
           Add an I-profile.

           Args:
               name: Profile name (e.g., "I500x200x12x20")
               height: Total height in meters
               flange_width: Flange width in meters
               web_thickness: Web thickness in meters
               flange_thickness: Flange thickness in meters
           """
           profile = IProfile(
               name=name,
               height=height,
               flange_width=flange_width,
               web_thickness=web_thickness,
               flange_thickness=flange_thickness
           )
           return self.add(profile)

       def add_standard_hp(self, name: str) -> "ProfileLibrary":
           """
           Add a standard HP profile from the database.

           Args:
               name: Standard HP name (e.g., "HP 200x10", "HP200x10")

           Raises:
               KeyError: If profile not found in standard database
           """
           if not self._try_add_standard_hp(name):
               raise KeyError(f"Standard HP profile '{name}' not found")
           return self

       def _try_add_standard_hp(self, name: str) -> bool:
           """Try to add HP from standard database. Returns True if found."""
           # Normalize name (remove spaces, lowercase)
           normalized = name.replace(" ", "").upper()

           for std_name, dims in STANDARD_HP_PROFILES.items():
               if std_name.replace(" ", "").upper() == normalized:
                   profile = HPProfile(
                       name=name,
                       height=dims["height"],
                       thickness=dims["thickness"]
                   )
                   # Override with exact properties if available
                   if "A" in dims:
                       profile._properties = self._props_from_dict(dims)
                   self._profiles[name] = profile
                   return True
           return False

       def _props_from_dict(self, dims: dict):
           """Create SectionProperties from standard profile data."""
           from .sections import SectionProperties
           return SectionProperties(
               A=dims.get("A", 0),
               Iy=dims.get("Iy", 0),
               Iz=dims.get("Iz", 0),
               J=dims.get("J", 0),
               Cw=dims.get("Cw", 0),
               y_cg=dims.get("y_cg", 0),
               z_cg=dims.get("z_cg", 0),
               y_sc=dims.get("y_sc", 0),
               z_sc=dims.get("z_sc", 0),
           )

       def add_common_vessel_profiles(self) -> "ProfileLibrary":
           """Add commonly used vessel profiles."""
           # HP profiles
           for hp in ["HP100x6", "HP120x7", "HP140x8", "HP160x9",
                      "HP180x10", "HP200x10", "HP220x11", "HP240x11",
                      "HP260x12", "HP280x12", "HP300x13", "HP320x13",
                      "HP340x14", "HP370x13", "HP400x15", "HP430x15"]:
               try:
                   self.add_standard_hp(hp)
               except KeyError:
                   pass  # Not in database, skip
           return self
   ```

**Acceptance Criteria:**
- [ ] ProfileLibrary stores profiles by name
- [ ] add_flat_bar(), add_hp(), add_angle(), add_tee(), add_i_section() methods work
- [ ] get() retrieves profile by name
- [ ] add_standard_hp() looks up from standard database
- [ ] Auto-resolve attempts standard HP lookup if profile not found
- [ ] list_profiles() returns all profile names
- [ ] add_common_vessel_profiles() adds typical HP profiles

---

### Task 22.3: Create Standard HP Database

**Requirements:** New feature
**Dependencies:** Task 22.1
**Difficulty:** Low

**Description:**
Create a lookup table for standard HP (Holland Profile) bulb flat sections.

**Steps:**

1. Create `src/grillex/vessel/profiles/standard_hp.py`:
   ```python
   """
   Standard HP (Holland Profile) bulb flat sections.

   Properties are from steel manufacturer catalogs.
   All dimensions in meters, properties in m², m⁴, etc.
   """

   # Standard HP profiles (European)
   # Format: height x thickness in mm
   STANDARD_HP_PROFILES = {
       # Small profiles
       "HP 80x5": {
           "height": 0.080,
           "thickness": 0.005,
           "A": 5.21e-4,
           "Iy": 3.48e-7,
           "Iz": 1.08e-8,
           "z_cg": 0.044,
       },
       "HP 100x6": {
           "height": 0.100,
           "thickness": 0.006,
           "A": 7.97e-4,
           "Iy": 8.08e-7,
           "Iz": 2.39e-8,
           "z_cg": 0.054,
       },
       "HP 120x7": {
           "height": 0.120,
           "thickness": 0.007,
           "A": 11.2e-4,
           "Iy": 1.59e-6,
           "Iz": 4.57e-8,
           "z_cg": 0.064,
       },
       "HP 140x8": {
           "height": 0.140,
           "thickness": 0.008,
           "A": 15.0e-4,
           "Iy": 2.87e-6,
           "Iz": 8.00e-8,
           "z_cg": 0.074,
       },
       "HP 160x9": {
           "height": 0.160,
           "thickness": 0.009,
           "A": 19.3e-4,
           "Iy": 4.78e-6,
           "Iz": 1.31e-7,
           "z_cg": 0.084,
       },
       "HP 180x10": {
           "height": 0.180,
           "thickness": 0.010,
           "A": 24.2e-4,
           "Iy": 7.50e-6,
           "Iz": 2.02e-7,
           "z_cg": 0.094,
       },
       "HP 200x10": {
           "height": 0.200,
           "thickness": 0.010,
           "A": 26.8e-4,
           "Iy": 1.03e-5,
           "Iz": 2.23e-7,
           "z_cg": 0.105,
       },
       "HP 220x11": {
           "height": 0.220,
           "thickness": 0.011,
           "A": 32.4e-4,
           "Iy": 1.46e-5,
           "Iz": 3.26e-7,
           "z_cg": 0.115,
       },
       "HP 240x11": {
           "height": 0.240,
           "thickness": 0.011,
           "A": 35.2e-4,
           "Iy": 1.88e-5,
           "Iz": 3.55e-7,
           "z_cg": 0.126,
       },
       "HP 260x12": {
           "height": 0.260,
           "thickness": 0.012,
           "A": 41.9e-4,
           "Iy": 2.58e-5,
           "Iz": 5.03e-7,
           "z_cg": 0.136,
       },
       "HP 280x12": {
           "height": 0.280,
           "thickness": 0.012,
           "A": 45.0e-4,
           "Iy": 3.20e-5,
           "Iz": 5.40e-7,
           "z_cg": 0.147,
       },
       "HP 300x13": {
           "height": 0.300,
           "thickness": 0.013,
           "A": 52.5e-4,
           "Iy": 4.28e-5,
           "Iz": 7.39e-7,
           "z_cg": 0.157,
       },
       "HP 320x13": {
           "height": 0.320,
           "thickness": 0.013,
           "A": 55.9e-4,
           "Iy": 5.14e-5,
           "Iz": 7.86e-7,
           "z_cg": 0.168,
       },
       "HP 340x14": {
           "height": 0.340,
           "thickness": 0.014,
           "A": 64.0e-4,
           "Iy": 6.56e-5,
           "Iz": 1.05e-6,
           "z_cg": 0.178,
       },
       "HP 370x13": {
           "height": 0.370,
           "thickness": 0.013,
           "A": 64.2e-4,
           "Iy": 7.71e-5,
           "Iz": 9.03e-7,
           "z_cg": 0.194,
       },
       "HP 400x15": {
           "height": 0.400,
           "thickness": 0.015,
           "A": 80.6e-4,
           "Iy": 1.14e-4,
           "Iz": 1.51e-6,
           "z_cg": 0.209,
       },
       "HP 430x15": {
           "height": 0.430,
           "thickness": 0.015,
           "A": 85.7e-4,
           "Iy": 1.39e-4,
           "Iz": 1.61e-6,
           "z_cg": 0.225,
       },
   }


   def get_standard_hp(name: str) -> dict:
       """
       Get standard HP profile by name.

       Args:
           name: Profile name (e.g., "HP 200x10" or "HP200x10")

       Returns:
           Dict with dimensions and properties

       Raises:
           KeyError: If profile not found
       """
       normalized = name.replace(" ", "").upper()
       for std_name, props in STANDARD_HP_PROFILES.items():
           if std_name.replace(" ", "").upper() == normalized:
               return props
       raise KeyError(f"Standard HP profile '{name}' not found")


   def list_standard_hp() -> list:
       """List all available standard HP profiles."""
       return list(STANDARD_HP_PROFILES.keys())
   ```

**Acceptance Criteria:**
- [ ] STANDARD_HP_PROFILES dictionary with common HP sizes
- [ ] Each profile has height, thickness, A, Iy, Iz, z_cg
- [ ] get_standard_hp() looks up by name (case/space insensitive)
- [ ] list_standard_hp() returns all available profiles
- [ ] Covers range from HP 80x5 to HP 430x15

---

### Task 22.4: Integrate ProfileLibrary with Vessel

**Requirements:** New feature
**Dependencies:** Tasks 22.1-22.3, Phase 21
**Difficulty:** Low

**Description:**
Integrate the ProfileLibrary with the Vessel class.

**Steps:**

1. Update `src/grillex/vessel/geometry/vessel.py`:
   ```python
   # Add to imports
   from ..profiles import ProfileLibrary

   # Update Vessel class
   @dataclass
   class Vessel:
       # ... existing fields ...

       _profile_library: ProfileLibrary = field(default_factory=ProfileLibrary)

       @property
       def profiles(self) -> ProfileLibrary:
           """Get the profile library for this vessel."""
           return self._profile_library

       def set_profile_library(self, library: ProfileLibrary) -> "Vessel":
           """Set a custom profile library."""
           self._profile_library = library
           return self
   ```

2. Update `src/grillex/vessel/__init__.py` to export profile classes:
   ```python
   from .profiles import (
       ProfileLibrary,
       Profile,
       FlatBar,
       HPProfile,
       AngleProfile,
       TeeProfile,
       IProfile,
   )

   __all__ = [
       # ... existing exports ...
       "ProfileLibrary",
       "Profile",
       "FlatBar",
       "HPProfile",
       "AngleProfile",
       "TeeProfile",
       "IProfile",
   ]
   ```

**Acceptance Criteria:**
- [ ] Vessel has profiles property returning ProfileLibrary
- [ ] set_profile_library() allows custom library
- [ ] Profile classes exported from grillex.vessel

---

### Task 22.5: Write Profile Tests

**Requirements:** Testing
**Dependencies:** Tasks 22.1-22.4
**Difficulty:** Low

**Description:**
Create comprehensive tests for the profile library.

**Steps:**

1. Create `tests/python/test_phase22_vessel_profiles.py`:
   ```python
   """Tests for Phase 22: Vessel Profile Library."""

   import pytest
   import math
   from grillex.vessel.profiles import (
       ProfileLibrary, FlatBar, HPProfile, AngleProfile, TeeProfile, IProfile
   )
   from grillex.vessel.profiles.standard_hp import (
       get_standard_hp, list_standard_hp, STANDARD_HP_PROFILES
   )


   class TestFlatBar:
       """Tests for FlatBar profile."""

       def test_create_flat_bar(self):
           """FlatBar can be created with dimensions."""
           fb = FlatBar(name="FB100x10", height=0.100, thickness=0.010)
           assert fb.height == 0.100
           assert fb.thickness == 0.010

       def test_flat_bar_area(self):
           """FlatBar area is correct."""
           fb = FlatBar(name="FB100x10", height=0.100, thickness=0.010)
           assert abs(fb.A - 0.001) < 1e-9  # 100mm x 10mm = 1000mm² = 0.001m²

       def test_flat_bar_iy(self):
           """FlatBar Iy is correct."""
           fb = FlatBar(name="FB100x10", height=0.100, thickness=0.010)
           expected_iy = 0.010 * 0.100**3 / 12  # bt³/12
           assert abs(fb.Iy - expected_iy) < 1e-12


   class TestHPProfile:
       """Tests for HP bulb flat profile."""

       def test_create_hp(self):
           """HPProfile can be created."""
           hp = HPProfile(name="HP200x10", height=0.200, thickness=0.010)
           assert hp.height == 0.200

       def test_hp_area_larger_than_flat_bar(self):
           """HP area is larger than equivalent flat bar (bulb effect)."""
           hp = HPProfile(name="HP200x10", height=0.200, thickness=0.010)
           fb = FlatBar(name="FB200x10", height=0.200, thickness=0.010)
           assert hp.A > fb.A


   class TestTeeProfile:
       """Tests for T-profile."""

       def test_create_tee(self):
           """TeeProfile can be created."""
           tee = TeeProfile(
               name="T400x150x12x20",
               web_height=0.400,
               web_thickness=0.012,
               flange_width=0.150,
               flange_thickness=0.020
           )
           assert tee.web_height == 0.400

       def test_tee_area(self):
           """TeeProfile area is sum of web and flange."""
           tee = TeeProfile(
               name="T400x150x12x20",
               web_height=0.400,
               web_thickness=0.012,
               flange_width=0.150,
               flange_thickness=0.020
           )
           expected_a = 0.400 * 0.012 + 0.150 * 0.020
           assert abs(tee.A - expected_a) < 1e-9


   class TestProfileLibrary:
       """Tests for ProfileLibrary."""

       def test_create_library(self):
           """ProfileLibrary can be created."""
           lib = ProfileLibrary()
           assert lib is not None

       def test_add_flat_bar(self):
           """Can add flat bar to library."""
           lib = ProfileLibrary()
           lib.add_flat_bar("FB100x10", height=0.100, thickness=0.010)
           assert "FB100x10" in lib

       def test_get_profile(self):
           """Can retrieve profile from library."""
           lib = ProfileLibrary()
           lib.add_hp("HP200x10", height=0.200, thickness=0.010)
           profile = lib.get("HP200x10")
           assert profile.name == "HP200x10"

       def test_add_standard_hp(self):
           """Can add standard HP from database."""
           lib = ProfileLibrary()
           lib.add_standard_hp("HP 200x10")
           profile = lib.get("HP 200x10")
           # Should have exact properties from database
           assert abs(profile.A - 26.8e-4) < 1e-6

       def test_auto_resolve_standard_hp(self):
           """Library auto-resolves standard HP on get()."""
           lib = ProfileLibrary()
           # Don't add explicitly, should auto-resolve
           profile = lib.get("HP200x10")
           assert profile is not None


   class TestStandardHP:
       """Tests for standard HP database."""

       def test_list_standard_hp(self):
           """Can list standard HP profiles."""
           profiles = list_standard_hp()
           assert len(profiles) > 0
           assert "HP 200x10" in profiles

       def test_get_standard_hp(self):
           """Can get standard HP properties."""
           props = get_standard_hp("HP 200x10")
           assert props["height"] == 0.200
           assert props["thickness"] == 0.010

       def test_get_standard_hp_case_insensitive(self):
           """Standard HP lookup is case insensitive."""
           props1 = get_standard_hp("HP 200x10")
           props2 = get_standard_hp("hp200x10")
           assert props1["A"] == props2["A"]
   ```

**Acceptance Criteria:**
- [ ] FlatBar tests for creation and properties
- [ ] HPProfile tests verify bulb effect on area
- [ ] TeeProfile tests verify area calculation
- [ ] ProfileLibrary tests for add/get operations
- [ ] Standard HP lookup tests
- [ ] All tests pass

---

### Summary

| Task | Description | Difficulty | Status |
|------|-------------|------------|--------|
| 22.1 | Profile base class and types | Medium | Pending |
| 22.2 | ProfileLibrary class | Medium | Pending |
| 22.3 | Standard HP database | Low | Pending |
| 22.4 | Integrate with Vessel | Low | Pending |
| 22.5 | Profile tests | Low | Pending |

**Total Acceptance Criteria:** 28 items
