## Phase 27: Vessel Geometry - Database Integration

### Overview

This phase implements PostgreSQL database storage for vessel geometry, enabling a vessel library that can be shared between the barge modeller UI and grillex-webapp.

**Key Concepts:**
- **SQLAlchemy ORM**: Object-relational mapping for vessel storage
- **UUID-based IDs**: Consistent identification across systems
- **Version tracking**: Track modifications to vessels
- **Vessel library**: Reusable vessel definitions

**Dependencies:** Phase 21-24

**Directory Structure:**
```
src/grillex/vessel/
└── io/
    └── database.py         # SQLAlchemy models and repository
```

---

### Task 27.1: Create SQLAlchemy Models

**Requirements:** New feature
**Dependencies:** Phase 21-24
**Difficulty:** High

**Description:**
Create SQLAlchemy ORM models for vessel storage.

**Steps:**

1. Update `src/grillex/vessel/io/database.py`:
   ```python
   """
   PostgreSQL database integration for vessel geometry.

   Uses SQLAlchemy ORM for object-relational mapping.

   Schema:
   -------
   vessels
   ├── id (UUID, PK)
   ├── name
   ├── length, beam, depth
   ├── frame_spacing, frame_zero_x
   ├── created_at, updated_at
   └── version

   cargo_segments
   ├── id (UUID, PK)
   ├── vessel_id (FK)
   ├── name
   ├── x_start, x_end, y_start, y_end
   └── support_type

   decks
   ├── id (UUID, PK)
   ├── segment_id (FK)
   ├── z, name
   └── default_thickness, default_material

   ... (similar for other components)
   """

   from datetime import datetime
   from typing import Optional, List, TYPE_CHECKING
   from uuid import UUID, uuid4

   try:
       from sqlalchemy import (
           create_engine, Column, String, Float, Integer, Boolean,
           DateTime, ForeignKey, JSON, Enum as SQLEnum
       )
       from sqlalchemy.dialects.postgresql import UUID as PGUUID
       from sqlalchemy.orm import (
           declarative_base, relationship, sessionmaker, Session
       )
       from sqlalchemy.sql import func
       SQLALCHEMY_AVAILABLE = True
   except ImportError:
       SQLALCHEMY_AVAILABLE = False

   if TYPE_CHECKING:
       from ..geometry import Vessel

   if SQLALCHEMY_AVAILABLE:
       Base = declarative_base()

       class VesselModel(Base):
           """SQLAlchemy model for Vessel."""
           __tablename__ = "vessels"

           id = Column(PGUUID(as_uuid=True), primary_key=True, default=uuid4)
           name = Column(String(255), nullable=False)
           length = Column(Float, nullable=False)
           beam = Column(Float, nullable=False)
           depth = Column(Float, nullable=False)
           frame_spacing = Column(Float, default=2.5)
           frame_zero_x = Column(Float, default=0.0)

           # Metadata
           created_at = Column(DateTime, server_default=func.now())
           updated_at = Column(DateTime, onupdate=func.now())
           version = Column(Integer, default=1)
           description = Column(String(1000), nullable=True)

           # JSON storage for materials and profiles
           materials_json = Column(JSON, default=dict)
           profiles_json = Column(JSON, default=dict)

           # Relationships
           cargo_segments = relationship(
               "CargoSegmentModel",
               back_populates="vessel",
               cascade="all, delete-orphan"
           )

       class CargoSegmentModel(Base):
           """SQLAlchemy model for CargoSegment."""
           __tablename__ = "cargo_segments"

           id = Column(PGUUID(as_uuid=True), primary_key=True, default=uuid4)
           vessel_id = Column(PGUUID(as_uuid=True), ForeignKey("vessels.id"))
           name = Column(String(255), nullable=False)
           x_start = Column(Float, nullable=False)
           x_end = Column(Float, nullable=False)
           y_start = Column(Float, nullable=False)
           y_end = Column(Float, nullable=False)
           support_type = Column(String(50), default="bottom")

           # Relationships
           vessel = relationship("VesselModel", back_populates="cargo_segments")
           decks = relationship(
               "DeckModel",
               back_populates="segment",
               cascade="all, delete-orphan"
           )
           webframes = relationship(
               "WebFrameModel",
               back_populates="segment",
               cascade="all, delete-orphan"
           )
           longitudinal_girders = relationship(
               "LongitudinalGirderModel",
               back_populates="segment",
               cascade="all, delete-orphan"
           )

       class DeckModel(Base):
           """SQLAlchemy model for Deck."""
           __tablename__ = "decks"

           id = Column(PGUUID(as_uuid=True), primary_key=True, default=uuid4)
           segment_id = Column(PGUUID(as_uuid=True), ForeignKey("cargo_segments.id"))
           z = Column(Float, nullable=False)
           name = Column(String(255))
           default_thickness = Column(Float)
           default_material = Column(String(255))

           # Stiffeners stored as JSON array
           stiffeners_json = Column(JSON, default=list)

           segment = relationship("CargoSegmentModel", back_populates="decks")

       class WebFrameModel(Base):
           """SQLAlchemy model for WebFrame."""
           __tablename__ = "webframes"

           id = Column(PGUUID(as_uuid=True), primary_key=True, default=uuid4)
           segment_id = Column(PGUUID(as_uuid=True), ForeignKey("cargo_segments.id"))
           x = Column(Float, nullable=False)
           type = Column(String(50), default="beam")
           section = Column(String(255))

           segment = relationship("CargoSegmentModel", back_populates="webframes")

       class LongitudinalGirderModel(Base):
           """SQLAlchemy model for LongitudinalGirder."""
           __tablename__ = "longitudinal_girders"

           id = Column(PGUUID(as_uuid=True), primary_key=True, default=uuid4)
           segment_id = Column(PGUUID(as_uuid=True), ForeignKey("cargo_segments.id"))
           y = Column(Float, nullable=False)
           z = Column(Float, nullable=False)
           section = Column(String(255))

           segment = relationship(
               "CargoSegmentModel",
               back_populates="longitudinal_girders"
           )

   else:
       # Placeholder when SQLAlchemy not available
       Base = None
       VesselModel = None
       CargoSegmentModel = None
       DeckModel = None
       WebFrameModel = None
       LongitudinalGirderModel = None


   def check_database_available() -> bool:
       """Check if database dependencies are available."""
       return SQLALCHEMY_AVAILABLE
   ```

**Acceptance Criteria:**
- [ ] VesselModel with all vessel attributes
- [ ] CargoSegmentModel with foreign key to vessel
- [ ] DeckModel with segment relationship
- [ ] WebFrameModel with segment relationship
- [ ] LongitudinalGirderModel with segment relationship
- [ ] JSON columns for materials, profiles, stiffeners
- [ ] Version tracking with created_at, updated_at
- [ ] Graceful fallback if SQLAlchemy not installed

---

### Task 27.2: Create Repository Pattern

**Requirements:** New feature
**Dependencies:** Task 27.1
**Difficulty:** Medium

**Description:**
Implement repository pattern for vessel CRUD operations.

**Steps:**

1. Add repository class to `database.py`:
   ```python
   class VesselRepository:
       """
       Repository for vessel database operations.

       Example:
           >>> from grillex.vessel.io.database import VesselRepository
           >>> repo = VesselRepository("postgresql://localhost/vessels")
           >>>
           >>> # Save vessel
           >>> vessel = create_standard_barge_100x24()
           >>> repo.save(vessel)
           >>>
           >>> # Load vessel
           >>> loaded = repo.get_by_name("Standard Barge 100x24")
           >>>
           >>> # List all vessels
           >>> vessels = repo.list_all()
       """

       def __init__(self, connection_string: str):
           """
           Initialize repository.

           Args:
               connection_string: PostgreSQL connection string
                   e.g., "postgresql://user:pass@localhost/dbname"
           """
           if not SQLALCHEMY_AVAILABLE:
               raise ImportError(
                   "SQLAlchemy is required for database operations. "
                   "Install with: pip install sqlalchemy psycopg2-binary"
               )

           self.engine = create_engine(connection_string)
           self.SessionLocal = sessionmaker(bind=self.engine)

       def create_tables(self) -> None:
           """Create all tables if they don't exist."""
           Base.metadata.create_all(self.engine)

       def save(self, vessel: "Vessel") -> UUID:
           """
           Save vessel to database.

           Args:
               vessel: Vessel to save

           Returns:
               UUID of saved vessel
           """
           with self.SessionLocal() as session:
               model = self._vessel_to_model(vessel)
               session.merge(model)
               session.commit()
               return model.id

       def get_by_id(self, vessel_id: UUID) -> Optional["Vessel"]:
           """
           Get vessel by ID.

           Args:
               vessel_id: UUID of vessel

           Returns:
               Vessel or None if not found
           """
           with self.SessionLocal() as session:
               model = session.query(VesselModel).filter(
                   VesselModel.id == vessel_id
               ).first()
               if model:
                   return self._model_to_vessel(model)
               return None

       def get_by_name(self, name: str) -> Optional["Vessel"]:
           """
           Get vessel by name.

           Args:
               name: Vessel name

           Returns:
               Vessel or None if not found
           """
           with self.SessionLocal() as session:
               model = session.query(VesselModel).filter(
                   VesselModel.name == name
               ).first()
               if model:
                   return self._model_to_vessel(model)
               return None

       def list_all(self) -> List[dict]:
           """
           List all vessels (summary info only).

           Returns:
               List of vessel summaries
           """
           with self.SessionLocal() as session:
               models = session.query(VesselModel).all()
               return [
                   {
                       "id": str(m.id),
                       "name": m.name,
                       "length": m.length,
                       "beam": m.beam,
                       "depth": m.depth,
                       "version": m.version,
                       "created_at": m.created_at.isoformat() if m.created_at else None,
                       "updated_at": m.updated_at.isoformat() if m.updated_at else None,
                   }
                   for m in models
               ]

       def delete(self, vessel_id: UUID) -> bool:
           """
           Delete vessel by ID.

           Args:
               vessel_id: UUID of vessel to delete

           Returns:
               True if deleted, False if not found
           """
           with self.SessionLocal() as session:
               model = session.query(VesselModel).filter(
                   VesselModel.id == vessel_id
               ).first()
               if model:
                   session.delete(model)
                   session.commit()
                   return True
               return False

       def _vessel_to_model(self, vessel: "Vessel") -> VesselModel:
           """Convert Vessel to SQLAlchemy model."""
           model = VesselModel(
               id=vessel.id,
               name=vessel.name,
               length=vessel.length,
               beam=vessel.beam,
               depth=vessel.depth,
               frame_spacing=vessel.frame_spacing,
               frame_zero_x=vessel.frame_zero_x,
               materials_json=vessel._materials,
           )

           # Add cargo segments
           for seg in vessel.get_cargo_segments():
               seg_model = CargoSegmentModel(
                   id=seg.id,
                   name=seg.name,
                   x_start=seg.x_start,
                   x_end=seg.x_end,
                   y_start=seg.y_start,
                   y_end=seg.y_end,
                   support_type=seg.support_type,
               )

               # Add decks
               for deck in seg.get_decks():
                   deck_model = DeckModel(
                       id=deck.id,
                       z=deck.z,
                       name=deck.name,
                       default_thickness=deck.plate_field.default_thickness,
                       default_material=deck.plate_field.default_material,
                       stiffeners_json=self._stiffeners_to_json(
                           deck.plate_field.get_stiffeners()
                       ),
                   )
                   seg_model.decks.append(deck_model)

               # Add webframes
               for wf in seg.get_webframes():
                   wf_model = WebFrameModel(
                       id=wf.id,
                       x=wf.x,
                       type=wf.type,
                       section=wf.section,
                   )
                   seg_model.webframes.append(wf_model)

               # Add longitudinal girders
               for girder in seg.get_longitudinal_girders():
                   girder_model = LongitudinalGirderModel(
                       id=girder.id,
                       y=girder.y,
                       z=girder.z,
                       section=girder.section,
                   )
                   seg_model.longitudinal_girders.append(girder_model)

               model.cargo_segments.append(seg_model)

           return model

       def _model_to_vessel(self, model: VesselModel) -> "Vessel":
           """Convert SQLAlchemy model to Vessel."""
           from ..geometry import Vessel

           vessel = Vessel(
               name=model.name,
               length=model.length,
               beam=model.beam,
               depth=model.depth,
               frame_spacing=model.frame_spacing,
               frame_zero_x=model.frame_zero_x,
           )
           # Preserve ID
           vessel.id = model.id

           # Load materials
           for name, props in (model.materials_json or {}).items():
               vessel.add_material(name, **props)

           # Load cargo segments
           for seg_model in model.cargo_segments:
               segment = vessel.add_cargo_segment(
                   name=seg_model.name,
                   x_start=seg_model.x_start,
                   x_end=seg_model.x_end,
                   y_start=seg_model.y_start,
                   y_end=seg_model.y_end,
                   support_type=seg_model.support_type,
               )
               segment.id = seg_model.id

               # Load decks
               for deck_model in seg_model.decks:
                   deck = segment.add_deck(z=deck_model.z, name=deck_model.name)
                   deck.id = deck_model.id
                   if deck_model.default_thickness:
                       deck.set_plating(
                           thickness=deck_model.default_thickness,
                           material=deck_model.default_material
                       )
                   # Load stiffeners from JSON
                   for stiff_data in (deck_model.stiffeners_json or []):
                       deck.add_stiffener(
                           y=stiff_data.get("position"),
                           section=stiff_data.get("section")
                       )

               # Load webframes
               for wf_model in seg_model.webframes:
                   wf = segment.add_webframe(
                       x=wf_model.x,
                       type=wf_model.type,
                       section=wf_model.section
                   )
                   wf.id = wf_model.id

               # Load longitudinal girders
               for girder_model in seg_model.longitudinal_girders:
                   girder = segment.add_longitudinal_girder(
                       y=girder_model.y,
                       z=girder_model.z,
                       section=girder_model.section
                   )
                   girder.id = girder_model.id

           return vessel

       def _stiffeners_to_json(self, stiffeners) -> list:
           """Convert stiffeners to JSON-serializable list."""
           return [
               {
                   "position": s.position,
                   "section": s.section,
                   "direction": s.direction.value if s.direction else None,
               }
               for s in stiffeners
           ]
   ```

**Acceptance Criteria:**
- [ ] VesselRepository with connection string initialization
- [ ] create_tables() creates schema
- [ ] save() persists vessel to database
- [ ] get_by_id() retrieves vessel by UUID
- [ ] get_by_name() retrieves vessel by name
- [ ] list_all() returns vessel summaries
- [ ] delete() removes vessel
- [ ] Round-trip (save/load) preserves vessel geometry

---

### Task 27.3: Add Version Tracking

**Requirements:** New feature
**Dependencies:** Task 27.2
**Difficulty:** Medium

**Description:**
Implement version tracking for vessel modifications.

**Steps:**

1. Add version methods to repository:
   ```python
   def get_version_history(self, vessel_id: UUID) -> List[dict]:
       """
       Get version history for a vessel.

       Requires a separate version history table or audit log.
       """
       # Placeholder - full implementation would use audit tables
       pass

   def save_as_new_version(self, vessel: "Vessel") -> UUID:
       """
       Save vessel as a new version.

       Increments version number and preserves history.
       """
       with self.SessionLocal() as session:
           # Get current version
           existing = session.query(VesselModel).filter(
               VesselModel.id == vessel.id
           ).first()

           if existing:
               vessel.version = existing.version + 1

           model = self._vessel_to_model(vessel)
           session.merge(model)
           session.commit()
           return model.id
   ```

**Acceptance Criteria:**
- [ ] Version number increments on update
- [ ] updated_at timestamp updated on modification
- [ ] save_as_new_version() preserves history (placeholder)

---

### Task 27.4: Create Database Migration Support

**Requirements:** New feature
**Dependencies:** Task 27.1
**Difficulty:** Medium

**Description:**
Add support for database migrations using Alembic or similar.

**Steps:**

1. Create migration utilities:
   ```python
   def get_migration_sql() -> str:
       """
       Get SQL for creating all tables.

       Useful for manual database setup without SQLAlchemy migrations.
       """
       if not SQLALCHEMY_AVAILABLE:
           raise ImportError("SQLAlchemy required")

       from sqlalchemy import create_engine
       from sqlalchemy.schema import CreateTable

       # Create in-memory engine for schema generation
       engine = create_engine("postgresql://")

       sql_statements = []
       for table in Base.metadata.sorted_tables:
           sql_statements.append(str(CreateTable(table).compile(engine)))

       return "\n\n".join(sql_statements)


   def init_database(connection_string: str) -> None:
       """
       Initialize database with all tables.

       Args:
           connection_string: PostgreSQL connection string
       """
       repo = VesselRepository(connection_string)
       repo.create_tables()
       print("Database tables created successfully")
   ```

**Acceptance Criteria:**
- [ ] get_migration_sql() generates CREATE TABLE statements
- [ ] init_database() initializes schema
- [ ] Alembic compatibility (placeholder)

---

### Task 27.5: Write Database Tests

**Requirements:** Testing
**Dependencies:** Tasks 27.1-27.4
**Difficulty:** Medium

**Description:**
Create tests for database operations.

**Steps:**

1. Create `tests/python/test_phase27_vessel_database.py`:
   ```python
   """Tests for Phase 27: Vessel Database Integration."""

   import pytest
   from uuid import uuid4

   # Skip all tests if SQLAlchemy not available
   try:
       from sqlalchemy import create_engine
       SQLALCHEMY_AVAILABLE = True
   except ImportError:
       SQLALCHEMY_AVAILABLE = False

   from grillex.vessel import Vessel
   from grillex.vessel.io.database import (
       check_database_available,
       VesselRepository,
   )
   from grillex.vessel.reference_vessels import create_standard_barge_100x24


   @pytest.mark.skipif(
       not SQLALCHEMY_AVAILABLE,
       reason="SQLAlchemy not installed"
   )
   class TestDatabaseAvailability:
       """Tests for database availability."""

       def test_check_database_available(self):
           """Database availability check works."""
           result = check_database_available()
           assert isinstance(result, bool)


   @pytest.mark.skipif(
       not SQLALCHEMY_AVAILABLE,
       reason="SQLAlchemy not installed"
   )
   class TestVesselRepository:
       """Tests for VesselRepository."""

       @pytest.fixture
       def repo(self, tmp_path):
           """Create repository with SQLite for testing."""
           db_path = tmp_path / "test.db"
           repo = VesselRepository(f"sqlite:///{db_path}")
           repo.create_tables()
           return repo

       def test_create_tables(self, repo):
           """Tables can be created."""
           # If we got here, tables were created in fixture
           assert True

       def test_save_vessel(self, repo):
           """Vessel can be saved."""
           vessel = Vessel(name="Test", length=100, beam=24, depth=6)
           vessel.add_cargo_segment(x_start=20, x_end=80)

           vessel_id = repo.save(vessel)
           assert vessel_id is not None

       def test_load_vessel_by_id(self, repo):
           """Vessel can be loaded by ID."""
           vessel = Vessel(name="Test", length=100, beam=24, depth=6)
           vessel.add_cargo_segment(x_start=20, x_end=80)

           vessel_id = repo.save(vessel)
           loaded = repo.get_by_id(vessel_id)

           assert loaded is not None
           assert loaded.name == "Test"
           assert loaded.length == 100

       def test_load_vessel_by_name(self, repo):
           """Vessel can be loaded by name."""
           vessel = Vessel(name="MyBarge", length=100, beam=24, depth=6)
           repo.save(vessel)

           loaded = repo.get_by_name("MyBarge")
           assert loaded is not None
           assert loaded.name == "MyBarge"

       def test_list_all_vessels(self, repo):
           """All vessels can be listed."""
           v1 = Vessel(name="Barge1", length=100, beam=24, depth=6)
           v2 = Vessel(name="Barge2", length=120, beam=30, depth=8)

           repo.save(v1)
           repo.save(v2)

           vessels = repo.list_all()
           assert len(vessels) == 2
           names = [v["name"] for v in vessels]
           assert "Barge1" in names
           assert "Barge2" in names

       def test_delete_vessel(self, repo):
           """Vessel can be deleted."""
           vessel = Vessel(name="ToDelete", length=100, beam=24, depth=6)
           vessel_id = repo.save(vessel)

           result = repo.delete(vessel_id)
           assert result is True

           loaded = repo.get_by_id(vessel_id)
           assert loaded is None

       def test_round_trip_preserves_cargo_segment(self, repo):
           """Round-trip preserves cargo segment."""
           vessel = Vessel(name="Test", length=100, beam=24, depth=6)
           segment = vessel.add_cargo_segment(
               name="main",
               x_start=20, x_end=80,
               y_start=-12, y_end=12
           )
           deck = segment.add_deck(z=6.0, name="main_deck")
           deck.set_plating(thickness=0.016, material="S355")

           vessel_id = repo.save(vessel)
           loaded = repo.get_by_id(vessel_id)

           assert len(loaded.get_cargo_segments()) == 1
           loaded_seg = loaded.get_cargo_segment("main")
           assert loaded_seg.x_start == 20
           assert loaded_seg.x_end == 80

       def test_reference_vessel_round_trip(self, repo):
           """Reference vessel survives round-trip."""
           vessel = create_standard_barge_100x24()
           vessel_id = repo.save(vessel)

           loaded = repo.get_by_id(vessel_id)
           assert loaded.name == vessel.name
           assert len(loaded.get_cargo_segments()) == 1
   ```

**Acceptance Criteria:**
- [ ] Tests skip gracefully if SQLAlchemy not installed
- [ ] Create tables test passes
- [ ] Save vessel test passes
- [ ] Load by ID test passes
- [ ] Load by name test passes
- [ ] List all vessels test passes
- [ ] Delete vessel test passes
- [ ] Round-trip preserves cargo segments
- [ ] Reference vessel round-trip works

---

### Summary

| Task | Description | Difficulty | Status |
|------|-------------|------------|--------|
| 27.1 | SQLAlchemy models | High | Pending |
| 27.2 | Repository pattern | Medium | Pending |
| 27.3 | Version tracking | Medium | Pending |
| 27.4 | Migration support | Medium | Pending |
| 27.5 | Database tests | Medium | Pending |

**Total Acceptance Criteria:** 26 items

---

## Integration Notes

### Barge Modeller UI Integration

The barge modeller web UI will use the repository to:
1. Create new vessels
2. Edit existing vessels
3. Browse vessel library
4. Export vessels (YAML, Sesam, Abaqus)

### Grillex-Webapp Integration

The grillex-webapp will use the repository to:
1. Load vessels from library
2. Convert to StructuralModel
3. Run analysis
4. Store analysis results (separate tables)

### Connection String Management

```python
# Environment variable for connection string
import os
DATABASE_URL = os.getenv(
    "VESSEL_DATABASE_URL",
    "postgresql://localhost/vessels"
)

repo = VesselRepository(DATABASE_URL)
```
