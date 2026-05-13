# ─────────────────────────────────────────────────────────────────────────────
# version.py
#
# Single source of truth for the Gantry Master software version.
# This string is displayed in the UI, embedded in CSV/plot titles, and
# referenced in claude.md.
#
# Format: "MAJOR.MINOR"
#   MAJOR  — increment when the protocol between PC ↔ Arduinos changes,
#             or when a breaking config-file change is made.
#   MINOR  — increment for new features, bug fixes, refactors.
#
# To release a new build:
#   1. Change VERSION here.
#   2. Update the matching #define FIRMWARE_VERSION in both .ino files.
#   3. Update the changelog block at the top of claude.md.
# ─────────────────────────────────────────────────────────────────────────────

VERSION = "1.0"
