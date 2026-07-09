# Fake Map Artifact Contract

M3-B5 adds a fake map artifact contract for the full autonomous SLAM fake/replay pipeline.

The artifact is metadata only. It is not an occupancy grid, not a navigation map, and not a production map file. It records the completed fake pipeline scenario, accepted deterministic backend update count, active scan command count, final coverage proxy, yaw coverage proxy, and valid scan count.

This stage does not write map files and does not load map files. `FakeMapStorage` stores artifacts in memory only and returns structured results. The contract exists so the pipeline has a save/load shape that can later be replaced by real map storage without changing the coordinator or policy flow.

A fake map artifact may be built only after the pipeline has completed and the final map quality is good when those requirements are enabled. It cannot be used for real navigation or relocalization.

## M3-B6 Relocalization Use

M3-B6 uses the fake map artifact as metadata input to FakeRelocalizationBinding. The artifact remains metadata only and is not loaded as a real map file.
