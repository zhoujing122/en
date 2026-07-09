# Fake Map Save/Load Acceptance Tests

M3-B5 acceptance covers:

- Completed pipeline with good quality builds a fake map artifact.
- Fake map save succeeds when in-memory save is enabled.
- Fake map load succeeds when in-memory load is enabled.
- Bad quality rejects artifact build.
- Save disabled rejects save.
- Duplicate save rejects when overwrite is disabled.
- No filesystem map access is used.
- No real map file is written.

Passing these tests validates the fake map metadata contract only. It does not prove real map storage, real relocalization, or production SLAM readiness.
