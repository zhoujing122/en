# Real SLAM Backend Implementation TODO

M3-B3 leaves production SLAM unimplemented. Future work needs named owners and explicit acceptance before live use.

Sensor and SLAM owners must define the `SlamBackendInputFrame` field mapping from ToF, IMU, Wheel, and predicted pose. ToF scan integration must be replaced with real map update logic. IMU and Wheel must be used for pose prediction only through an accepted adapter contract.

Map quality must eventually come from the real map backend, not the deterministic skeleton coverage proxy. `save_map` and `load_map` must be specified, implemented, and tested separately before any production use.

Replay golden logs from M3-B2/M3-B2.1 must be used to regression-test the real backend. Required pre-live acceptance includes sensor contract pass, replay contract pass, backend contract pass, replay-to-backend regression pass, STOP/E-STOP/TTL verification, and suspended direction probe readiness.
