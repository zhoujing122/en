# Multi-ToF Degradation Policy

M3-C1 defines explicit degradation modes for offline sync checking:

- `RequireAll`: front, left, and right must all be present. This is the default.
- `AllowMissingOne`: at least two ToF frames must be present; one missing ToF is accepted as degraded.
- `AllowAnyTwo`: any two ToF frames are enough; front is not required.
- `AllowFrontOnly`: front must be present; front-only packets are accepted as degraded when the minimum count allows it.

`min_required_tof_count` is enforced in every mode. Degraded acceptance is still explicit and reportable; it is not silent fallback.
