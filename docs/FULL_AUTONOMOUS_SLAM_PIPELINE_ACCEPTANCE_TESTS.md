# Full Autonomous SLAM Pipeline Acceptance Tests

M3-B4 acceptance covers completes-without-active-scan, requires-active-scan-then-completes, invalid sensor replay fault, backend rejected fault, motion rejected fault, map quality stuck fault, no forward/backward commands, no hardware access, and no real motion.

Passing these tests proves the fake/replay autonomous loop is wired. It does not prove real hardware readiness or production SLAM readiness.

## M3-B5 Acceptance

Additional acceptance verifies sensor consumed/skipped trace events, no replay advancement during motion command settle phases, fake map build/save after completion, and save failure when a fake map is required but disabled.
