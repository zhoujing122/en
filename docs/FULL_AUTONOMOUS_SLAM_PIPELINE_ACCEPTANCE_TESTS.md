# Full Autonomous SLAM Pipeline Acceptance Tests

M3-B4 acceptance covers completes-without-active-scan, requires-active-scan-then-completes, invalid sensor replay fault, backend rejected fault, motion rejected fault, map quality stuck fault, no forward/backward commands, no hardware access, and no real motion.

Passing these tests proves the fake/replay autonomous loop is wired. It does not prove real hardware readiness or production SLAM readiness.
