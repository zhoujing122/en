# Fake Map To Relocalization Pipeline

M3-B6 consumes the M3-B5 fake map artifact. The full fake pipeline first builds and saves fake map metadata in memory. The fake map relocalization runner then loads that artifact from in-memory storage, reads one deterministic replay scan, and calls FakeRelocalizationBinding.

A successful run proves the fake product path is connected: build map, save metadata, load metadata, evaluate current scan, produce report-only relocalization result. It does not prove real localization quality, real map storage, real scan matching, or live readiness.

M3-B7 removes the temporary yaw-threshold relaxation from the runner. The success path now requires the fake map artifact itself to satisfy the strict yaw coverage gate.
