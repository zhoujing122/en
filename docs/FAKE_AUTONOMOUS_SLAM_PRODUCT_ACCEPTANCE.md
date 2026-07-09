# Fake Autonomous SLAM Product Acceptance

M3-B7 defines a fake/replay/shadow product acceptance package for the autonomous SLAM flow. It runs the fake mapping pipeline, fake map artifact build/save/load, fake relocalization, strict relocalization readiness gate, and adapter replacement manifest.

This acceptance does not prove real hardware readiness and does not prove production SLAM readiness. All sensor, map, motion, backend, and relocalization components are fake/replay/shadow.

The gate keeps pose writeback forbidden. Low relocalization confidence or low yaw coverage must block readiness and must not write x/y/yaw/full pose into localization.
