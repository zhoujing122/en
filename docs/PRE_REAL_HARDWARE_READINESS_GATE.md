# Pre Real Hardware Readiness Gate

M3-B7 is a pre-real-hardware gate for product flow shape, not a live readiness proof. It verifies fake/replay sensor input, deterministic backend updates, coordinator progress, active scan commands, fake motion, fake map artifact handling, fake relocalization, no pose writeback, and adapter replacement documentation.

Before real hardware, every fake/replay/shadow adapter listed in the manifest must be replaced or explicitly reviewed. The app runtime must not default-start product acceptance, fake relocalization, fake map load/save, replay file reads, hardware writes, or pose writeback.
