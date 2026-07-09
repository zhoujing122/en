# Fake Relocalization Contract

M3-B6 adds a fake relocalization contract for the fake/replay autonomous SLAM pipeline. It is report-only and is not real relocalization.

The binding accepts a fake map artifact and a replay sensor snapshot, checks map metadata quality, checks the current ToF scan, computes a deterministic confidence, and returns a fake pose candidate for reports. The pose candidate is never written back to localization.

This stage does not read real maps, does not call Localizer, does not call a real scan matcher, does not use startup or lost recovery writeback, and does not change YawCorrectionApply. It only validates the product flow shape: fake map artifact -> scan -> confidence/result.

M3-B7 adds a strict readiness gate: min_map_yaw_coverage_ratio remains at least 0.30, and low confidence or low yaw coverage blocks readiness without pose writeback.
