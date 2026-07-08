# SLAM Backend Acceptance Tests

A SLAM backend binding must pass these contract checks before it is used by pre-live integration:

1. Backend ready is true when required.
2. Input frame contract passes.
3. Update result is accepted when required.
4. Map quality is valid.
5. save_map passes when explicitly required.
6. map_updated and integrated_scan_count are coherent.
7. No hardware IO is performed by the acceptance runner.
8. No pose writeback is performed.
9. Localizer behavior is not modified.

These tests validate the binding shape, not live mapping performance.
