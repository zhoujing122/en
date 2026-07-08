# SLAM Backend Handoff Checklist

Before implementing RealSlamBackendBinding, assign and document:

1. Owner for RealSlamBackendBinding.
2. ToF scan fields and frame convention.
3. IMU or wheel motion reference source.
4. Map update API and threading model.
5. Map quality score source.
6. save_map output path convention.
7. keyframe and integrated scan counters.
8. update latency measurement.
9. fault code mapping into SlamBackendFault.
10. Joint test time with the pre-live runner.

The implementation target is to replace the replay binding without changing AutonomousSlamCoordinator, AutonomousSlamPolicy, or PreLiveAutonomousSlamRunner.
