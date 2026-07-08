# Real Adapter Handoff Checklist

Assign owners before implementing live adapters:

- RealTof adapter owner
- RealImu adapter owner
- RealWheel adapter owner
- RealMap adapter owner
- RealSoftwareMotionPort owner

Information required from software and sensor teams:

- ToF data source, frequency, units, coordinate frame, and timestamp source
- IMU data source, frequency, units, axis convention, and timestamp source
- Wheel odom source, units, direction convention, and timestamp source
- Map backend input/output contract and save behavior
- TTL, STOP, and emergency stop validation plan
- Lifted direction probe test window
- Low-speed in-place scan test window

Do not enter live tests until the real adapters pass contract checks, software
transport shadow acceptance, and the lifted direction probe checklist.
