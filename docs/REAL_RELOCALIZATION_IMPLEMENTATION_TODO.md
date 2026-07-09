# Real Relocalization Implementation TODO

Before real relocalization is enabled, the team must define the real map format, the real scan matcher or relocalization backend, the initial pose output format, confidence thresholds, and low-confidence behavior. Low confidence must not write pose.

Future work must decide when pose writeback is allowed, what real vehicle data is required, and how the backend connects to Localizer, startup recovery, and lost recovery without bypassing safety gates. Required acceptance includes no writeback by default, explicit operator/config gating, replay regression, hardware dry-run review, and failure handling for bad maps or low confidence.
