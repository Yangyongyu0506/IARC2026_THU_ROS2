"""iarc_main: Core mission nodes.

- FrameTransformerNode: computes & broadcasts arena ← PX4-NED static transform
- OdomTFBroadcasterNode: publishes PX4 odometry → base_link tf transforms
- SetpointSenderNode: receives UDP position commands and publishes PX4 offboard setpoints
- TargetFeedbackNode: transforms AprilTag poses to arena grid and relays via UDP
"""
