# sim
A zero dependency quadcoptor simulation

New approach finding MLP controller:
Implement geometric controller that can read perfect states
and use perfect control signal of geometric controller to train MLP
that only has desired state, current timestep and noised simulated sensor
readings as input