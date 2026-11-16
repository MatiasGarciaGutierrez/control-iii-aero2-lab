# Professor's hardware files

The four supplied `.m` and `.slx` files in this directory are preserved unchanged. Their SHA-256 hashes are recorded in [source_checksums.json](source_checksums.json).

The two models use the same AERO 2 USB channel settings. The `(1)` version has additional controller options. Neither supplied setup script is used to configure our adapted model: one selects a speed test by default, and the other position-gain assignments do not match our Question 5 gains.

Our separate position-control model is [aero2_hardware_cascade.slx](../simulink/aero2_hardware_cascade.slx). On laboratory MATLAB R2021a, open [aero2_hardware_cascade_R2021a.slx](../simulink/aero2_hardware_cascade_R2021a.slx). It copies the `System` interface and `HIL Initialize1` from `lin_feedback_EMPTY.slx` and connects our Question 5 controller.

Read [the hardware notes](../docs/hardware.md) for the port mapping, Scopes, offline results and outstanding laboratory checks. Use the separate offline model to simulate on a computer without QUARC. The hardware model has not been built or run on the device.
