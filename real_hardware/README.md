# Professor's hardware files

The four supplied `.m` and `.slx` files in this directory are preserved unchanged. Their SHA-256 hashes are recorded in [source_checksums.json](source_checksums.json).

One added file is not a supplied original: [lin_feedback_Lars_Matias.slx](lin_feedback_Lars_Matias.slx). It is the authors' lab model: the professor's template with its sheets filled in by Matías García and Lars Snoeijink. All physical AERO 2 runs used it with the unchanged gains of `nlc_setup_solucion.m`, which is byte-identical to the script used in the lab. Its SHA-256 is `1775b55f8bf10934f04e6cb877b4c9887f1c6baa3d8d513b8f5ef4954debe69b`. It is not listed in `source_checksums.json`, which covers only the supplied originals.

The two supplied models use the same AERO 2 USB channel settings. The `(1)` version has additional controller options. Neither supplied setup script is used to configure our adapted model: one selects a speed test by default, and the other position-gain assignments do not match our Question 5 gains. The lab runs did use `nlc_setup_solucion.m`, together with `lin_feedback_Lars_Matias.slx`.

Our separate position-control model is [aero2_hardware_cascade.slx](../simulink/aero2_hardware_cascade.slx). On laboratory MATLAB R2021a, open [aero2_hardware_cascade_R2021a.slx](../simulink/aero2_hardware_cascade_R2021a.slx). It copies the `System` interface and `HIL Initialize1` from `lin_feedback_EMPTY.slx` and connects our Question 5 controller.

Read [the hardware notes](../docs/hardware.md) for the port mapping, Scopes, offline results and outstanding laboratory checks. Use the separate offline model to simulate on a computer without QUARC. This hardware port has not been built or run on the device; it was prepared and tested offline only. The device runs used `lin_feedback_Lars_Matias.slx`.
