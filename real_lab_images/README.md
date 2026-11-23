# Physical laboratory records

These are original Scope screenshots supplied by Matías García from physical AERO 2 laboratory runs. They are not simulator plots. They are course-reference runs, and the user confirmed that disturbances were deliberately applied by hand. The runs used the authors' lab model, [lin_feedback_Lars_Matias.slx](../real_hardware/lin_feedback_Lars_Matias.slx), with the unchanged gains of [nlc_setup_solucion.m](../real_hardware/nlc_setup_solucion.m), not the derived controller.

| Original file | Selected documentation/report copy |
|---|---|
| `image.png` | `hardware_manual_disturbance_run_1.png` |
| `image (1).png` | `hardware_manual_disturbance_run_2.png` |

The selected copies under `docs/figures/` and `report/images/` are byte-identical to the originals. No trace extraction, recoloring, cropping or reconstruction was performed.

The yellow profiles match the original 60 s course references; the blue profiles are interpreted as measured angles, with yaw above and pitch below. The nominal lab-ready model is the derived PD/IP cascade with position gains Kq=2.25 s^-2 and Dq=2.25 s^-1 (offline only, not used on the device). The device runs used the script's position gains Kp = 2.5, Kd = 2, Ki = 1 and speed-loop gains kp_PI_motor = 0.8, ki_PI_motor = 10, kw_PI_motor = 0.5; the script is byte-identical to the one used in the lab. Runtime parameter/build logs were not supplied, so the screenshots do not independently establish the exact loaded gains, MATLAB/QUARC version or software revision. No separate integral-enabled comparison was identified. Integral action (Ki = 1) was active in both runs; no run without integral action was made.

Manual disturbance force, torque and application times were not recorded. Event intervals in captions are approximate visual response intervals, not measured force-onset times. Some pitch excursions are clipped by the displayed vertical limits. No angle/speed/voltage MAT or CSV recordings accompany these screenshots; numerical RMS, exact peak errors, settling times and voltage saturation are therefore not reported as experimental measurements.
