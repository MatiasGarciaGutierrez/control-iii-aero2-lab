# Physical laboratory records

These are original Scope screenshots supplied by Matías García from physical AERO 2 laboratory runs. They are not simulator plots. The user confirmed that the derived controller was used and that disturbances were deliberately applied by hand.

| Original file | Selected documentation/report copy |
|---|---|
| `image.png` | `hardware_manual_disturbance_run_1.png` |
| `image (1).png` | `hardware_manual_disturbance_run_2.png` |

The selected copies under `docs/figures/` and `report/images/` are byte-identical to the originals. No trace extraction, recoloring, cropping or reconstruction was performed.

The yellow profiles match the original 60 s course references; the blue profiles are interpreted as measured angles, with yaw above and pitch below. The nominal lab-ready model is the derived PD/IP cascade with position gains Kq=2.25 s^-2 and Dq=2.25 s^-1. Runtime parameter/build logs were not supplied, so the screenshots do not independently establish the exact loaded gains, MATLAB/QUARC version or software revision. No separate integral-enabled comparison was identified.

Manual disturbance force, torque and application times were not recorded. Event intervals in captions are approximate visual response intervals, not measured force-onset times. Some pitch excursions are clipped by the displayed vertical limits. No angle/speed/voltage MAT or CSV recordings accompany these screenshots; numerical RMS, exact peak errors, settling times and voltage saturation are therefore not reported as experimental measurements.
