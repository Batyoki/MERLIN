# Control Simulator: worked example (the MATLAB window)

The Control Simulator (`arm_bench_dev/src/control_simulator/`) lets you type a control law, simulate it in MATLAB on a
mass–spring–damper plant, and tune its parameters automatically. This page walks through one complete session and
documents the `@param` syntax that makes the script the single source of truth for parameters, bounds and tuning ranges.

**What was verified without MATLAB:** the parser (`param_parser.py`, 16 unit tests), the GUI wiring (PyQt5, offscreen),
and every number below, using `python_sim.py`, a Python mirror of `matlab_scripts/run_control_sim.m` (same plant, 10 ms
explicit Euler, |u| ≤ 10, same metrics). **What was not verified:** the MATLAB engine itself and the MATLAB window on a
machine with MATLAB. If your MATLAB run differs from the numbers here, the mirror is wrong; please tell us.

## 1. The plant and the variables a law can use

```
m = 1 kg, b = 0.5 N s/m, k = 2 N/m      x = [position; velocity],  ẋ = A x + B u,  y = position
step 10 ms, explicit Euler, |u| ≤ 10 N
```

A control law is a MATLAB snippet that assigns `u`. It can read `e` (r − y), `integral` (∫e), `derivative` (Δe/dt; note
that it is 100·e at the first sample, which saturates `u` at t = 0), `x`, `r`, `t`, `dt`, and its own parameters.

## 2. Declaring parameters: `@param`

```matlab
% @param min=0 max=50 desc="proportional gain" unit="N/m"
Kp = 8.0;

Ki = 0.5;   % @param min=0 max=10 desc="integral gain"

% @param Kd default=0.4 min=0 max=20          (no assignment needed)

u = Kp*e + Ki*integral + Kd*derivative;
```

| rule | |
|---|---|
| marker | `%` (MATLAB), `#` or `//`, then `@param` |
| options | `min`, `max`, `default`, `step`, `unit`, `desc` (quote values with spaces) |
| applies to | the assignment on the same line; or, for an annotation alone on its line, the next code line; or the explicit name in `@param NAME …` |
| default | the assigned number or constant expression (`2*pi` works), else `default=` |
| missing `min`/`max` | derived from the default (0…10×, or ±10× for negatives) **with a warning** |
| removed from the executable | the annotated assignments are not executed, otherwise they would overwrite the tuned values |
| errors (red text in the GUI) | `min ≥ max`, default outside `[min, max]`, reserved names (`u e dt integral derivative x r t …`), duplicates, an annotation with nothing to apply to, non-numeric values |
| no `@param` anywhere | old behaviour: every free identifier becomes a parameter with range 0…100 |

## 3. A session in the GUI

1. `python3 control_simulator_gui.py` (needs MATLAB Engine for Python; the status bar says *MATLAB engine ready*).
2. **Load script…** and choose `examples/pd_position.m` (or paste it into the box). The parameter table fills with
   `Kp = 8, Ki = 0.5, Kd = 0.4`, ranges `[0, 50]`, `[0, 10]`, `[0, 20]` and the descriptions as tool-tips. The line under the
   box says *3 parameter(s) read from @param* and lists warnings, if any. Changing a number in the script updates the
   table; a value you typed into a spin box is kept as long as you do not change that parameter's default in the script.
3. **Run**: reference `step`, amplitude 1, 10 s. Result with the declared defaults:

   | rise time | settling (2 %) | overshoot | steady-state error | ISE | ITAE | peak u |
   |---|---|---|---|---|---|---|
   | 0.42 s | does not settle | 36.3 % | 0.138 | 0.730 | 7.595 | 10 (saturated) |

   The response overshoots a lot, and the integral term is still winding down at 10 s.
4. **Tune**: objective `itae`, method `differential_evolution`. The tuner searches exactly the declared ranges. One run
   (15 iterations; the optimiser is stochastic, so yours will differ in the last digits):

   | | Kp | Ki | Kd | rise | settling | overshoot | ss. error | ISE | ITAE |
   |---|---|---|---|---|---|---|---|---|---|
   | declared defaults | 8.0 | 0.5 | 0.4 | 0.42 | – | 36.3 % | 0.138 | 0.730 | 7.595 |
   | tuned for ITAE | 45.4 | 6.0 | 9.4 | 0.39 | 0.92 s | 2.5 % | 0.000 | 0.261 | 0.074 |
   | tuned for ISE | 50.0 | 6.0 | 8.1 | 0.34 | 0.94 s | 7.2 % | 0.001 | 0.259 | 0.153 |

5. How to read it, honestly: `Kp` ends at (or near) its **upper bound** of 50 and the control saturates at ±10 during the
   transient, so the result is limited by the actuator and by the range you declared, not by the optimiser. Raise `max` to see
   whether a larger gain helps; the answer on this plant is limited by saturation. The ITAE and ISE objectives pick different
   trade-offs (overshoot vs. speed); the table shows both.
6. **Save results** writes the history (JSON) that the comparison tab reads.

## 4. Second example: feed-forward instead of an integral

`examples/pd_with_feedforward.m` uses `Kp = 6, Kd = 1.2` and a spring feed-forward `Kff*r` (declared with a standalone
`% @param Kff default=2.0 …`; the plant's spring is 2 N/m, so the ideal is 2). It removes the steady-state error without an
integral term (error 0.000, settling 4.8 s, overshoot 38.7 %), which demonstrates that a standalone declaration and the `#`
marker work. It is not tuned.

## 5. Without MATLAB (tests, CI, a quick look)

```python
from control_simulator.python_sim import simulate, PythonSimulator
from control_simulator.parameter_tuner import ParameterTuner, bounds_from_script

text = open("examples/pd_position.m").read()
print(simulate(text)["metrics"])                       # the defaults of the script
tuner = ParameterTuner(PythonSimulator())              # the same tuner the GUI uses
best = tuner.tune_parameters("pid", bounds_from_script(text), objective="itae",
                             max_iterations=15, control_law_text=text)
```

`python_sim.py` evaluates the scalar subset of MATLAB that the examples use (assignments, `+ − * / ^`, `sin cos tan exp log
sqrt abs max min sign pi`, comments). `if/for/while` raise `UnsupportedLaw`; use the MATLAB window for those.

## 6. Writing your own script: checklist

* Put every tunable number in an annotated assignment and give it `min`/`max` (the tuner needs them).
* Keep the final line an assignment to `u`.
* Leave constants that are not meant to be tuned as plain assignments without `@param`: they stay in the executed body.
  Do not assign again to a declared name (that would overwrite the tuned value; the parser warns).
* The simulator's plant is fixed; for your own plant edit `matlab_scripts/run_control_sim.m`.
