# egn26 Introduction Task - Driverless

Welcome. This exercise gives a basic yet hands-on introduction to a main problem in our autonomous system: estimating the vehicle state from sensor readings.
The task is intentionally focused and intended for you to demonstrate your engineering approach, working with unknown problems and practical solutions.

## 1 Overview

You will implement and evaluate a vehicle state estimation using recorded car data.

Provided files:
- `state_estimator.py` — This is the file you must modify. Implement the `StateEstimator` class and its `predict(meas: CarSignals)` method.
- `simulate.py` — Runs the estimator on the provided datasets and creates plots. You may edit this for development/visualization only.
- `signals.py` — `CarSignals` dataclass: the only sensor fields you may use inside `state_estimator.py`.
- `prepare_data.py` — Converts raw logs to CSV files. No edits or usage required, as it was only used by us to generate the datasets.
- `data/autox_*.csv` — Datasets for your estimator to run on. They have slightly different sensor characteristics and difficulties.

## 2 Task
To control an autonomous vehicle, we first need to know where the vehicle is and how it is moving.
The first part of the *Sense, Think, Act* paradigm is therefore estimating the vehicle's state from available sensor measurements.

**Your task is to estimate the vehicle state and compare your results with our reference data.**

### Requirements

* Implement and improve `StateEstimator.predict(meas: CarSignals)` in `state_estimator.py`.
* The estimator must output `(x, y, yaw, vx)` when called by `simulate.py`.
* `x` and `y` describe the vehicle position in the provided reference frame, `yaw` its heading, and `vx` its longitudinal velocity.
* You may use any measurements available through the fields declared in `signals.py`.
* Do not rely on additional external sensor signals. Vehicle parameters, physical constants, weights, or other relevant vehicle specifications may be used.
* You may modify `simulate.py` to help with debugging, analysis, or visualization. However, the estimator logic itself must remain in `state_estimator.py` for submission.
* Feel free to store previous measurements or other data as state in the estimator class.

### Recommended starting point

A good starting point is a **kinematic bicycle model** using the rear wheel speeds and steering angle.

[Simple Understanding of Kinematic Bicycle Model](https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357)

The recommended approach is intentionally not necessarily the best one. You are encouraged to identify its limitations and improve upon it.

### Optional Challenge

**Want to challenge yourself? Try building an estimator without using the wheel-speed measurements.**

Wheel speeds can become unreliable during periods of high wheel slip. Investigate whether the remaining available measurements can be combined to obtain a robust estimate of the vehicle state.

There is no single expected solution. We are interested in both the quality of the resulting estimate and your reasoning about the assumptions, limitations, and improvements of your approach.


## 3 Data & units

There are two datasets (`data/autox_01.csv` and `data/autox_02.csv`) provided from the same day and track, but one was driven faster, which could make the estimation more challenging.
Feel free to test both.

The datasets contain:
- Wheel speeds (RPM) for all four wheels
- Steering angle (degrees)
- [IMU](https://en.wikipedia.org/wiki/Inertial_measurement_unit) linear accelerations (m/s²)
- IMU yaw rate (rad/s) — see [yaw (rotation)](https://en.wikipedia.org/wiki/Yaw_(rotation)) for definitions
- Ground truth `x`, `y`, `vx`, and `yaw` for validation only

Use the vehicle specifications for modeling the car in your estimator, if you choose to do so.
Be explicit in your code/comments about any unit conversions you apply.

### Vehicle specifications
You can roughly assume the COG to be positioned at the center between the tires.

| Dimension | Value |
|---|---:|
| Wheelbase | 1530 mm |
| Track | 1230 mm |
| Weight | 192 kg |
| Tire radius | 203.2 mm |

## 4 Time & scope

We recommend you spend at most 4 hours.
The goal is not a perfect solution but a thoughtful one you understand and can explain: how you design, justify, and document your choices matters more than squeezing the last bit of numeric accuracy.

Don't expect to hit the reference values exactly, or even nearly.
They were estimated with more data on hand to correct more for an accurate estimation.

## 5 Evaluation

Please submit your results when you are done, but at the latest the day before your scheduled meeting with the core team.

You should be prepared to explain:
- Which signals you used and why
- Any assumptions (initialization, conversions, parameters)
- How you handle sensor disagreement, noise, and drift, if necessary
- Limitations and possible improvements

The resulting comparison to our reference data is considered, but primarily as evidence supporting your explanation.

## 6 How to run

Install the [UV package manager](https://docs.astral.sh/uv/getting-started/installation/) for Python.

Run the simulation from the project root:
```bash
uv run simulate.py
```

By default, the script runs all available `autox_*.csv` datasets and writes one PNG per dataset to the `output/` directory. You can optionally run only specific datasets:
```bash
uv run simulate.py autox_01
uv run simulate.py autox_02
```

The script generates plots comparing your estimated trajectory, velocity, and yaw to the ground truth.
When ground-truth data is available, the simulation also uses the first ground-truth `x`, `y`, and `yaw` values to initialize the estimator for the comparison.

## Tips

- Use the provided vehicle specs for better accuracy.
- Keep in mind some signals are quite unprocessed sensor data.
- Compare your results to the ground truth plots and check when the estimate diverges.
- Document any assumptions or improvements you make.
- If you explore sensor fusion, common references are the [Kalman filter](https://en.wikipedia.org/wiki/Kalman_filter) and the [complementary filter](https://en.wikipedia.org/wiki/Complementary_filter). For tire/handling effects see [slip angle](https://en.wikipedia.org/wiki/Slip_angle).

## Submission

- Submit your modified `state_estimator.py` as well as the generated PNG outputs.
- Optionally, include any changes you made to `simulate.py` for development (though `state_estimator.py` should still work with the default `simulate.py`).
