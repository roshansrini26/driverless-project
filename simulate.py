from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import pandas as pd
from state_estimator import StateEstimator
from datatypes import CarSignals, State


def run_dataset(dataset_path: Path):
    # Load example data using pandas for robustness
    df = pd.read_csv(dataset_path)

    # Store true trajectory if available
    xs_true = df.get('true_x', pd.Series(np.nan, index=df.index)).tolist()
    ys_true = df.get('true_y', pd.Series(np.nan, index=df.index)).tolist()
    yaws_true = df.get('true_yaw', pd.Series(np.nan, index=df.index)).tolist()
    vxs_true = df.get('true_vx', pd.Series(np.nan, index=df.index)).tolist()

    estimator = StateEstimator()
    estimator.x = xs_true[0] if not np.isnan(xs_true[0]) else 0.0
    estimator.y = ys_true[0] if not np.isnan(ys_true[0]) else 0.0
    estimator.yaw = yaws_true[0] if not np.isnan(yaws_true[0]) else 0.0

    # Store predicted trajectory and yaw
    xs_pred, ys_pred, yaws_pred, vxs_pred = [], [], [], []

    for _, row in df.iterrows():
        signals = CarSignals(
            elapsed_time=row['elapsed_time'],
            acc_x=row['acc_x'],
            acc_y=row['acc_y'],
            yaw_rate=row['yaw_rate'],
            steer_angle=row['steer_angle'],
            wheel_speed_rr=row['wheel_speed_rr'],
            wheel_speed_rl=row['wheel_speed_rl'],
            wheel_speed_fr=row['wheel_speed_fr'],
            wheel_speed_fl=row['wheel_speed_fl'],
        )
        state = estimator.predict(signals)

        xs_pred.append(state.x)
        ys_pred.append(state.y)
        yaws_pred.append(state.yaw)
        vxs_pred.append(state.vx)

    fig = plt.figure(figsize=(14, 6))
    gs = gridspec.GridSpec(2, 2, width_ratios=[2, 1])

    # Trajectory plot (left, spans both rows)
    ax1 = fig.add_subplot(gs[:, 0])
    ax1.plot(xs_true, ys_true, label="Ground Truth", color="black", linestyle="--")
    ax1.plot(xs_pred, ys_pred, label="Estimated", color="red")
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("y [m]")
    ax1.set_title("Trajectory")
    ax1.grid(True)
    ax1.axis("equal")
    ax1.legend()

    # Yaw plot (top right)
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(yaws_true, label="Ground Truth Yaw", color="black", linestyle="--")
    ax2.plot(yaws_pred, label="Estimated Yaw", color="red")
    ax2.set_xlabel("Timestep")
    ax2.set_ylabel("Yaw [rad]")
    ax2.set_title("Yaw")
    ax2.grid(True)
    ax2.legend()

    # Velocity plot (bottom right)
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.plot(vxs_true, label="Ground Truth Velocity", color="black", linestyle="--")
    ax3.plot(vxs_pred, label="Estimated Velocity", color="red")
    ax3.set_xlabel("Timestep")
    ax3.set_ylabel("Velocity [m/s]")
    ax3.set_title("Velocity")
    ax3.grid(True)
    ax3.legend()

    output_dir = Path("output")
    output_dir.mkdir(exist_ok=True)
    output_path = output_dir / f"{dataset_path.stem}.png"

    plt.tight_layout()
    plt.savefig(output_path)
    plt.close(fig)
    print(f"Combined plot saved to {output_path}")


def main():
    dataset_paths = sorted(Path("data").glob("autox_*.csv"))

    if not dataset_paths:
        raise FileNotFoundError("No datasets found in data/. Expected files matching data/autox_*.csv.")

    for dataset_path in dataset_paths:
        run_dataset(dataset_path)


if __name__ == "__main__":
    main()
