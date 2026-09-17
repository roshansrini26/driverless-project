import argparse

import numpy as np
import pandas as pd


# =========================================================
# CONFIG
# =========================================================

STEERING_FACTOR = 0.32
CROP_SPEED = 1.0
CROP_MARGIN = 1.0
TARGET_HZ = 100

ROS1=False
if(ROS1):
    WHEEL_SPEED_FACTOR = 1.0
    SIGNALS = {
        "/cog_position.pose.pose.position.x": "true_x",
        "/cog_position.pose.pose.position.y": "true_y",
        "/cog_position.twist.twist.linear.x": "true_vx",
        "/imu.linear_acceleration.x": "acc_x",
        "/imu.linear_acceleration.y": "acc_y",
        "/imu.angular_velocity.z": "yaw_rate",
        "/car_sensors.steerAngle": "steer_angle",
        "/car_sensors.wheelSpeedRR": "wheel_speed_rr",
        "/car_sensors.wheelSpeedRL": "wheel_speed_rl",
        "/car_sensors.wheelSpeedFR": "wheel_speed_fr",
        "/car_sensors.wheelSpeedFL": "wheel_speed_fl",
        "/cog_position.pose.pose.orientation.x": "orientation_x",
        "/cog_position.pose.pose.orientation.y": "orientation_y",
        "/cog_position.pose.pose.orientation.z": "orientation_z",
        "/cog_position.pose.pose.orientation.w": "orientation_w",
    }
else:
    WHEEL_SPEED_FACTOR = 11.81
    SIGNALS = {
        "/cog_position.pose.pose.position.x": "true_x",
        "/cog_position.pose.pose.position.y": "true_y",
        "/dcp/velocity.x_mps": "true_vx",
        "/imu.linear_acceleration.x": "acc_x",
        "/imu.linear_acceleration.y": "acc_y",
        "/imu.angular_velocity.z": "yaw_rate",
        "/dcp/steer_angle.data": "steer_angle",
        "/dcp/wheel_speeds.data_rr": "wheel_speed_rr",
        "/dcp/wheel_speeds.data_rl": "wheel_speed_rl",
        "/dcp/wheel_speeds.data_fr": "wheel_speed_fr",
        "/dcp/wheel_speeds.data_fl": "wheel_speed_fl",
        "/cog_position.pose.pose.orientation.x": "orientation_x",
        "/cog_position.pose.pose.orientation.y": "orientation_y",
        "/cog_position.pose.pose.orientation.z": "orientation_z",
        "/cog_position.pose.pose.orientation.w": "orientation_w",
    }

WHEEL_SPEED_COLUMNS = [
    "wheel_speed_fl",
    "wheel_speed_fr",
    "wheel_speed_rl",
    "wheel_speed_rr",
]


QUATERNION_COLUMNS = [
    "orientation_x",
    "orientation_y",
    "orientation_z",
    "orientation_w",
]


# =========================================================
# CROP DRIVING SECTION
# =========================================================

def crop_driving_section(df):
    if "true_vx" not in df.columns:
        return df

    moving = df["true_vx"] > CROP_SPEED

    if not moving.any():
        return df

    indices = np.flatnonzero(moving.to_numpy())

    start_time = (
        df["elapsed_time"].iloc[indices[0]]
        - CROP_MARGIN
    )

    end_time = (
        df["elapsed_time"].iloc[indices[-1]]
        + CROP_MARGIN
    )

    return df[
        (df["elapsed_time"] >= start_time)
        & (df["elapsed_time"] <= end_time)
    ].copy()


# =========================================================
# MAIN
# =========================================================

def prepare_data(input_file, output_file):

    # -----------------------------------------------------
    # Read CSV
    # -----------------------------------------------------

    df = pd.read_csv(input_file)

    if "elapsed time" in df.columns:
        df = df.rename(
            columns={"elapsed time": "elapsed_time"}
        )

    required_columns = [
        "elapsed_time",
        "topic",
        "value",
    ]

    for col in required_columns:
        if col not in df.columns:
            raise ValueError(
                f"Missing required column: {col}"
            )

    df["elapsed_time"] = pd.to_numeric(
        df["elapsed_time"],
        errors="coerce",
    )

    df["value"] = pd.to_numeric(
        df["value"],
        errors="coerce",
    )

    df = df.dropna(
        subset=[
            "elapsed_time",
            "topic",
            "value",
        ]
    )

    # -----------------------------------------------------
    # Keep only requested topics
    # -----------------------------------------------------

    df = df[df["topic"].isin(SIGNALS)].copy()

    if df.empty:
        raise ValueError(
            "None of the requested topics were found."
        )

    df["signal"] = df["topic"].map(SIGNALS)

    # -----------------------------------------------------
    # Check that ALL configured signals are present
    # -----------------------------------------------------

    missing_topics = [
        topic
        for topic in SIGNALS
        if topic not in df["topic"].unique()
    ]

    if missing_topics:
        missing_names = [
            SIGNALS[topic]
            for topic in missing_topics
        ]

        raise ValueError(
            "Missing required signals:\n"
            + "\n".join(
                f"  {topic} -> {name}"
                for topic, name in zip(
                    missing_topics,
                    missing_names,
                )
            )
        )

    # -----------------------------------------------------
    # Split signals
    #
    # Each signal keeps its own original timestamps.
    # -----------------------------------------------------

    signal_data = {}

    for topic, name in SIGNALS.items():

        data = df[
            df["topic"] == topic
        ][
            ["elapsed_time", "value"]
        ].copy()

        data = data.rename(
            columns={"value": name}
        )

        data = (
            data
            .sort_values("elapsed_time")
            .drop_duplicates(
                subset="elapsed_time",
                keep="last",
            )
        )

        signal_data[name] = data

    # -----------------------------------------------------
    # Determine common time range
    #
    # Start:
    #   when ALL signals have at least one sample.
    #
    # End:
    #   when ALL signals still have data.
    #
    # This guarantees every output row can contain a
    # value for every signal.
    # -----------------------------------------------------

    first_times = [
        data["elapsed_time"].iloc[0]
        for data in signal_data.values()
    ]

    last_times = [
        data["elapsed_time"].iloc[-1]
        for data in signal_data.values()
    ]

    start_time = max(first_times)
    end_time = min(last_times)

    if start_time >= end_time:
        raise ValueError(
            "No time range where all signals have data."
        )

    # -----------------------------------------------------
    # Common 100 Hz timeline
    # -----------------------------------------------------

    timestep = 1.0 / TARGET_HZ

    common_time = np.arange(
        start_time,
        end_time + timestep / 2.0,
        timestep,
    )

    result = pd.DataFrame({
        "elapsed_time": common_time
    })

    # -----------------------------------------------------
    # Latest available value for EVERY signal
    #
    # For each output timestamp:
    # use the most recent source sample whose timestamp
    # is <= the output timestamp.
    # -----------------------------------------------------

    for name, data in signal_data.items():

        result = pd.merge_asof(
            result.sort_values("elapsed_time"),
            data.sort_values("elapsed_time"),
            on="elapsed_time",
            direction="backward",
        )

    # -----------------------------------------------------
    # Calculate yaw from quaternion
    # -----------------------------------------------------

    x = result["orientation_x"].to_numpy(
        dtype=float
    )
    y = result["orientation_y"].to_numpy(
        dtype=float
    )
    z = result["orientation_z"].to_numpy(
        dtype=float
    )
    w = result["orientation_w"].to_numpy(
        dtype=float
    )

    yaw = np.arctan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y**2 + z**2),
    )

    # Remove +/- pi discontinuities
    result["true_yaw"] = np.unwrap(yaw)

    # -----------------------------------------------------
    # Crop driving section
    # -----------------------------------------------------

    result = crop_driving_section(result)

    # -----------------------------------------------------
    # Wheel speed conversion
    # -----------------------------------------------------

    for col in WHEEL_SPEED_COLUMNS:
        result[col] /= WHEEL_SPEED_FACTOR

    # -----------------------------------------------------
    # Steering conversion
    # -----------------------------------------------------

    if "steer_angle" in result.columns:
        result["steer_angle"] *= STEERING_FACTOR

    # -----------------------------------------------------
    # Remove quaternion components
    # -----------------------------------------------------

    result = result.drop(
        columns=QUATERNION_COLUMNS,
        errors="ignore",
    )

    # -----------------------------------------------------
    # Final signal completeness check
    # -----------------------------------------------------

    expected_output_signals = [
        SIGNALS[topic]
        for topic in SIGNALS
        if SIGNALS[topic] not in QUATERNION_COLUMNS
    ]

    missing_output_columns = [
        name
        for name in expected_output_signals
        if name not in result.columns
    ]

    if missing_output_columns:
        raise ValueError(
            "Missing output signals: "
            + ", ".join(missing_output_columns)
        )

    # -----------------------------------------------------
    # Check for NaNs
    # -----------------------------------------------------

    nan_columns = result.columns[
        result.isna().any()
    ].tolist()

    if nan_columns:
        raise ValueError(
            "NaNs remain in: "
            + ", ".join(nan_columns)
        )

    # -----------------------------------------------------
    # Save
    # -----------------------------------------------------

    result.to_csv(
        output_file,
        index=False,
    )

    print(f"Saved: {output_file}")
    print(f"Rows: {len(result)}")
    print(f"Frequency: {TARGET_HZ} Hz")
    print(
        f"Duration: "
        f"{result['elapsed_time'].iloc[-1] - result['elapsed_time'].iloc[0]:.2f} s"
    )


# =========================================================
# COMMAND LINE
# =========================================================

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "input_file",
        help="Input Foxglove CSV",
    )

    args = parser.parse_args()

    if not args.input_file.endswith("_raw.csv"):
        raise ValueError(
            "Input file must end with '_raw.csv'"
        )

    output_file = args.input_file.replace(
        "_raw.csv",
        ".csv",
    )

    prepare_data(
        args.input_file,
        output_file,
    )


if __name__ == "__main__":
    main()
