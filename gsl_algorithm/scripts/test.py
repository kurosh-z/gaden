import numpy as np
import scipy.interpolate as si
import matplotlib.pyplot as plt
import pandas as pd

# pylint:disable=missing-function-docstring
# pylint:disable=missing-class-docstring
# pylint:disable=invalid-name

if __name__ == "__main__":
    sensor_type = "mox0"
    file_name = f"central_w0__{sensor_type}"

    data = None
    with open(f"/home/kurosh/gaden_ws/src/gsl_algorithm/results/{file_name}.npy", "rb") as f:
        data = np.load(f, allow_pickle=True)

    print("successfully loaded the data")
    timestamps = []
    raw_values = []
    for measurement in data:
        timestamps.append(measurement[sensor_type].header.stamp.to_sec())
        raw_values.append(measurement[sensor_type].raw)

    timestamps = np.array(timestamps)

    if sensor_type == "pid":
        raw_values = np.array(raw_values)
    else:
        assert measurement is not None, "measurement is None"
        raw_values = np.array(raw_values)
        raw_values = measurement[sensor_type].raw_air - raw_values

    df = pd.DataFrame({"time": timestamps, "raw": raw_values})
    df.columns = ["timestamp", "raw_value"]
    tau_half = 1
    df["xs"] = df["raw_value"].ewm(halflife=tau_half).mean()

    df["xs_prime"] = df["xs"].diff() / df["timestamp"].diff()
    df["xs_prime"] = df["xs_prime"].ewm(halflife=tau_half).mean()

    df["xs_dprime"] = df["xs_prime"].diff() / df["timestamp"].diff()
    df["xs_dprime"] = df["xs_dprime"].ewm(halflife=tau_half).mean()
    # calculating the bouts:
    # the bouts are the rising edges of xs_prime which are delimited by two consecutive zero-crossing in the positiv drivation of xs_prime = xs_dprime
    # the amplitude of the bout is the x_prime at the end of the respective bout segment minus the x_prime at the beginning of the bout segment
    # find zero crossings by checking the sign of xs_dprime going changing sign:
    xs_dprime_sing = np.sign(df["xs_dprime"].to_numpy())
    # check for sing changes from negative to positive:
    raising_edes_indices = np.argwhere(xs_dprime_sing > 0)
    ## shifte index by one to get the next index of the zero crossing:
    shifted_indices = np.roll(raising_edes_indices, 1)  ## last element
    # calculate the bout amplitudes:
    bout_amplitudes = (
        df["xs_prime"].to_numpy()[shifted_indices] - df["xs_prime"].to_numpy()[raising_edes_indices]
    ).squeeze()
    # first element has no meaning so remove it
    bout_amplitudes = np.absolute(bout_amplitudes[1::])
    ## remove bouts with amplitude smaller than 0.25:
    bout_indices = np.argwhere(bout_amplitudes > 0.25)
    bout_amplitudes = bout_amplitudes[bout_indices]
    bout_timestamps = df["timestamp"].to_numpy()[bout_indices]

    fig, ax = plt.subplots()
    ax.plot(df["timestamp"].to_numpy(), df["raw_value"].to_numpy(), label="raw")
    # ax.plot(df["timestamp"].to_numpy(), df["xs"].to_numpy(), label="xs")
    ax.plot(df["timestamp"].to_numpy(), df["xs_prime"].to_numpy(), "o-", label="xs_prime")
    ax.plot(df["timestamp"].to_numpy(), df["xs_dprime"].to_numpy(), "o-", label="xs_dprime")
    ax.plot(bout_timestamps, bout_amplitudes, "o-", label="bout")
    ax.legend()
    plt.show()
