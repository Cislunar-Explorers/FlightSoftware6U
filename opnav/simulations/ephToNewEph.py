import argparse
import pandas as pd
import numpy as np
from scipy.interpolate import InterpolatedUnivariateSpline
from tqdm import tqdm


def changeEph(path, sample_rate):
    """
    Samples ephemeris or trajectory from original table [path] at new [sample_rate].
    For example, if original sun ephemeris table was sampled at 1 hour intervals, this
    function will return an ephemeris table sampled at 1 min intervals if sample_rate=1./60.0.

    Precondition: file at [path] must be a csv that contains 6 columns (x, y, z, vx, vy, vz).

    Returns: original file size * 1/sample_rate by len(columns) numpy array
    """

    assert sample_rate > 0

    Ephdf = pd.read_csv(path)
    m = np.zeros((len(Ephdf.index), 6))

    for index, row in tqdm(Ephdf.iterrows(), total=Ephdf.shape[0]):
        m[index, 0] = float(row["x"])
        m[index, 1] = float(row["y"])
        m[index, 2] = float(row["z"])
        m[index, 3] = float(row["vx"])
        m[index, 4] = float(row["vy"])
        m[index, 5] = float(row["vz"])

    totaltime = np.arange(0, m.shape[0], 1)
    mx = InterpolatedUnivariateSpline(totaltime, m[:, 0])
    my = InterpolatedUnivariateSpline(totaltime, m[:, 1])
    mz = InterpolatedUnivariateSpline(totaltime, m[:, 2])
    mvx = InterpolatedUnivariateSpline(totaltime, m[:, 3])
    mvy = InterpolatedUnivariateSpline(totaltime, m[:, 4])
    mvz = InterpolatedUnivariateSpline(totaltime, m[:, 5])

    sample_timeline = np.arange(0, m.shape[0], sample_rate)
    x = np.zeros((sample_timeline.shape[0]))
    y = np.zeros((sample_timeline.shape[0]))
    z = np.zeros((sample_timeline.shape[0]))
    vx = np.zeros((sample_timeline.shape[0]))
    vy = np.zeros((sample_timeline.shape[0]))
    vz = np.zeros((sample_timeline.shape[0]))

    for i, t in enumerate(tqdm(sample_timeline)):
        x[i] = mx(t)
        y[i] = my(t)
        z[i] = mz(t)
        vx[i] = mvx(t)
        vy[i] = mvy(t)
        vz[i] = mvz(t)

    d = {"x": x, "y": y, "z": z, "vx": vx, "vy": vy, "vz": vz}
    return pd.DataFrame(d, columns=["x", "y", "z", "vx", "vy", "vz"])


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-f", "--filepath", help="path to ephemeris CSV file to sample")
    ap.add_argument("-r", "--samplerate", help="new sample rate (float)")
    ap.add_argument("-o", "--outfile", help="path to output CSV file")
    args = vars(ap.parse_args())

    df = changeEph(args["filepath"], float(args["samplerate"]))
    df.to_csv(args["outfile"], index=False)
