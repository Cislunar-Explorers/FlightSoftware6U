import argparse
import pandas as pd
import numpy as np
from scipy.interpolate import InterpolatedUnivariateSpline
from tqdm import tqdm

def changeAcc(path, sample_rate):
    """
    Samples acceleration from original table [path] at new [sample_rate].
    For example, if original acceleration table was sampled at 1 hour intervals, this
    function will return an acceleration table sampled at 1 min intervals if sample_rate=1./60.0.
    
    Precondition: file at [path] must be a csv that contains atleast 2 columns (acc mag (km/sec^2), speed (km/sec)).

    Returns: original file size * 1/sample_rate by 1 numpy array
    """

    assert sample_rate > 0

    accDf = pd.read_csv(path)
    m = np.zeros((len(accDf.index),1))

    for index, row in tqdm(accDf.iterrows(), total=accDf.shape[0]):
        m[index, 0] = float(row['acc mag (km/sec^2)'])

    totaltime = np.arange(0, m.shape[0], 1)
    macc = InterpolatedUnivariateSpline(totaltime, m[:,0])
    
    sample_timeline = np.arange(0, m.shape[0], sample_rate)
    acc = np.zeros((sample_timeline.shape[0]))

    for i,t in enumerate(tqdm(sample_timeline)):
        acc[i] = macc(t)

    d = {'acc': acc}
    return pd.DataFrame(d, columns = ['acc'])

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-f", "--filepath", help = "path to acceleration CSV file to sample")
    ap.add_argument("-r", "--samplerate", help = "new sample rate (float). Eg: If original is sampled at 1 hour, then 0.01667 will return 1-min intervals.")
    ap.add_argument("-o", "--outfile", help = "path to output CSV file")
    args = vars(ap.parse_args())

    df = changeAcc(args['filepath'], float(args['samplerate']))
    df.to_csv(args['outfile'], index=False)
