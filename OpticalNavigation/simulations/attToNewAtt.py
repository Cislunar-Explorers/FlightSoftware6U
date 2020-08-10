import argparse
import pandas as pd
import numpy as np
from scipy.interpolate import InterpolatedUnivariateSpline
from tqdm import tqdm

def changeEph(path, sample_rate):
    """
    Samples attitude from original table [path] at new [sample_rate].
    For example, if original attitude table was sampled at 1 hour intervals, this
    function will return an attitude table sampled at 1 min intervals if sample_rate=1./60.0.
    
    Precondition: file at [path] must be a csv that contains atleast 4 columns (q1, q2, q3, q4).

    Returns: original file size * 1/sample_rate by len(columns) numpy array
    """

    assert sample_rate > 0

    Ephdf = pd.read_csv(path)
    m = np.zeros((len(Ephdf.index),4))

    for index, row in tqdm(Ephdf.iterrows(), total=Ephdf.shape[0]):
        m[index, 0] = float(row['q1'])
        m[index, 1] = float(row['q2'])
        m[index, 2] = float(row['q3'])
        m[index, 3] = float(row['q4'])

    totaltime = np.arange(0, m.shape[0], 1)
    mq1 = InterpolatedUnivariateSpline(totaltime, m[:,0])
    mq2 = InterpolatedUnivariateSpline(totaltime, m[:,1])
    mq3 = InterpolatedUnivariateSpline(totaltime, m[:,2])
    mq4 = InterpolatedUnivariateSpline(totaltime, m[:,3])
    
    sample_timeline = np.arange(0, m.shape[0], sample_rate)
    q1 = np.zeros((sample_timeline.shape[0]))
    q2 = np.zeros((sample_timeline.shape[0]))
    q3 = np.zeros((sample_timeline.shape[0]))
    q4 = np.zeros((sample_timeline.shape[0]))

    for i,t in enumerate(tqdm(sample_timeline)):
        q1[i] = mq1(t)
        q2[i] = mq2(t)
        q3[i] = mq3(t)
        q4[i] = mq4(t)


    d = {'q1': q1, 'q2':q2, 'q3':q3, 'q4':q4}
    return pd.DataFrame(d, columns = ['q1','q2','q3','q4'])

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-f", "--filepath", help = "path to attitude CSV file to sample")
    ap.add_argument("-r", "--samplerate", help = "new sample rate (float)")
    ap.add_argument("-o", "--outfile", help = "path to output CSV file")
    args = vars(ap.parse_args())

    df = changeEph(args['filepath'], float(args['samplerate']))
    df.to_csv(args['outfile'], index=False)
