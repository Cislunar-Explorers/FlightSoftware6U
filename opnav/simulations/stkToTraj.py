import os
import argparse
import pandas as pd
import numpy as np

"""
Eashaan Kumar (ek485)

This file is used for extracting X,Y,Z,VX,VY,VZ from STK trajectory tables.
"""

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-f", "--filename", help="path to the STK .e file")
    ap.add_argument(
        "-s", "--startline", help="line number of first data point (where t=0)"
    )
    ap.add_argument("-e", "--endline", help="line number of last data point")
    ap.add_argument("-o", "--outfile", help="path to output CSV file")
    args = vars(ap.parse_args())

    d = {"time": [], "x": [], "y": [], "z": [], "vx": [], "vy": [], "vz": []}

    file1 = open(args["filename"], "r")
    Lines = file1.readlines()

    start = int(args["startline"])
    end = int(args["endline"])

    dataLines = Lines[start - 1 : end]

    # Strips the newline character
    for i in range(0, len(dataLines)):
        # Line: time (# secs from 0) x y z vx vy vz
        line = dataLines[i]
        splt = line.split(" ")

        d["time"].append(splt[0])
        d["x"].append(str(float(splt[1]) / 1000.0))
        d["y"].append(str(float(splt[2]) / 1000.0))
        d["z"].append(str(float(splt[3]) / 1000.0))
        d["vx"].append(str(float(splt[4]) / 1000.0))
        d["vy"].append(str(float(splt[5]) / 1000.0))
        d["vz"].append(str(float(splt[6]) / 1000.0))

    df = pd.DataFrame.from_dict(d)
    df.to_csv(args["outfile"], index=False)
