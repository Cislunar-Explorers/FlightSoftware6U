import os
import argparse
import pandas as pd
import re
import numpy as np

"""
Eashaan Kumar (ek485)

This file is used for extracting X,Y,Z,VX,VY,VZ from HORIZONs ephemeris tables.
https://ssd.jpl.nasa.gov/horizons.cgi#top
"""


def extractVariable(line, re, varName, varForm, lineNum):
    """
    Uses regex [re] to extract [varName] of form [varForm] from [line].
    [lineNum] is the line number of [line] in the HORIZONS txt file.
    returns:
        float variable
        if regex matching fails, then None is returned
    """
    v = re.search(line)
    if v is None:
        print(
            f"{varName} not found. Please check if horizons file contains {varName} coordinates in the form {varForm} @ line {lineNum}"
        )
        return None
    return np.float64(v.group(1))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-f", "--filename", help="path to the horizons text file")
    ap.add_argument("-s", "--startline", help="line number of $$SOE symbol")
    ap.add_argument("-e", "--endline", help="line number of $$EOE symbol")
    ap.add_argument("-o", "--outfile", help="path to output CSV file")
    args = vars(ap.parse_args())

    d = {"time": [], "x": [], "y": [], "z": [], "vx": [], "vy": [], "vz": []}

    file1 = open(args["filename"], "r")
    Lines = file1.readlines()

    start = int(args["startline"])
    end = int(args["endline"])

    timeRe = re.compile(r"(\d+.\d+) = A.D. (\d+-[a-zA-Z]+-\d+ \d+:\d+:\d+.\d+)")

    XRe = re.compile(r" X =([ -]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)")
    YRe = re.compile(r" Y =([ -]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)")
    ZRe = re.compile(r" Z =([ -]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)")

    VXRe = re.compile(r" VX=([ -]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)")
    VYRe = re.compile(r" VY=([ -]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)")
    VZRe = re.compile(r" VZ=([ -]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)")

    dataLines = Lines[start : end - 1]
    # Strips the newline character
    for i in range(0, len(dataLines), 3):
        # First line: ... = [date]
        time = timeRe.search(dataLines[i])
        if time is None:
            print(
                f"Time not found. Please check if horizons file contains time in approprate format @ line {i + start + 0 + 1}"
            )
            exit()
        time = str(time.group(2))
        # Second line: X =(-)... Y =(-)... Z =(-)...
        x = extractVariable(dataLines[i + 1], XRe, "X", "X =...", i + start + 1 + 1)
        y = extractVariable(dataLines[i + 1], YRe, "Y", "Y =...", i + start + 1 + 1)
        z = extractVariable(dataLines[i + 1], ZRe, "Z", "Z =...", i + start + 1 + 1)
        # Third line: VX=(-)... VY=(-)... VZ=(-)...
        vx = extractVariable(dataLines[i + 2], VXRe, "VX", "VX=...", i + start + 2 + 1)
        vy = extractVariable(dataLines[i + 2], VYRe, "VY", "VY=...", i + start + 2 + 1)
        vz = extractVariable(dataLines[i + 2], VZRe, "VZ", "VZ=...", i + start + 2 + 1)

        if (
            x is None
            or y is None
            or z is None
            or vx is None
            or vy is None
            or vz is None
        ):
            exit()
        d["time"].append(time)
        d["x"].append(x)
        d["y"].append(y)
        d["z"].append(z)
        d["vx"].append(vx)
        d["vy"].append(vy)
        d["vz"].append(vz)

    df = pd.DataFrame.from_dict(d)
    df.to_csv(args["outfile"], index=False)
