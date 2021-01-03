#!/bin/python3

import click
import math
import matplotlib.pyplot as plt
import numpy as np
import scipy.stats as stats
from pathlib import Path
import sys
import re

def moving_average(a, n=5) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n

class Sample(object):
    def __init__(self, variance, times, values):
        self.variance = variance
        self.times = times
        self.values = values

    def points(self):
        return zip(self.times, self.values)

def read_measurements(f):
    samples = []
    line = f.readline()
    while line != "":
        match = re.match(r"Measurement (\d+)", line)
        assert(match != None)

        line = f.readline()
        match = re.match(r"variance = (\d+)", line)
        assert(match != None)
        variance = int(match.group(1))

        # Header
        f.readline()

        times = []
        values = []
        while True:
            line = f.readline()
            if line == "" or line == "\n":
                break
            match = re.match(r"(\d+(?:\.\d+)?);(\d+)", line)
            assert(match is not None)
            times.append(float(match.group(1)))
            values.append(int(match.group(2)))

        samples.append(Sample(variance, times, values))

        line = f.readline()
        if line == "":
            break

    return samples

def linear_interp(x, y):
    periodic_x = np.linspace(x[0], x[-1], len(x))
    periodic_y = np.interp(periodic_x, x, y)

    return (periodic_x, periodic_y)

def unifunc(x, a, b):
    if x < a or x > b:
        return 0
    else:
        return 1 / b-a

class InputArg(object):
    def __init__(self, arg):
        split = arg.split(":", 1)
        if len(split) > 1:
            self.title = split[0]
            self.path = Path(split[1])
        else:
            self.title = split[0]
            self.path = Path(split[0])

@click.command()
@click.argument("data", nargs=-1)
@click.option("--output", "-o", type=click.File("w"), default=sys.stdout, help="Write values to files instead of stdout")
@click.option("--header", is_flag=True, help="Write a header")
@click.option("--trim", "-t", type=float, default=0, help="Trim the lag times")
def main(data, output, header, trim):
    if header:
        output.write(f"               title     signal    lag_min  lag_delta  rise_mean rise_stddev\n")

    args = [InputArg(x) for x in data]

    for arg in args:
        if arg.path.exists() and arg.path.is_file():
            continue

        sys.stderr.write(f"{arg.path}: file does not exist\n")
        exit(1)

    for arg in args:
        with open(arg.path, "r") as f:
            samples = read_measurements(f)

        changetimes = []
        risetimes = []
        deltas = []
        for sample in samples:

            times = np.array(sample.times)
            values = np.array(sample.values)

            (times, values) = linear_interp(times, values)
            values = moving_average(values)
            # The moving average discards the ends of the values to reduce error
            times = times[2:-2]

            # Check if there's a significant difference between the initial and
            # final light level
            rise = values[-1] - values[0]
            if rise < 10:
                raise Exception("No significant different in light level")

            deltas.append(rise)

            rise_lim = rise * .1

            begin = np.argmax(values > (values[1] + rise_lim))
            end = np.argmin(values < (values[-1] - rise_lim))

            risetime = (times[end] - times[begin])
            risetimes.append(risetime)

            midpoint = np.argmax(values > (values[1] + (rise/2)))

            changetime = times[midpoint]
            changetimes.append(changetime)

        signal_delta = sum(deltas) / len(deltas)

        changetimes = np.array(changetimes)
        if trim != 0:
            changetimes = stats.trimboth(changetimes, trim)
        (loc, scale) = stats.uniform.fit(changetimes)
        lag_min = loc

        (rise_mu, rise_std) = stats.norm.fit(risetimes)

        output.write(f"{arg.title:>20} {signal_delta:10.3f} {lag_min:10.3f} {scale:10.3f} {rise_mu:10.3f} {rise_std:11.3f}\n")

if __name__ == "__main__":
    main()
