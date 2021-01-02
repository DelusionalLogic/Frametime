#!/bin/python3

import click
import math
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
from scipy.stats import (
    norm,
    uniform
)
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

@click.command()
@click.argument("data", type=click.File("r"), default=sys.stdin, nargs=1)
@click.option("--output", "-o", type=click.File("w"), default=sys.stdout, help="Write values to files instead of stdout")
def main(data, output):
    changetimes = []
    risetimes = []
    deltas = []
    samples = read_measurements(data)

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

    (loc, scale) = uniform.fit(changetimes)
    lag_min = loc
    lag_max = loc + scale

    (rise_mu, rise_std) = norm.fit(risetimes)

    output.write(f"Signal: {signal_delta}\n")
    output.write(f"Minimum Lag: {lag_min}\n")
    output.write(f"Maximum Lag: {lag_max}\n")
    output.write(f"Mean Risetime: {rise_mu}\n")
    output.write(f"Stddev Risetime: {rise_std}\n")

if __name__ == "__main__":
    main()
