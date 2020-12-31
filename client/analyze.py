#!/bin/python3

import click
import matplotlib.pyplot as plt
import ruptures as rpt
import numpy as np
from scipy.stats import norm
import sys
import re

def find_changepoint(signal):
    algo = rpt.Dynp(model="l1", jump=1).fit(signal)
    changes = algo.predict(n_bkps=1)
    return changes[0]

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

@click.command()
@click.argument("data", type=click.File("r"), nargs=1)
def main(data):
    changetimes = []
    samples = read_measurements(data)

    (fig, (time, hist)) = plt.subplots(2)
    fig.suptitle("Plot")
    for sample in samples:

        times = np.array(sample.times)
        values = np.array(sample.values)

        (times, values) = linear_interp(times, values)

        changepoint = find_changepoint(values)

        changetime = times[changepoint]
        changetimes.append(changetime)

        time.plot(times, values)
        time.axvline(changetime)
    hist.hist(changetimes, bins=100)
    plt.show()

if __name__ == "__main__":
    main()
