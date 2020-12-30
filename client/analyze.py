#!/bin/python3

import click
import matplotlib.pyplot as plt
import ruptures as rpt
import numpy as np
from scipy.stats import norm
import sys
import re

def find_changepoint(values):
    signal = np.array(values)
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

@click.command()
@click.argument("data", type=click.File("r"), nargs=1)
def main(data):
    changetimes = []
    samples = read_measurements(data)
    for sample in samples:

        changepoint = find_changepoint(sample.values)

        changetime = sample.times[changepoint]
        changetimes.append(changetime)
        plt.plot(sample.times, sample.values)
        plt.axvline(changetime)
    # plt.hist(changetimes, bins="auto")
    plt.show()

if __name__ == "__main__":
    main()
