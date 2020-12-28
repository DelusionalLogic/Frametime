#!/bin/python3

import click
import matplotlib.pyplot as plt
import ruptures as rpt
import numpy as np
from scipy.stats import norm
import sys
import csv

def find_changepoint(values):
    signal = np.array(values)
    algo = rpt.Dynp(model="l1", jump=1).fit(signal)
    changes = algo.predict(n_bkps=1)
    return changes[0]

@click.command()
@click.argument("inputs", type=click.File("r"), nargs=-1)
def main(inputs):
    changetimes = []
    for input_ in inputs:
        reader = csv.reader(input_, delimiter=';')
        times = []
        values = []
        next(reader, None)
        for (x, y) in reader:
            times.append(float(x))
            values.append(int(y))

        changepoint = find_changepoint(values)

        changetime = times[changepoint]
        changetimes.append(changetime)
        plt.plot(times, values)
        plt.axvline(changetime)
    # plt.hist(changetimes, bins="auto")
    plt.show()

if __name__ == "__main__":
    main()
