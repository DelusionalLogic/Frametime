#!/bin/python3

import time
import click
from serial.tools.list_ports import comports as scan_ports
from serial import Serial
import sys
import csv
import struct

def find_device():
    for port in scan_ports():
        if port.vid == 0x16C0 and port.pid == 0x047A:
            return port.device

def calibrate(serial):
    serial.write(b"C\n")

    if serial.read_until() != b"CSTA\n":
        raise Exception("Expected calibration to start")

    data = []

    line = serial.read_until()
    ts = 0
    while line != b"CSUC\n" and line != b"CERR\n":

        line = line[:-1]
        time, value = line.split(b";")
        ts += int(time)
        data.append((ts, int(value)))

        line = serial.read_until()

    if line != b"CSUC\n":
        raise Exception("Calibration failed")

    return data

def measure(serial):
    serial.write(b"M\n")

    if serial.read_until() != b"MSTA\n":
        raise Exception("Expected measurement to start")

    variance = serial.read(2)
    (variance,) = struct.unpack("!H", variance)

    values = []
    line = serial.read(4)
    while line != b"\xFF\xFF\xFF\xFF" and line != b"\xFF\xFF\xFF\xFE":
        values.append(line)
        line = serial.read(4)

    if line != b"\xFF\xFF\xFF\xFE":
        raise Exception("Measurement failed")

    data = []
    if len(values) >= 1:
        ts = 0
        for line in values[1:]:
            time, value = struct.unpack("!HH", line)
            ts += time
            # ts = time
            data.append((ts, value))

    return (variance, data)

def handshake(serial):
    welcome = serial.read_until()
    if not welcome.startswith(b"HELO"):
        raise Exception("Device did not greet us, is this a ScreenTimer?")

def info(serial):
    serial.write(b"I\n")
    resolution = serial.read_until()

    if not resolution.startswith(b"RESL"):
        raise Exception("Incorrect info response")
    resolution = resolution[4:-1]
    # The string is in C style int formatting, so we have to remove any
    # trailing UL
    resolution = resolution.strip(b"UL")
    resolution = int(resolution)

    return (resolution, )

def keycodes(serial, test, reset):
    serial.write(b"K %d %d\n" % (test, reset))
    answer = serial.read_until()

    if answer != b"ACPT\n":
        raise Exception("Keycode not accepted")

def ts_to_us(resolution, data):
    # Prescale the resolution to number of microseconds per tick. Should keep
    # the floating point arithmatic somewhat accurate
    resolution = 1 / (resolution / 1000000)
    newData = []
    for (ts, val) in data:
        ms = resolution * ts
        newData.append((ms, val))

    return newData

@click.command()
@click.option("--output", "-o", type=click.File("w"), default=sys.stdout, help="Write values to files instead of stdout")
@click.option("--delay", "-d", type=click.INT, default=0, help="Wait n seconds before taking the measurement")
def main(output, delay):
    device = find_device()
    serial = Serial(device)
    handshake(serial)
    keycodes(serial, 4, 42)
    (resolution,) = info(serial)

    time.sleep(delay)
    (_, measurement) = measure(serial)
    measurement = ts_to_us(resolution, measurement)
    # measurement = measure(serial)

    writer = csv.writer(output, delimiter=';')
    writer.writerow(("Time(us)", "Light(unitless)"))
    for (x, y) in measurement:
        writer.writerow((x, y))

if __name__ == "__main__":
    main()
