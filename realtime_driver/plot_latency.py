#!/usr/bin/env python3
"""Histogram the per-iteration latencies dumped by rt.c's RECORD_LAT build."""
import argparse

import numpy as np
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path', nargs='?', default='control_loop_latency.bin',
                         help='binary file written by rt.c (native long, ns)')
    parser.add_argument('--bins', type=int, default=100)
    parser.add_argument('--out', help='save the plot here instead of showing it')
    args = parser.parse_args()

    latencies_ns = np.fromfile(args.path, dtype=np.int64)
    if latencies_ns.size == 0:
        parser.error(f'{args.path} is empty or missing')
    latencies_us = latencies_ns / 1e3

    print(f'n          = {latencies_us.size}')
    print(f'min        = {latencies_us.min():.3f} us')
    print(f'mean       = {latencies_us.mean():.3f} us')
    print(f'median     = {np.median(latencies_us):.3f} us')
    print(f'p99        = {np.percentile(latencies_us, 99):.3f} us')
    print(f'max (WCET) = {latencies_us.max():.3f} us')

    plt.hist(latencies_us, bins=args.bins)
    plt.xlabel('Latency (us)')
    plt.ylabel('Count')
    plt.title(f'Control loop latency ({args.path})')
    plt.axvline(latencies_us.max(), color='r', linestyle='--', label='max (WCET)')
    plt.legend()

    if args.out:
        plt.savefig(args.out)
    else:
        plt.show()


if __name__ == '__main__':
    main()
