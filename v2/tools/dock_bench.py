#!/usr/bin/env python3
"""SesAnt v2 home-puck bench: N x (undock -> stand -> dock), logs success, time and charge voltage to CSV.

The robot starts standing over the home puck (or seated on it). Each cycle:
  undock -> wait until state 0 (free) -> hold `--stand` s -> dock -> wait for state 3 (charging) or 5 (no contact).
Usage:  python dock_bench.py /dev/ttyACM0 --cycles 100 --csv dock_log.csv
Needs pyserial. The serial port is the same one ant_server.py uses, so stop ant_server first.
"""
import argparse, csv, sys, time

STATES = {0: 'free', 1: 'docking', 2: 'docked', 3: 'charging', 4: 'undocking', 5: 'dock_fail'}


def parse_meas(line):
    """meas line -> dict(t, vbat, chg, state) or None. v2 appends 3 fields after the 29 RealAnt fields + heap."""
    if not line.startswith('meas '): return None
    v = line[5:].strip().split('\t')
    if len(v) < 33: return None
    return {'t': int(v[0]), 'vbat': int(v[30]), 'chg': int(v[31]), 'state': int(v[32])}


def wait_state(ser, want, timeout):
    t0 = time.time(); last = None
    while time.time() - t0 < timeout:
        m = parse_meas(ser.readline().decode(errors='ignore'))
        if m:
            last = m
            if m['state'] in want: return m, time.time() - t0
    return last, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('port'); ap.add_argument('--cycles', type=int, default=100)
    ap.add_argument('--stand', type=float, default=2.0); ap.add_argument('--csv', default='dock_log.csv')
    a = ap.parse_args()
    import serial
    ser = serial.Serial(a.port, 115200, timeout=0.2); time.sleep(2.0); ser.reset_input_buffer()
    ok = 0
    with open(a.csv, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['cycle', 'result', 'dock_s', 'vbat_mV', 'charge_mV'])
        for i in range(1, a.cycles + 1):
            ser.write(b'undock\n')
            m, dt = wait_state(ser, {0}, 6)
            if dt is None: print(f'{i}: undock timeout, last {m}'); w.writerow([i, 'undock_timeout', '', '', '']); break
            time.sleep(a.stand); ser.reset_input_buffer()
            ser.write(b'dock\n')
            m, dt = wait_state(ser, {3, 5}, 8)
            res = 'timeout' if dt is None else STATES[m['state']]
            ok += res == 'charging'
            w.writerow([i, res, f'{dt:.2f}' if dt else '', m and m['vbat'], m and m['chg']]); f.flush()
            print(f'{i:4d}: {res:9s} {dt or 0:5.2f} s  vbat {m and m["vbat"]} mV  pads {m and m["chg"]} mV   success {ok}/{i}')
    print(f'done: {ok}/{i} docked with charge contact')


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--selftest':
        line = 'meas 1234\t' + '\t'.join(['512'] * 16) + '\t' + '\t'.join(['0'] * 12) + '\t200000\t7650\t8120\t3\n'
        assert parse_meas(line) == {'t': 1234, 'vbat': 7650, 'chg': 8120, 'state': 3}, parse_meas(line)
        assert parse_meas('meas 1\t2') is None
        print('selftest ok')
    else:
        main()
