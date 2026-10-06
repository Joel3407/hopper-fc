#!/usr/bin/env python3
"""Tests for the ground station frame parser. Run: .venv/bin/python test_parser.py"""

import os
import struct
import sys

os.environ.setdefault("MPLBACKEND", "Agg")  # no display needed
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ground_station import FrameParser, FRAME_SIZE


def make_frame(state=3, roll=1.5, pitch=-2.25, alt=4.0, thr=0.5, vbat=22.2,
               corrupt_checksum=False):
    payload = struct.pack('<B5f', state, roll, pitch, alt, thr, vbat)
    cs = 0
    for b in payload:
        cs ^= b
    if corrupt_checksum:
        cs ^= 0xFF
    return b'\xaa\x55' + payload + bytes([cs])


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)


def test_frame_size():
    check(FRAME_SIZE == 24 == len(make_frame()), "frame size should be 24")


def test_good_frame():
    pkts = FrameParser().feed(make_frame())
    check(len(pkts) == 1, f"expected 1 packet, got {len(pkts)}")
    p = pkts[0]
    check(p['state'] == 3, "state")
    check(p['roll'] == 1.5 and p['pitch'] == -2.25, "roll/pitch")
    check(p['altitude'] == 4.0 and p['throttle'] == 0.5, "alt/throttle")
    check(abs(p['voltage'] - 22.2) < 1e-5, "voltage")


def test_bad_checksum_dropped_then_resync():
    parser = FrameParser()
    pkts = parser.feed(make_frame(corrupt_checksum=True) + make_frame(state=4))
    check(len(pkts) == 1, f"expected only the good frame, got {len(pkts)}")
    check(pkts[0]['state'] == 4, "should have re-synced to the second frame")
    check(parser.bad_frames == 1, "bad frame should be counted")


def test_garbage_before_sync():
    garbage = b'\x00\x13\xaa\x99\x55\xaa\xaa\x07'  # includes stray 0xAA bytes
    pkts = FrameParser().feed(garbage + make_frame(state=2))
    check(len(pkts) == 1 and pkts[0]['state'] == 2, "frame after garbage lost")


def test_split_across_reads():
    frame = make_frame(state=5)
    for cut in range(1, FRAME_SIZE):  # every possible split point
        parser = FrameParser()
        first = parser.feed(frame[:cut])
        second = parser.feed(frame[cut:])
        check(first == [], f"cut={cut}: partial frame must not yield a packet")
        check(len(second) == 1 and second[0]['state'] == 5, f"cut={cut}: lost frame")


def test_back_to_back_frames():
    pkts = FrameParser().feed(make_frame(state=1) + make_frame(state=2))
    check([p['state'] for p in pkts] == [1, 2], "two frames in one read")


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"PASS  {t.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {t.__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)
