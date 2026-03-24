#!/usr/bin/env python3
"""
Keyboard usage analyzer for k2k_quiet_and_proud.

Captures key-press events from the USB CDC serial port (e.g. /dev/ttyACM0)
and analyzes frequency, digram, and trigram statistics to help optimize
the keyboard layout.

Protocol from firmware:
  P<HH>\n  – key at matrix index HH (hex) was pressed
  R<HH>\n  – key at matrix index HH (hex) was released

Usage:
  # Capture to file (Ctrl-C to stop):
  python analyze_keys.py capture /dev/ttyACM0 keys.log

  # Analyze a previously captured file:
  python analyze_keys.py analyze keys.log

  # Capture and analyze live (shows rolling stats, Ctrl-C to stop):
  python analyze_keys.py live /dev/ttyACM0
"""

import sys
import time
import collections
from pathlib import Path

# ---------------------------------------------------------------------------
# Matrix-index → key name mapping (mirrors TRANSLATION in main.rs)
# Index 0x00-0x41 maps to the physical key label (QWERTY positions).
# ---------------------------------------------------------------------------
MATRIX_KEY_NAMES = {
    0x00: "F",
    0x01: "S",
    0x02: "A",
    0x03: "BSpace",
    0x04: "Down",
    0x05: "Slash",
    0x06: "M",
    0x07: "Comma",
    0x08: "4",
    0x09: "2",
    0x0A: "Minus",
    0x0B: "PgUp",
    0x0C: "Home",
    0x0D: "Equal",
    0x0E: "7",
    0x0F: "9",
    0x10: "V",
    0x11: "X",
    0x12: "Z",
    0x13: "Left",
    0x14: "Enter",
    0x15: "Quote",
    0x16: "J",
    0x17: "L",
    0x18: "R",
    0x19: "E",
    0x1A: "Q",
    0x1B: "LAlt",
    0x1C: "Up",
    0x1D: "RShift",
    0x1E: "N",
    0x1F: "Dot",
    0x20: "B",
    0x21: "C",
    0x22: "LShift",
    0x23: "Right",
    0x24: "Delete",
    0x25: "SColon",
    0x26: "H",
    0x27: "K",
    0x28: "G",
    0x29: "D",
    0x2A: "CapsLock",
    0x2B: "Space",
    0x2C: "End",
    0x2D: "0",
    0x2E: "6",
    0x2F: "8",
    0x30: "T",
    0x31: "W",
    0x32: "Tab",
    0x33: "LCtrl",
    0x34: "LGui",
    0x35: "P",
    0x36: "U",
    0x37: "I",
    0x38: "5",
    0x39: "3",
    0x3A: "1",
    0x3B: "PgDown",
    0x3C: "UK0",
    0x3D: "LBracket",
    0x3E: "Y",
    0x3F: "O",
    0x40: "UK1(IR)",
    0x41: "UK2(Snip)",
}

# Dvorak→QWERTY remapping applied by the firmware's dvorak handler.
# Maps from the QWERTY key name in MATRIX_KEY_NAMES to what the user
# actually types (so frequency counts reflect real character usage).
DVORAK_MAP = {
    "A": "A", "B": "X", "C": "J", "D": "E", "E": ".",
    "F": "U", "G": "I", "H": "D", "I": "C", "J": "H",
    "K": "T", "L": "N", "M": "M", "N": "B", "O": "R",
    "P": "L", "Q": "Quote", "R": "P", "S": "O", "T": "Y",
    "U": "G", "V": "K", "W": "Comma", "X": "Q", "Y": "F",
    "Z": "SColon",
    "SColon": "S", "Quote": "Minus", "LBracket": "Slash",
    "Comma": "W", "Dot": "V", "Slash": "Z",
    "1": "1", "2": "2", "3": "3", "4": "4", "5": "5",
    "6": "6", "7": "7", "8": "8", "9": "9", "0": "0",
}


def key_name(idx: int, use_dvorak: bool = True) -> str:
    name = MATRIX_KEY_NAMES.get(idx, f"0x{idx:02X}")
    if use_dvorak:
        return DVORAK_MAP.get(name, name)
    return name


# ---------------------------------------------------------------------------
# Event parsing
# ---------------------------------------------------------------------------

def parse_line(line: str):
    """
    Parse a firmware event line.
    Returns (event_type, matrix_index) or None on parse error.
    event_type is 'P' (press) or 'R' (release).
    """
    line = line.strip()
    if len(line) == 3 and line[0] in ('P', 'R'):
        try:
            return line[0], int(line[1:], 16)
        except ValueError:
            pass
    return None


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------

class KeyStats:
    def __init__(self, use_dvorak: bool = True):
        self.use_dvorak = use_dvorak
        self.press_count: collections.Counter = collections.Counter()
        self.digrams: collections.Counter = collections.Counter()
        self.trigrams: collections.Counter = collections.Counter()
        self._last_two: list = []  # rolling window of last 2 presses

    def record_press(self, idx: int):
        name = key_name(idx, self.use_dvorak)
        self.press_count[name] += 1

        if self._last_two:
            self.digrams[(self._last_two[-1], name)] += 1
        if len(self._last_two) >= 2:
            self.trigrams[(self._last_two[-2], self._last_two[-1], name)] += 1

        self._last_two.append(name)
        if len(self._last_two) > 2:
            self._last_two.pop(0)

    def total_presses(self) -> int:
        return sum(self.press_count.values())

    def report(self, top_n: int = 20):
        total = self.total_presses()
        if total == 0:
            print("No key presses recorded.")
            return

        print(f"\n{'='*60}")
        print(f"Total key presses: {total}")
        print(f"{'='*60}")

        print(f"\nTop {top_n} keys by frequency:")
        print(f"  {'Key':<14} {'Count':>7}  {'%':>6}")
        print(f"  {'-'*14} {'-'*7}  {'-'*6}")
        for name, count in self.press_count.most_common(top_n):
            print(f"  {name:<14} {count:>7}  {count/total*100:>5.1f}%")

        print(f"\nTop {top_n} digrams (consecutive key pairs):")
        print(f"  {'Digram':<20} {'Count':>7}  {'%':>6}")
        print(f"  {'-'*20} {'-'*7}  {'-'*6}")
        total_di = sum(self.digrams.values())
        for (a, b), count in self.digrams.most_common(top_n):
            label = f"{a} → {b}"
            print(f"  {label:<20} {count:>7}  {count/total_di*100:>5.1f}%")

        if self.trigrams:
            print(f"\nTop {top_n} trigrams:")
            print(f"  {'Trigram':<28} {'Count':>7}  {'%':>6}")
            print(f"  {'-'*28} {'-'*7}  {'-'*6}")
            total_tri = sum(self.trigrams.values())
            for (a, b, c), count in self.trigrams.most_common(top_n):
                label = f"{a} → {b} → {c}"
                print(f"  {label:<28} {count:>7}  {count/total_tri*100:>5.1f}%")

        # Hand balance (rough heuristic based on column position)
        self._hand_balance_report()

    def _hand_balance_report(self):
        """Very rough left/right hand split based on matrix index."""
        # Left-hand matrix indices (adjust for your physical layout)
        LEFT_KEYS = {
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x10, 0x11, 0x12, 0x13,
            0x14, 0x15, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x30, 0x31,
            0x32, 0x33, 0x34,
        }
        left = sum(
            count for name, count in self.press_count.items()
            if any(
                MATRIX_KEY_NAMES.get(idx) == (DVORAK_MAP.get(name, name) if self.use_dvorak else name)
                for idx in LEFT_KEYS
            )
        )
        total = self.total_presses()
        right = total - left
        print(f"\nHand balance (rough): Left {left/total*100:.1f}%  Right {right/total*100:.1f}%")


# ---------------------------------------------------------------------------
# Modes
# ---------------------------------------------------------------------------

def cmd_capture(port: str, outfile: str):
    """Capture raw event lines from the USB CDC port to a file."""
    import serial  # type: ignore
    count = 0
    print(f"Capturing from {port} → {outfile}  (Ctrl-C to stop)")
    with serial.Serial(port, timeout=1) as ser, open(outfile, "w") as f:
        try:
            while True:
                line = ser.readline().decode("ascii", errors="replace").strip()
                if line:
                    f.write(line + "\n")
                    f.flush()
                    count += 1
                    if count % 100 == 0:
                        print(f"  {count} events captured…", end="\r")
        except KeyboardInterrupt:
            pass
    print(f"\nDone. {count} events saved to {outfile}")


def cmd_analyze(infile: str, use_dvorak: bool = True):
    """Analyze a previously captured log file."""
    stats = KeyStats(use_dvorak=use_dvorak)
    with open(infile) as f:
        for line in f:
            result = parse_line(line)
            if result and result[0] == 'P':
                stats.record_press(result[1])
    stats.report()


def cmd_live(port: str, use_dvorak: bool = True):
    """Live capture + rolling stats display."""
    import serial  # type: ignore
    stats = KeyStats(use_dvorak=use_dvorak)
    print(f"Live capture from {port}  (Ctrl-C to stop and show final report)")
    last_report = time.time()
    try:
        with serial.Serial(port, timeout=0.1) as ser:
            while True:
                line = ser.readline().decode("ascii", errors="replace").strip()
                if line:
                    result = parse_line(line)
                    if result and result[0] == 'P':
                        stats.record_press(result[1])
                now = time.time()
                if now - last_report >= 30:
                    print(f"\n--- Rolling report ({stats.total_presses()} presses) ---")
                    stats.report(top_n=10)
                    last_report = now
    except KeyboardInterrupt:
        pass
    stats.report()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        sys.exit(0)

    cmd = args[0]
    dvorak = "--qwerty" not in args  # default: apply Dvorak mapping

    if cmd == "capture" and len(args) >= 3:
        cmd_capture(args[1], args[2])
    elif cmd == "analyze" and len(args) >= 2:
        cmd_analyze(args[1], use_dvorak=dvorak)
    elif cmd == "live" and len(args) >= 2:
        cmd_live(args[1], use_dvorak=dvorak)
    else:
        print(__doc__)
        sys.exit(1)


if __name__ == "__main__":
    main()
