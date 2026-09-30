"""Run tests from the ESP-IDF Unity menu of a flashed board over its serial port.

Sends a test filter to the menu (a [tag], a test name, or * for everything), echoes the
board's output and exits with 0 if all selected tests pass, 1 if any fail, 2 on problems.

Examples:
    python run_unity_menu.py                 # auto-detect the board, run every test
    python run_unity_menu.py --filter "[hw]"
    python run_unity_menu.py --print-port    # print the detected port and exit
    python run_unity_menu.py --print-console # print the console the board needs and exit
"""
import argparse
import re
import sys
import time

import serial
from serial.tools import list_ports

# ESP32-C3 built-in USB-Serial/JTAG, and the USB-UART bridges common on dev boards
BOARD_VIDS = {
    0x303A: 'Espressif USB-Serial/JTAG',
    0x10C4: 'Silicon Labs CP210x',
    0x1A86: 'WCH CH340/CH343',
    0x0403: 'FTDI',
}
SUMMARY = re.compile(r'(\d+) Tests (\d+) Failures (\d+) Ignored')


def console_for(port: str) -> str:
    """The ESP32-C3's built-in USB port needs the USB-Serial/JTAG console; a bridge needs UART."""
    for info in list_ports.comports():
        if info.device == port and info.vid == 0x303A:
            return 'usb_serial_jtag'
    return 'uart'


def find_port() -> str:
    candidates = [p for p in list_ports.comports() if p.vid in BOARD_VIDS]
    if len(candidates) == 1:
        return candidates[0].device
    if not candidates:
        found = ', '.join(f'{p.device} ({p.description})' for p in list_ports.comports()) or 'none'
        sys.exit(f'No ESP32 board found. Serial ports: {found}. Pass --port to choose one.')
    listed = ', '.join(f'{p.device} ({BOARD_VIDS[p.vid]})' for p in candidates)
    sys.exit(f'More than one possible board: {listed}. Pass --port to choose one.')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', help='Serial port (default: auto-detect)')
    parser.add_argument('--filter', default='*', help='Unity menu input (default: * runs all tests)')
    parser.add_argument('--timeout', type=float, default=300, help='Seconds to wait for the summary')
    parser.add_argument('--print-port', action='store_true', help='Print the detected port and exit')
    parser.add_argument('--print-console', action='store_true',
                        help='Print the console the board needs (usb_serial_jtag or uart) and exit')
    args = parser.parse_args()

    port = args.port or find_port()
    if args.print_port:
        print(port)
        return 0
    if args.print_console:
        print(console_for(port))
        return 0

    link = serial.Serial()
    link.port = port
    link.baudrate = 115200
    link.timeout = 0.2
    # Writes block while the firmware is not reading its console, e.g. the wrong console is selected
    link.write_timeout = 5
    # Leave DTR/RTS released: asserting them holds the chip in reset or bootloader mode
    link.dtr = False
    link.rts = False
    try:
        link.open()
    except serial.SerialException as err:
        print(f'Cannot open {port}: {err}', file=sys.stderr)
        return 2

    with link:
        # Let the boot banner and menu prompt through; the menu waits for input indefinitely
        settle = time.monotonic() + 2
        while time.monotonic() < settle:
            sys.stdout.write(link.read(4096).decode(errors='replace'))
        link.reset_input_buffer()
        link.write(args.filter.encode() + b'\n')

        pending = ''
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            chunk = link.read(4096).decode(errors='replace')
            if not chunk:
                continue
            sys.stdout.write(chunk)
            sys.stdout.flush()
            pending = (pending + chunk)[-512:]
            match = SUMMARY.search(pending)
            if match:
                tests, failures = int(match.group(1)), int(match.group(2))
                if tests == 0:
                    print(f'\nNo tests matched {args.filter!r}', file=sys.stderr)
                    return 2
                return 1 if failures else 0

    print(f'\nNo test summary within {args.timeout:.0f}s', file=sys.stderr)
    return 2


if __name__ == '__main__':
    sys.exit(main())
