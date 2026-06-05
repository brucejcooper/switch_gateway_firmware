#!/usr/bin/env python3
"""End-to-end test harness for the light-switch controller + TinyModbus DALI bus.

The harness drives the controller over its USB-CDC CLI (the same text commands a
human types), captures *all* controller log output for diagnosis, prints it live
for readability, and runs a series of tests against the live DALI bus.

Typical flow (run from the controller directory):

    .venv/bin/python test/run_tests.py                  # build, flash, test
    .venv/bin/python test/run_tests.py --no-build --no-flash   # just test

It will:
  1. build the controller firmware ( make in the build/ subdir ),
  2. flash it ( picotool load -f -x light_switch.uf2 ),
  3. wait ~2 s for the Pico to re-enumerate and reboot,
  4. wait for the controller to finish its boot-time bus discovery,
  5. raise the log level to dbg so the Modbus traffic is captured,
  6. run the tests, dumping the captured controller logs (with a heuristic
     diagnosis) whenever a test fails.

Test rig wiring note: two devices share Modbus slave id 2 on the RS-485 bus —
a relay board and a TinyModbus DALI gateway driving an LED ballast.  The DALI
tests target slave 2.

Requires pyserial (already in controller/.venv).
"""

import argparse
import collections
import glob
import queue
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial not found — run with the venv: .venv/bin/python test/run_tests.py")

# ── Paths / defaults ────────────────────────────────────────────────────────
CONTROLLER_DIR = Path(__file__).resolve().parent.parent
BUILD_DIR = CONTROLLER_DIR / "build"
UF2_NAME = "light_switch.uf2"
DEFAULT_PORT = "/dev/tty.usbmodem111201"
DEFAULT_SLAVE = 2

# ── DALI command opcodes (IEC 62386-102) ────────────────────────────────────
DALI_QUERY_CONTROL_GEAR_PRESENT = 0x91  # gear answers 0xFF (YES) if present
DALI_QUERY_STATUS = 0x90

# TinyModbus register map (see TinyModbus/src/registers.h)
GW_RAW_CMD_RESULT = 0x0108  # FC04 input reg: outcome of the last raw DALI cmd

# ── Pretty printing (thread-safe) ───────────────────────────────────────────
_print_lock = threading.Lock()


def _supports_color():
    return sys.stdout.isatty()


class C:
    if _supports_color():
        DIM = "\033[90m"
        RED = "\033[31m"
        GREEN = "\033[32m"
        YELLOW = "\033[33m"
        CYAN = "\033[36m"
        BOLD = "\033[1m"
        RST = "\033[0m"
    else:
        DIM = RED = GREEN = YELLOW = CYAN = BOLD = RST = ""


def out(msg=""):
    with _print_lock:
        print(msg, flush=True)


# ── DALI frame helpers ──────────────────────────────────────────────────────
def short_addr_command_frame(short_addr, opcode):
    """Build a 16-bit DALI forward frame addressing a short address in command
    form: address byte = 0AAAAAA1, opcode in the low byte."""
    addr_byte = ((short_addr & 0x3F) << 1) | 0x01
    return (addr_byte << 8) | (opcode & 0xFF)


# ── Harness ─────────────────────────────────────────────────────────────────
class HarnessError(Exception):
    pass


class Harness:
    """Owns the serial connection, a background reader thread that classifies
    every line (controller log vs JSON event vs command response), and helpers
    to issue CLI commands and parse their replies."""

    def __init__(self, port, print_logs=True):
        self.port = port
        self.print_logs = print_logs
        self._ser = None
        self._stop = threading.Event()
        self._reader = None
        self._t0 = time.monotonic()
        self._resp = queue.Queue()              # command-response lines
        self._logs = collections.deque(maxlen=5000)  # (ts, text) controller logs
        self._events = collections.deque(maxlen=2000)  # (ts, text) JSON events
        self._logs_lock = threading.Lock()

    # -- lifecycle --
    @staticmethod
    def resolve_port(preferred):
        """Return `preferred` if it exists, else the first /dev/tty.usbmodem*.
        The Pico's USB-CDC suffix can change across reflashes on macOS, so we
        fall back to whatever modem port is currently present."""
        import os
        if preferred and os.path.exists(preferred):
            return preferred
        cands = sorted(glob.glob("/dev/tty.usbmodem*"))
        return cands[0] if cands else preferred

    def open(self, timeout=25.0):
        """Open the port, retrying while the Pico re-enumerates after a flash."""
        deadline = time.monotonic() + timeout
        last_err = None
        while time.monotonic() < deadline:
            try:
                self.port = self.resolve_port(self.port)
                self._ser = serial.Serial(self.port, baudrate=115200, timeout=0.2)
                break
            except (serial.SerialException, OSError) as e:
                last_err = e
                time.sleep(0.3)
        if self._ser is None:
            raise HarnessError(f"could not open {self.port} within {timeout}s: {last_err}")
        self._t0 = time.monotonic()
        self._stop.clear()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def close(self):
        self._stop.set()
        if self._reader:
            self._reader.join(timeout=1.0)
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass

    # -- reader thread --
    def _read_loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                data = self._ser.read(256)
            except (serial.SerialException, OSError):
                break
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self._handle_line(raw.decode("utf-8", "replace").rstrip("\r"))

    def _handle_line(self, text):
        ts = time.monotonic() - self._t0
        if text.startswith("# "):
            with self._logs_lock:
                self._logs.append((ts, text))
            if self.print_logs:
                out(f"  {C.DIM}{ts:7.2f} {text}{C.RST}")
        elif text.startswith("{"):
            self._events.append((ts, text))
            if self.print_logs:
                out(f"  {C.DIM}{ts:7.2f} » {text}{C.RST}")
        else:
            if text.strip():
                self._resp.put((ts, text))

    # -- log access for diagnosis --
    def mark(self):
        return time.monotonic() - self._t0

    def logs_since(self, t):
        with self._logs_lock:
            return [(ts, txt) for ts, txt in self._logs if ts >= t]

    # -- command primitives --
    def _drain_resp(self):
        try:
            while True:
                self._resp.get_nowait()
        except queue.Empty:
            pass

    def _send(self, cmd):
        self._ser.write((cmd + "\n").encode())
        self._ser.flush()

    def simple_cmd(self, cmd, timeout=4.0):
        """Send a command whose reply terminates with 'ok' or 'err: ...'.
        Returns the terminal line (str)."""
        self._drain_resp()
        self._send(cmd)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                _, line = self._resp.get(timeout=0.2)
            except queue.Empty:
                continue
            if line == "ok" or line.startswith("err:"):
                return line
        raise HarnessError(f"timeout waiting for reply to {cmd!r}")

    _ROW_RE = re.compile(r"^\s*(\d+)\s+(\d+)\s+0x([0-9A-Fa-f]{4})\s*$")

    def read_regs(self, slave, reg, count=1, fc="input", timeout=4.0):
        """Issue get_input / get_holding and parse the register table.
        Returns a list[int] of length `count`; raises HarnessError on err."""
        cmd = "get_input" if fc == "input" else "get_holding"
        self._drain_resp()
        self._send(f"{cmd} {slave} 0x{reg:04X} {count}")
        rows = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                _, line = self._resp.get(timeout=0.2)
            except queue.Empty:
                continue
            if line.startswith("err:"):
                raise HarnessError(f"{cmd} {slave} 0x{reg:04X}: {line}")
            m = self._ROW_RE.match(line)
            if m:
                rows.append(int(m.group(3), 16))
                if len(rows) >= count:
                    return rows
        raise HarnessError(f"timeout reading {count} reg(s) from slave {slave} @0x{reg:04X}")

    # -- higher-level helpers --
    def set_log_level(self, level):
        self._drain_resp()
        self._send(f"log {level}")
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            try:
                _, line = self._resp.get(timeout=0.2)
            except queue.Empty:
                continue
            if line.startswith("log:") or line.startswith("err:"):
                return line
        raise HarnessError("no reply to log-level command (is the controller alive?)")

    def wait_ready(self, timeout=30.0):
        """Block until the controller finishes booting.  Watches for the hello
        JSON / 'serial: hello' log emitted at the end of boot.  If neither shows
        up (e.g. --no-flash against an already-running controller), falls back to
        a liveness check via the log command."""
        deadline = time.monotonic() + timeout
        out(f"{C.CYAN}Waiting for controller boot/discovery…{C.RST}")
        while time.monotonic() < deadline:
            for _, txt in list(self._events):
                if '"t":"hello"' in txt:
                    out(f"{C.GREEN}Controller ready (hello received).{C.RST}")
                    return
            with self._logs_lock:
                logs = list(self._logs)
            for _, txt in logs:
                if "serial: hello" in txt or "main loop" in txt:
                    out(f"{C.GREEN}Controller ready.{C.RST}")
                    return
            time.sleep(0.2)
        # Fallback: maybe it booted before we connected. Probe liveness.
        try:
            reply = self.set_log_level("dbg")
            out(f"{C.YELLOW}No boot banner seen, but controller responds ({reply}). "
                f"Assuming already running.{C.RST}")
            return
        except HarnessError:
            raise HarnessError("controller did not become ready (no boot banner, no reply)")


# ── Diagnosis ───────────────────────────────────────────────────────────────
_DIAG_KEYWORDS = ("exception", "timeout", "busy", "crc", "unreachable",
                  "failed", "gave up", "error", "stuck")


def diagnose(log_lines):
    """Pick out the controller log lines that hint at what went wrong."""
    hits = [txt for _, txt in log_lines
            if any(k in txt.lower() for k in _DIAG_KEYWORDS)]
    return hits


def dump_logs(h, since, header="controller logs during test"):
    lines = h.logs_since(since)
    out(f"\n{C.YELLOW}── {header} ({len(lines)} lines) ──{C.RST}")
    for ts, txt in lines:
        out(f"  {C.DIM}{ts:7.2f}{C.RST} {txt}")
    hits = diagnose(lines)
    if hits:
        out(f"{C.YELLOW}── likely-relevant lines ──{C.RST}")
        for txt in hits:
            out(f"  {C.RED}{txt}{C.RST}")


# ── Tests ───────────────────────────────────────────────────────────────────
# Each test takes (h, cfg) and returns (passed: bool, detail: str).

def interpret_raw_result(result):
    """Decode TinyModbus raw_cmd_result (FC04 0x0108)."""
    hi, lo = result >> 8, result & 0xFF
    if hi == 0xFF:
        return ("pending", f"still PENDING (0x{result:04X}, {lo} send(s) left)")
    if hi == 0x00:
        return ("ok", f"DALI backward frame = 0x{lo:02X}")
    if hi == 0x01:
        return ("nak", "NAK — no ballast answered (normal for send-twice)")
    if hi == 0x02:
        sub = {1: "malformed", 2: "too_short", 3: "in_middle", 4: "too_long"}.get(lo, f"subtype {lo}")
        return ("err", f"DALI bus/timing error: {sub}")
    return ("unknown", f"0x{result:04X}")


def recover_stuck_gateway(h, cfg):
    """If the gateway is wedged returning DEVICE_BUSY, reboot it and wait."""
    out(f"{C.YELLOW}Gateway appears stuck-busy — rebooting slave {cfg.slave} "
        f"via gw_reboot…{C.RST}")
    try:
        reply = h.simple_cmd(f"gw_reboot {cfg.slave}")
    except HarnessError as e:
        out(f"{C.RED}gw_reboot got no reply: {e}{C.RST}")
        reply = "(no reply)"
    out(f"  gw_reboot -> {reply}")
    # Gateway holds off the bus ~1 s after reset, then re-runs its own scan.
    time.sleep(3.0)


def test_presence(h, cfg):
    """Send a raw DALI 'QUERY CONTROL GEAR PRESENT' to short address `addr` and
    read the gateway's stored backward frame to decide if a device is present."""
    slave, addr = cfg.slave, cfg.addr
    frame = short_addr_command_frame(addr, DALI_QUERY_CONTROL_GEAR_PRESENT)
    out(f"DALI QUERY CONTROL GEAR PRESENT -> short addr {addr} "
        f"(raw frame 0x{frame:04X}) on slave {slave}")

    # 1. Send the raw query.
    reply = h.simple_cmd(f"dali_raw {slave} 0x{frame:04X}")
    out(f"  dali_raw -> {reply}")

    # 1b. If the channel is wedged busy, reboot the gateway and retry once.
    if reply.startswith("err:") and ("0x06" in reply or "busy" in reply.lower()):
        recover_stuck_gateway(h, cfg)
        reply = h.simple_cmd(f"dali_raw {slave} 0x{frame:04X}")
        out(f"  dali_raw (after reboot) -> {reply}")

    if reply.startswith("err:"):
        return False, f"raw send rejected: {reply}"

    # 2. Poll the raw_cmd_result register until the query completes.
    result = None
    for _ in range(40):  # ~2 s
        try:
            result = h.read_regs(slave, GW_RAW_CMD_RESULT, 1, fc="input")[0]
        except HarnessError as e:
            return False, f"could not read raw_cmd_result: {e}"
        state, _ = interpret_raw_result(result)
        if state != "pending":
            break
        time.sleep(0.05)

    state, detail = interpret_raw_result(result)
    out(f"  raw_cmd_result = 0x{result:04X} -> {detail}")

    if state == "ok":
        return True, f"device present at addr {addr} ({detail})"
    if state == "nak":
        return False, f"no device answered at addr {addr} (NAK)"
    if state == "pending":
        return False, "gateway never completed the query (stuck PENDING)"
    return False, f"DALI error: {detail}"


def test_present_bitmask(h, cfg):
    """Cross-check: read the gateway's presence bitmask (FC04 0x0100..0x0103)
    and report which short addresses it believes are present."""
    slave = cfg.slave
    regs = h.read_regs(slave, 0x0100, 4, fc="input")
    mask = regs[0] | (regs[1] << 16) | (regs[2] << 32) | (regs[3] << 48)
    present = [a for a in range(64) if mask & (1 << a)]
    out(f"  present bitmask = 0x{mask:016X} -> addresses {present or '(none)'}")
    if cfg.addr in present:
        return True, f"addr {cfg.addr} present in gateway bitmask"
    return False, (f"addr {cfg.addr} not in gateway presence bitmask "
                   f"(present: {present or 'none'})")


TESTS = [
    ("dali_presence_addr0", test_presence),
    ("gateway_present_bitmask", test_present_bitmask),
]


# ── Build / flash ───────────────────────────────────────────────────────────
def run_step(title, argv, cwd):
    out(f"{C.BOLD}{C.CYAN}● {title}{C.RST}  ({' '.join(argv)})")
    proc = subprocess.run(argv, cwd=cwd)
    if proc.returncode != 0:
        raise SystemExit(f"{C.RED}{title} failed (exit {proc.returncode}){C.RST}")


def build_firmware():
    run_step("Building controller firmware", ["make"], cwd=BUILD_DIR)


def flash_firmware():
    run_step("Flashing controller firmware", ["picotool", "load", "-f", "-x", UF2_NAME],
             cwd=BUILD_DIR)
    out(f"{C.CYAN}Sleeping 2 s for the Pico to re-enumerate…{C.RST}")
    time.sleep(2.0)


# ── Optional Saleae line capture ────────────────────────────────────────────
def start_capture(cfg):
    """Begin a Saleae capture of the DALI/RS-485 lines. Returns a capture
    object or None if Saleae is unavailable (the test still runs without it)."""
    try:
        from saleae_capture import SaleaeCapture
    except ImportError as e:
        out(f"{C.YELLOW}--capture requested but saleae_capture import failed: {e}{C.RST}")
        return None
    # Timed mode: the app auto-stops after capture-seconds, so a crash/kill of
    # this harness can never leave the Saleae app wedged in "recording".
    cap = SaleaeCapture(baud=cfg.capture_baud, port=cfg.saleae_port,
                        duration=cfg.capture_seconds)
    try:
        cap.start()
        out(f"{C.CYAN}Saleae capture running ({cap.sample_rate} S/s, "
            f"channels {cap.channels}, {cfg.capture_seconds:.0f}s window)…{C.RST}")
        return cap
    except Exception as e:
        out(f"{C.YELLOW}Saleae capture unavailable — continuing without it:\n  {e}{C.RST}")
        cap.close()
        return None


def finish_capture(cap, name):
    try:
        from saleae_capture import format_summary
        summary = cap.stop_and_export(name)
        out(f"\n{C.CYAN}── Saleae line trace ({name}) ──{C.RST}")
        out(format_summary(summary))
    except Exception as e:
        out(f"{C.YELLOW}Saleae capture export failed: {e}{C.RST}")
    finally:
        cap.close()


# ── Main ────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="Controller + TinyModbus end-to-end test rig")
    ap.add_argument("--port", default=DEFAULT_PORT, help="controller USB-CDC serial port")
    ap.add_argument("--slave", type=int, default=DEFAULT_SLAVE, help="DALI gateway Modbus slave id")
    ap.add_argument("--addr", type=int, default=0, help="DALI short address to probe")
    ap.add_argument("--no-build", action="store_true", help="skip the firmware build")
    ap.add_argument("--no-flash", action="store_true", help="skip flashing")
    ap.add_argument("--log-level", default="dbg", help="controller log level during tests")
    ap.add_argument("--quiet-logs", action="store_true", help="don't stream controller logs live")
    ap.add_argument("--capture", action="store_true",
                    help="capture the DALI/RS-485 lines with Saleae Logic during the DALI test")
    ap.add_argument("--saleae-port", type=int, default=10430, help="Saleae automation server port")
    ap.add_argument("--capture-baud", type=int, default=19200,
                    help="RS-485 baud for Modbus decode in the trace")
    ap.add_argument("--capture-test", default="dali_presence_addr0",
                    help="which test to wrap with a Saleae line capture")
    ap.add_argument("--capture-seconds", type=float, default=6.0,
                    help="fixed Saleae capture window (timed mode — auto-stops, can't wedge the app)")
    cfg = ap.parse_args()

    if not cfg.no_build:
        build_firmware()
    if not cfg.no_flash:
        flash_firmware()

    h = Harness(cfg.port, print_logs=not cfg.quiet_logs)
    out(f"{C.CYAN}Opening {cfg.port}…{C.RST}")
    h.open()
    try:
        h.wait_ready()
        out(f"{C.CYAN}Setting controller log level to '{cfg.log_level}'…{C.RST}")
        out(f"  {h.set_log_level(cfg.log_level)}")
        # Let a little background polling settle / be captured before testing.
        time.sleep(0.5)

        results = []
        for name, fn in TESTS:
            out(f"\n{C.BOLD}══ TEST: {name} ══{C.RST}")
            cap = start_capture(cfg) if (cfg.capture and name == cfg.capture_test) else None
            start = h.mark()
            try:
                passed, detail = fn(h, cfg)
            except HarnessError as e:
                passed, detail = False, f"harness error: {e}"
            finally:
                if cap is not None:
                    finish_capture(cap, name)
            results.append((name, passed, detail))
            if passed:
                out(f"{C.GREEN}✔ PASS{C.RST} {name}: {detail}")
            else:
                out(f"{C.RED}✘ FAIL{C.RST} {name}: {detail}")
                dump_logs(h, start, header=f"controller logs during {name}")

        # Summary
        out(f"\n{C.BOLD}══ SUMMARY ══{C.RST}")
        npass = sum(1 for _, p, _ in results if p)
        for name, passed, detail in results:
            tag = f"{C.GREEN}PASS{C.RST}" if passed else f"{C.RED}FAIL{C.RST}"
            out(f"  [{tag}] {name}: {detail}")
        out(f"{npass}/{len(results)} passed")
        return 0 if npass == len(results) else 1
    finally:
        h.close()


if __name__ == "__main__":
    sys.exit(main())
