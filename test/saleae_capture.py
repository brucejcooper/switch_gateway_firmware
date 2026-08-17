#!/usr/bin/env python3
"""Saleae Logic 2 capture helper for the light-switch bench.

Captures the DALI and RS-485 lines via the Saleae Logic 2 *automation API* while
something else (e.g. test/run_tests.py) drives the controller, then decodes the
RS-485 (Modbus) bytes and summarises DALI line activity so a failing DALI
transaction can be diagnosed at the wire level.

Probe channel map (Saleae digital inputs on the TinyModbus gateway):
    D0  DALI TX   (gateway → DALI bus, forward frames)
    D1  DALI RX   (DALI bus → gateway, ballast backward frames)
    D4  RS485 RX  (controller → gateway, Modbus requests)
    D5  RS485 TX  (gateway → controller, Modbus responses)
    D6  DE        (RS-485 driver-enable)

Requirements:
  * `logic2-automation` (installed in the venv).
  * The Saleae Logic 2 app must be RUNNING with its automation server enabled:
        app menu → Preferences → enable "Automation Server" (port 10430),
    then restart the app if it was already open.
  * The logic probe must be plugged in.

Standalone use (capture N seconds of whatever is on the bus):
    .venv/bin/python test/saleae_capture.py --seconds 2

Library use: see SaleaeCapture (used by run_tests.py --capture and
dali_tx_probe.py).

IMPORTANT — arming latency: there is a multi-second gap between starting a
capture and the device actually sampling.  Traffic that happens during that gap
is NOT recorded.  So to reliably capture a brief event (e.g. one DALI frame),
keep relevant bus traffic flowing for the *whole* window and fire the event of
interest a couple of seconds in (past arming) — see dali_tx_probe.py for the
pattern.  Always use a TimedCaptureMode (duration=) capture, never an unattended
ManualCaptureMode one: a manual capture whose client dies leaves the Saleae app
wedged in "recording" (every later start fails with "Cannot switch sessions
while recording") until the app is restarted.
"""

import argparse
import csv
import glob
import os
import sys
import tempfile
import time

try:
    import saleae.automation as sa
except ImportError:
    sys.exit("logic2-automation not installed — run: .venv/bin/pip install logic2-automation")

# Default channel assignments (see module docstring).
DALI_TX, DALI_RX = 0, 1
SER_RX, SER_TX, SER_DE = 4, 5, 6
DEFAULT_CHANNELS = [DALI_TX, DALI_RX, SER_RX, SER_TX, SER_DE]

# Digital sample rates to try, in preference order — different Saleae devices
# support different rate sets, so we fall back until one is accepted.  All are
# far above what 19200-baud RS-485 (52 us/bit) or 1200-baud DALI needs.
CANDIDATE_RATES = [12_500_000, 16_000_000, 12_000_000, 25_000_000,
                   24_000_000, 10_000_000, 8_000_000, 6_250_000, 50_000_000]


class SaleaeError(Exception):
    pass


class SaleaeCapture:
    """Manual-mode capture wrapper: start() … (run your test) … stop_and_export()."""

    def __init__(self, channels=None, sample_rate=None, baud=19200,
                 ser_rx=SER_RX, ser_tx=SER_TX, port=10430, out_dir=None,
                 duration=None):
        self.channels = list(channels) if channels else list(DEFAULT_CHANNELS)
        self.sample_rate = sample_rate
        self.baud = baud
        self.ser_rx = ser_rx
        self.ser_tx = ser_tx
        self.port = port
        self.duration = duration  # set → TimedCaptureMode (auto-stops, can't wedge)
        self.out_dir = out_dir or tempfile.mkdtemp(prefix="saleae_")
        self._mgr = None
        self._cap = None
        self._stopped = False
        self._analyzers = []  # (label, handle)

    # -- lifecycle --
    def connect(self):
        try:
            self._mgr = sa.Manager.connect(port=self.port, connect_timeout_seconds=8)
        except Exception as e:
            raise SaleaeError(
                f"could not reach the Saleae automation server on port {self.port}.\n"
                f"  Open the Saleae Logic app, enable Preferences → 'Automation Server',\n"
                f"  make sure the probe is connected, then retry.\n  ({type(e).__name__}: {e})")
        devs = [d for d in self._mgr.get_devices() if not d.is_simulation]
        if not devs:
            raise SaleaeError("no physical Saleae device found (is the probe plugged in?)")
        self._device_id = devs[0].device_id
        return self

    def start(self):
        """Begin a manual capture.  Returns self."""
        if self._mgr is None:
            self.connect()
        rates = [self.sample_rate] if self.sample_rate else CANDIDATE_RATES
        last_err = None
        for rate in rates:
            cfg = sa.LogicDeviceConfiguration(
                enabled_digital_channels=self.channels,
                digital_sample_rate=rate,
            )
            mode = (sa.TimedCaptureMode(duration_seconds=self.duration)
                    if self.duration else sa.ManualCaptureMode())
            cap_cfg = sa.CaptureConfiguration(capture_mode=mode)
            try:
                self._cap = self._mgr.start_capture(
                    device_id=self._device_id,
                    device_configuration=cfg,
                    capture_configuration=cap_cfg,
                )
                self.sample_rate = rate
                break
            except Exception as e:
                last_err = e
                if "sample rate" not in str(e).lower():
                    raise SaleaeError(f"start_capture failed: {type(e).__name__}: {e}")
        if self._cap is None:
            raise SaleaeError(f"no usable digital sample rate: {last_err}")
        # Async-serial analyzers on the two RS-485 directions (Modbus @ baud).
        for label, ch in (("modbus_rx", self.ser_rx), ("modbus_tx", self.ser_tx)):
            if ch in self.channels:
                h = self._cap.add_analyzer("Async Serial", label=label, settings={
                    "Input Channel": ch,
                    "Bit Rate (Bits/s)": self.baud,
                })
                self._analyzers.append((label, h))
        return self

    def stop_and_export(self, name="capture"):
        """Stop the capture, export decoded serial + raw digital, return a summary dict."""
        if self._cap is None:
            raise SaleaeError("capture not started")
        # Timed captures auto-stop after their duration — wait for that; manual
        # captures run until we stop them.
        if self.duration:
            self._cap.wait()
        else:
            self._cap.stop()
        self._stopped = True
        os.makedirs(self.out_dir, exist_ok=True)
        sal_path = os.path.join(self.out_dir, f"{name}.sal")
        serial_csv = os.path.join(self.out_dir, f"{name}_serial.csv")
        digital_dir = os.path.join(self.out_dir, f"{name}_digital")

        summary = {"sal": sal_path, "sample_rate": self.sample_rate}
        try:
            self._cap.save_capture(sal_path)
        except Exception as e:
            summary["save_error"] = str(e)

        # Decoded Modbus bytes per direction.
        if self._analyzers:
            try:
                exports = [sa.DataTableExportConfiguration(h, sa.RadixType.HEXADECIMAL)
                           for _, h in self._analyzers]
                self._cap.export_data_table(serial_csv, analyzers=exports)
                summary["serial_csv"] = serial_csv
                summary["modbus"] = _parse_serial_csv(serial_csv)
            except Exception as e:
                summary["serial_error"] = str(e)

        # Raw digital edges (for DALI lines, which have no async-serial decode).
        try:
            os.makedirs(digital_dir, exist_ok=True)
            self._cap.export_raw_data_csv(digital_dir, digital_channels=self.channels)
            summary["digital_dir"] = digital_dir
            summary["lines"] = _parse_digital_activity(digital_dir)
        except Exception as e:
            summary["digital_error"] = str(e)

        return summary

    def close(self):
        # Best-effort: stop a manual capture that was never stopped, otherwise
        # the app stays wedged in "recording" after we disconnect.
        if self._cap is not None and not self._stopped and not self.duration:
            try:
                self._cap.stop()
            except Exception:
                pass
            self._stopped = True
        for obj in (self._cap, self._mgr):
            try:
                if obj:
                    obj.close()
            except Exception:
                pass
        self._cap = self._mgr = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.close()


# ── CSV parsing / analysis ──────────────────────────────────────────────────
def _find_col(fieldnames, *needles):
    for f in fieldnames:
        low = f.lower()
        if any(n in low for n in needles):
            return f
    return None


def _parse_serial_csv(path):
    """Group decoded async-serial bytes into Modbus frames per direction.
    Returns {'modbus_rx': [frame, …], 'modbus_tx': [...]}, each frame a list of
    int byte values (split on >2 ms inter-byte gaps ≈ Modbus 3.5-char silence)."""
    rows = []
    with open(path, newline="") as fh:
        reader = csv.DictReader(fh)
        if not reader.fieldnames:
            return {}
        name_c = _find_col(reader.fieldnames, "name")
        time_c = _find_col(reader.fieldnames, "start_time", "start", "time")
        data_c = _find_col(reader.fieldnames, "data")
        for r in reader:
            val = (r.get(data_c) or "").strip()
            if not val:
                continue
            try:
                # export_data_table above is always configured with
                # RadixType.HEXADECIMAL, so Data is always hex; int(val, 16)
                # already tolerates an optional "0x"/"0X" prefix on its own.
                b = int(val, 16)
            except ValueError:
                continue
            try:
                t = float(r.get(time_c, "nan"))
            except ValueError:
                t = float("nan")
            rows.append((r.get(name_c, ""), t, b))
    out = {}
    for label in ("modbus_rx", "modbus_tx"):
        seq = [(t, b) for nm, t, b in rows if nm == label]
        seq.sort(key=lambda x: x[0])
        frames, cur, last_t = [], [], None
        for t, b in seq:
            if last_t is not None and (t - last_t) > 0.002:
                if cur:
                    frames.append(cur)
                cur = []
            cur.append(b)
            last_t = t
        if cur:
            frames.append(cur)
        out[label] = frames
    return out


def _parse_digital_activity(directory):
    """Count transitions and active span per channel from an exported digital CSV."""
    csvs = glob.glob(os.path.join(directory, "*.csv"))
    if not csvs:
        return {}
    path = csvs[0]
    with open(path, newline="") as fh:
        reader = csv.reader(fh)
        header = next(reader, None)
        if not header:
            return {}
        # header[0] is time; remaining columns are channels in enable order.
        chan_cols = header[1:]
        last = [None] * len(chan_cols)
        edges = [0] * len(chan_cols)
        first_edge = [None] * len(chan_cols)
        last_edge = [None] * len(chan_cols)
        for row in reader:
            if not row:
                continue
            try:
                t = float(row[0])
            except ValueError:
                continue
            for i, cell in enumerate(row[1:len(chan_cols) + 1]):
                v = cell.strip()
                if v == "":
                    continue
                if last[i] is not None and v != last[i]:
                    edges[i] += 1
                    if first_edge[i] is None:
                        first_edge[i] = t
                    last_edge[i] = t
                last[i] = v
    result = {}
    for i, name in enumerate(chan_cols):
        span = (last_edge[i] - first_edge[i]) if first_edge[i] is not None else 0.0
        result[name.strip()] = {"edges": edges[i], "active_span_s": span}
    return result


# ── Pretty summary ──────────────────────────────────────────────────────────
def format_summary(summary, dali_tx=DALI_TX, dali_rx=DALI_RX):
    lines = []
    lines.append(f"Saleae capture @ {summary.get('sample_rate','?')} S/s")
    mb = summary.get("modbus", {})
    for label in ("modbus_rx", "modbus_tx"):
        frames = mb.get(label, [])
        lines.append(f"  {label}: {len(frames)} frame(s)")
        for fr in frames[:12]:
            lines.append("    " + " ".join(f"{b:02X}" for b in fr))
    ln = summary.get("lines", {})
    if ln:
        lines.append("  line activity (edges / active span):")
        for chan, info in ln.items():
            lines.append(f"    {chan}: {info['edges']} edges, {info['active_span_s']*1e3:.2f} ms")
        # Highlight the DALI verdict.
        tx = _chan_info(ln, dali_tx)
        rx = _chan_info(ln, dali_rx)
        if tx is not None:
            verdict = "TOGGLING" if tx["edges"] > 2 else "IDLE (no forward frame!)"
            lines.append(f"  → DALI TX (D{dali_tx}): {verdict}")
        if rx is not None:
            verdict = "response seen" if rx["edges"] > 2 else "NO response edges"
            lines.append(f"  → DALI RX (D{dali_rx}): {verdict}")
    if summary.get("sal"):
        lines.append(f"  saved capture: {summary['sal']}")
    for k in ("save_error", "serial_error", "digital_error"):
        if summary.get(k):
            lines.append(f"  {k}: {summary[k]}")
    return "\n".join(lines)


def _chan_info(lines_dict, ch_idx):
    # Exported column names are like "Channel 0" / "D0"; match the index.
    for name, info in lines_dict.items():
        digits = "".join(c for c in name if c.isdigit())
        if digits and int(digits) == ch_idx:
            return info
    return None


# ── Standalone CLI ──────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="Saleae Logic 2 capture of the DALI/RS-485 bench lines")
    ap.add_argument("--seconds", type=float, default=2.0, help="capture duration")
    ap.add_argument("--baud", type=int, default=19200, help="RS-485 / Modbus baud")
    ap.add_argument("--port", type=int, default=10430, help="automation server port")
    ap.add_argument("--out", default=None, help="output directory (default: temp)")
    ap.add_argument("--rate", type=int, default=None, help="force a digital sample rate")
    cfg = ap.parse_args()

    cap = SaleaeCapture(baud=cfg.baud, port=cfg.port, out_dir=cfg.out,
                        sample_rate=cfg.rate, duration=cfg.seconds)
    try:
        cap.start()
        print(f"Capturing {cfg.seconds:.1f}s on channels {cap.channels} @ {cap.sample_rate} S/s…")
        summary = cap.stop_and_export("standalone")
    except SaleaeError as e:
        sys.exit(f"Saleae error: {e}")
    finally:
        cap.close()
    print(format_summary(summary))


if __name__ == "__main__":
    main()
