#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rk.py -- minimal serial console driver for the RK3568 board (COM7 @ 1500000).

Usage:
    python rk.py "cmd1" "cmd2" ...       run commands, print merged output
    python rk.py --script file.sh        run a local shell script on the board
    python rk.py --put  local remote     upload a file (base64 over the console)
    python rk.py --get  remote local     download a file (base64 over the console)
    python rk.py --interactive           raw passthrough (Ctrl-] to quit)

Why not pySerial: not installed. We use the Win32 serial API through ctypes,
which is enough for line-oriented shell use.
"""
import ctypes
import ctypes.wintypes as wt
import os
import sys
import time
import base64
import argparse

PORT = os.environ.get("RK_PORT", "COM7")
BAUD = int(os.environ.get("RK_BAUD", "1500000"))

GENERIC_READ = 0x80000000
GENERIC_WRITE = 0x40000000
OPEN_EXISTING = 3
INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value

kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)

kernel32.CreateFileW.restype = wt.HANDLE
kernel32.CreateFileW.argtypes = [wt.LPCWSTR, wt.DWORD, wt.DWORD, ctypes.c_void_p,
                                 wt.DWORD, wt.DWORD, wt.HANDLE]


class DCB(ctypes.Structure):
    _fields_ = [
        ("DCBlength", wt.DWORD), ("BaudRate", wt.DWORD),
        ("fBinary", wt.DWORD, 1), ("fParity", wt.DWORD, 1),
        ("fOutxCtsFlow", wt.DWORD, 1), ("fOutxDsrFlow", wt.DWORD, 1),
        ("fDtrControl", wt.DWORD, 2), ("fDsrSensitivity", wt.DWORD, 1),
        ("fTXContinueOnXoff", wt.DWORD, 1), ("fOutX", wt.DWORD, 1),
        ("fInX", wt.DWORD, 1), ("fErrorChar", wt.DWORD, 1),
        ("fNull", wt.DWORD, 1), ("fRtsControl", wt.DWORD, 2),
        ("fAbortOnError", wt.DWORD, 1), ("fDummy2", wt.DWORD, 17),
        ("wReserved", wt.WORD), ("XonLim", wt.WORD), ("XoffLim", wt.WORD),
        ("ByteSize", ctypes.c_ubyte), ("Parity", ctypes.c_ubyte),
        ("StopBits", ctypes.c_ubyte), ("XonChar", ctypes.c_char),
        ("XoffChar", ctypes.c_char), ("ErrorChar", ctypes.c_char),
        ("EofChar", ctypes.c_char), ("EvtChar", ctypes.c_char),
        ("wReserved1", wt.WORD),
    ]


class COMMTIMEOUTS(ctypes.Structure):
    _fields_ = [("ReadIntervalTimeout", wt.DWORD),
                ("ReadTotalTimeoutMultiplier", wt.DWORD),
                ("ReadTotalTimeoutConstant", wt.DWORD),
                ("WriteTotalTimeoutMultiplier", wt.DWORD),
                ("WriteTotalTimeoutConstant", wt.DWORD)]


class Serial:
    def __init__(self, port=PORT, baud=BAUD):
        self.h = kernel32.CreateFileW(port, GENERIC_READ | GENERIC_WRITE,
                                      0, None, OPEN_EXISTING, 0, None)
        if self.h == INVALID_HANDLE_VALUE or self.h is None:
            raise OSError("cannot open %s: %s" % (port, ctypes.get_last_error()))
        dcb = DCB()
        dcb.DCBlength = ctypes.sizeof(DCB)
        if not kernel32.GetCommState(self.h, ctypes.byref(dcb)):
            raise OSError("GetCommState failed")
        dcb.BaudRate = baud
        dcb.ByteSize = 8
        dcb.Parity = 0
        dcb.StopBits = 0
        dcb.fBinary = 1
        dcb.fOutxCtsFlow = 0
        dcb.fOutxDsrFlow = 0
        dcb.fDtrControl = 1
        dcb.fRtsControl = 1
        dcb.fOutX = 0
        dcb.fInX = 0
        dcb.fAbortOnError = 0
        if not kernel32.SetCommState(self.h, ctypes.byref(dcb)):
            raise OSError("SetCommState failed")
        to = COMMTIMEOUTS(1, 0, 100, 0, 200)
        kernel32.SetCommTimeouts(self.h, ctypes.byref(to))
        kernel32.SetupComm(self.h, 1 << 16, 1 << 16)
        kernel32.PurgeComm(self.h, 0x0004 | 0x0008)  # RXABORT|TXABORT... clear

    def write(self, data: bytes):
        n = wt.DWORD(0)
        buf = ctypes.create_string_buffer(data, len(data))
        kernel32.WriteFile(self.h, buf, len(data), ctypes.byref(n), None)
        return n.value

    def read_avail(self, want: int = 65536) -> bytes:
        buf = ctypes.create_string_buffer(want)
        n = wt.DWORD(0)
        ok = kernel32.ReadFile(self.h, buf, want, ctypes.byref(n), None)
        if not ok:
            return b""
        return buf.raw[: n.value]

    def read_until_quiet(self, quiet=0.45, max_wait=180.0, sink=None):
        """Read until no new bytes for `quiet` seconds."""
        out = bytearray()
        t0 = time.time()
        last = time.time()
        while time.time() - t0 < max_wait:
            chunk = self.read_avail()
            if chunk:
                out += chunk
                last = time.time()
                if sink is not None:
                    try:
                        sink.write(chunk)
                    except TypeError:
                        sink.write(chunk.decode("utf-8", "replace"))
                    sink.flush()
            else:
                if time.time() - last > quiet:
                    break
                time.sleep(0.02)
        return bytes(out)

    def sync(self, timeout=3.0):
        """Flush pending output and make sure we're at a prompt."""
        kernel32.PurgeComm(self.h, 0x0004 | 0x0008)
        self.write(b"\r\n")
        time.sleep(0.3)
        self.read_until_quiet(quiet=0.4, max_wait=timeout)

    def close(self):
        if self.h and self.h != INVALID_HANDLE_VALUE:
            kernel32.CloseHandle(self.h)
            self.h = None


MARK = "__RK_DONE_%d__"


class Board:
    def __init__(self, **kw):
        self.s = Serial(**kw)

    def close(self):
        self.s.close()

    def write_paced(self, data: bytes, delay=0.015, group=8):
        """Write to the tty line-by-line.

        The board's tty is in canonical mode with a MAX_CANON line limit
        (typically 4096 B); a single huge write would silently drop
        characters.  Sending whole lines keeps every canonical buffer
        short, so nothing is lost.
        """
        lines = data.split(b"\n")
        buf = []
        for i, ln in enumerate(lines):
            last = (i == len(lines) - 1)
            buf.append(ln if last else ln + b"\n")
            if len(buf) >= group:
                self.s.write(b"".join(buf))
                buf = []
                time.sleep(delay)
        if buf:
            self.s.write(b"".join(buf))
            time.sleep(delay)

    def run(self, cmd: str, quiet=0.45, max_wait=180.0, echo=True, paced=False):
        """Run one command; return (output_text, exit_code_or_None)."""
        tag = MARK % (int(time.time() * 1000) % 1000000)
        # The status echo must live on its OWN line: appending " ; echo ..."
        # to a multi-line command (e.g. a heredoc) would corrupt its last line
        # and break the terminator.
        body = cmd if cmd.endswith("\n") else cmd + "\n"
        full = body + "echo %s$?\n" % tag
        if echo:
            sys.stderr.write("\n$ %s\n" % cmd[:200])
            sys.stderr.flush()
        payload = full.encode("utf-8", "replace")
        if paced or len(payload) > 512:
            self.write_paced(payload)
        else:
            self.s.write(payload)
        raw = self.s.read_until_quiet(quiet=quiet, max_wait=max_wait,
                                      sink=sys.stdout.buffer if echo else None)
        text = raw.decode("utf-8", "replace")
        # strip ANSI
        import re
        text = re.sub(r"\x1b\[[0-9;?]*[a-zA-Z]|\x1b\][^\x07]*\x07", "", text)
        # drop the terminal echo of the command we just sent
        first_nl = text.find("\n")
        if first_nl >= 0 and tag in text[:first_nl]:
            text = text[first_nl + 1:]
        code = None
        idx = text.rfind(tag)
        if idx >= 0:
            tail = text[idx + len(tag):].strip()
            digits = ""
            for ch in tail:
                if ch.isdigit():
                    digits += ch
                else:
                    break
            if digits:
                code = int(digits)
            text = text[:idx]
        text = text.replace(cmd, "", 1) if cmd in text else text
        return text.strip("\r\n"), code

    def put(self, local, remote, chunk=768):
        """Upload via base64 blocks appended with >> (text-safe, console-safe)."""
        data = open(local, "rb").read()
        b64 = base64.b64encode(data).decode()
        self.run("rm -f %s.b64" % remote, echo=False, quiet=0.2)
        for i in range(0, len(b64), chunk):
            piece = b64[i:i + chunk]
            self.run("printf '%%s' %s >> %s.b64" % (piece, remote),
                     echo=False, quiet=0.12)
        self.run("base64 -d %s.b64 > %s && rm -f %s.b64 && ls -l %s"
                 % (remote, remote, remote, remote), echo=False)
        return len(data)

    def get(self, remote, local, chunk=200000):
        """Download a file by base64-dumping it over the console.

        Echo is disabled on the board side so the transfer is clean; the
        payload is emitted in `chunk`-sized base64 pieces between markers.
        """
        import re as _re
        self.s.write(b"stty -echo -onlcr 2>/dev/null\n")
        time.sleep(0.4)
        self.s.read_until_quiet(quiet=0.3, max_wait=3)
        total = self.run("wc -c < %s" % remote, echo=False, quiet=0.4)[0].strip()
        try:
            nbytes = int(total.split()[0])
        except Exception:
            raise OSError("cannot stat %s on board: %r" % (remote, total))
        nb64 = ((nbytes + 2) // 3) * 4
        pieces = []
        for off in range(0, nb64, chunk):
            tag = "ZZ%d" % (int(time.time() * 1000) % 100000)
            self.s.write(("echo %s_S\ndd if=%s bs=1 skip=%d count=%d 2>/dev/null "
                          "| base64 -w0\necho\necho %s_E\n"
                          % (tag, remote, off, chunk, tag)).encode())
            raw = self.s.read_until_quiet(quiet=0.5, max_wait=90)
            text = raw.decode("utf-8", "replace")
            text = _re.sub(r"\x1b\[[0-9;?]*[a-zA-Z]|\x1b\][^\x07]*\x07", "", text)
            si = text.find(tag + "_S")
            ei = text.find(tag + "_E")
            if si < 0 or ei < 0:
                raise OSError("marker lost at offset %d" % off)
            body = text[si + len(tag) + 2:ei]
            body = "".join(ch for ch in body if ch.isalnum() or ch in "+/=")
            pieces.append(body)
        self.s.write(b"stty echo onlcr 2>/dev/null\n")
        time.sleep(0.3)
        self.s.read_until_quiet(quiet=0.3, max_wait=3)
        blob = "".join(pieces)
        data = base64.b64decode(blob)
        data = data[:nbytes]
        open(local, "wb").write(data)
        return len(data)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmds", nargs="*")
    ap.add_argument("--script")
    ap.add_argument("--put", nargs=2, metavar=("LOCAL", "REMOTE"))
    ap.add_argument("--get", nargs=2, metavar=("REMOTE", "LOCAL"))
    ap.add_argument("--quiet", type=float, default=0.45)
    ap.add_argument("--timeout", type=float, default=180.0)
    args = ap.parse_args()

    b = Board()
    try:
        b.s.sync()
        if args.put:
            n = b.put(args.put[0], args.put[1])
            print("[put] %d bytes -> %s" % (n, args.put[1]))
        if args.get:
            n = b.get(args.get[0], args.get[1])
            print("[get] %d bytes <- %s" % (n, args.get[0]))
        if args.script:
            txt = open(args.script, "r", encoding="utf-8").read()
            out, code = b.run("bash -s <<'RKEOF'\n" + txt + "\nRKEOF",
                              quiet=args.quiet, max_wait=args.timeout, paced=True)
            print(out)
            if code is not None:
                print("[exit %d]" % code)
        for c in args.cmds:
            out, code = b.run(c, quiet=args.quiet, max_wait=args.timeout)
            print(out)
            if code is not None and code != 0:
                print("[exit %d]" % code)
    finally:
        b.close()


if __name__ == "__main__":
    main()
