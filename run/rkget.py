#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rkget.py -- 可靠地从板子下载文件（base64 dd 分块 + 长度校验 + 自动重传）

r k.py 自带的 --get 在 1.5M 波特下会静默丢字节，导致文件损坏。
本脚本按固定小块传输，每块的长度是确定的，对不上就重传该块。

用法:  python rkget.py <远程路径> <本地路径> [块大小=4096]
"""
import os
import sys
import time
import base64

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rk import Board

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass


def fetch_chunk(b, remote, off, size, tries=12):
    """读取 remote 中 [off, off+size) 的字节，返回 bytes。"""
    want_b64 = ((size + 2) // 3) * 4
    for attempt in range(tries):
        tag = "Q%d" % (int(time.time() * 1000) % 1000000 + attempt)
        cmd = ("echo %s_S; dd if=%s bs=1 skip=%d count=%d 2>/dev/null | base64 -w0; "
               "echo; echo %s_E\n" % (tag, remote, off, size, tag))
        b.s.write(cmd.encode())
        raw = b.s.read_until_quiet(quiet=0.45, max_wait=60)
        text = raw.decode("utf-8", "replace")
        si = text.find(tag + "_S")
        ei = text.find(tag + "_E")
        if si < 0 or ei < 0:
            continue
        body = text[si + len(tag) + 2:ei]
        body = "".join(ch for ch in body if ch.isalnum() or ch in "+/=")
        body = body.strip()
        pad = body.count("=")
        expect = ((size + 2) // 3) * 4
        if len(body) != expect:
            continue  # 长度不符 → 丢了字节，重传
        try:
            data = base64.b64decode(body, validate=True)
        except Exception:
            continue
        if len(data) != size:
            continue
        return data
    raise OSError("chunk @%d+%d failed after %d tries" % (off, size, tries))


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    remote, local = sys.argv[1], sys.argv[2]
    chunk = int(sys.argv[3]) if len(sys.argv) > 3 else 4096

    b = Board()
    try:
        b.s.write(b"\x03")
        time.sleep(1.0)
        b.s.sync(timeout=8)
        # 关掉回显，避免污染传输
        b.s.write(b"stty -echo -onlcr 2>/dev/null\n")
        time.sleep(0.4)
        b.s.read_until_quiet(quiet=0.3, max_wait=3)

        out, _ = b.run("wc -c < %s" % remote, echo=False, quiet=0.5, max_wait=60)
        nums = [t for t in out.replace("\r", "\n").split() if t.isdigit()]
        if not nums:
            raise OSError("cannot stat %s on board: %r" % (remote, out))
        n = int(nums[-1])
        print("[rkget] remote=%s size=%d chunk=%d" % (remote, n, chunk))

        buf = bytearray()
        off = 0
        t0 = time.time()
        while off < n:
            sz = min(chunk, n - off)
            buf += fetch_chunk(b, remote, off, sz)
            off += sz
            pct = 100.0 * off / n
            print("\r  %6.1f%%  (%d/%d)" % (pct, off, n), end="", flush=True)
        print()

        b.s.write(b"stty echo onlcr 2>/dev/null\n")
        time.sleep(0.3)

        os.makedirs(os.path.dirname(os.path.abspath(local)) or ".", exist_ok=True)
        with open(local, "wb") as f:
            f.write(bytes(buf))
        print("[rkget] saved %s (%d bytes, %.1fs)" % (local, len(buf), time.time() - t0))
        return 0
    finally:
        b.close()


if __name__ == "__main__":
    sys.exit(main())
