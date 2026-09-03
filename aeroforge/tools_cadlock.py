"""Run a command under the machine-wide CAD build lock (one full CAD build
at a time - two thrash memory). Usage:
  .venv/Scripts/python.exe tools_cadlock.py <cmd> [args...]
Blocks until the lock is free (polls every 15 s), then runs the command
and releases. The lock is `aeroforge/.cadlock`; the holder refreshes its
mtime every minute and a stale lock older than 90 min is broken."""
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

LOCK = Path(__file__).with_name(".cadlock")


def acquire():
    while True:
        try:
            fd = os.open(LOCK, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
            os.write(fd, f"{os.getpid()} {time.time()}\n".encode())
            os.close(fd)
            return
        except FileExistsError:
            try:
                if time.time() - LOCK.stat().st_mtime > 90 * 60:
                    LOCK.unlink()
                    continue
            except FileNotFoundError:
                continue
            time.sleep(15)


def _keepalive():
    """Touch the lock every minute while the command runs, so a job longer
    than the 90 min stale threshold (a 16-config probe sweep) is not
    mistaken for a dead one and overtaken (measured 2026-08-27: a queued
    probe started beside a running sweep at the 90 min mark)."""
    while True:
        time.sleep(60)
        try:
            os.utime(LOCK, None)
        except OSError:
            return


def main():
    acquire()
    threading.Thread(target=_keepalive, daemon=True).start()
    try:
        rc = subprocess.call(sys.argv[1:])
    finally:
        try:
            LOCK.unlink()
        except FileNotFoundError:
            pass
    sys.exit(rc)


if __name__ == "__main__":
    main()
