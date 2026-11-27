"""The Pi's own numbers for the status screen: CPU, memory, temperature, uptime, disk, IP addresses.

Read from Linux's /proc and /sys files (standard library only, nothing to install). The parse_* functions take
the file's text, so the tests run on any laptop; read_*() open the real files and return None where a file is
missing (a laptop, or a container without it).
"""
import fcntl
import shutil
import socket
import struct

from rocko_env.recording.RecordRun import DEFAULT_MIN_FREE_MB


def parse_cpu_times(stat_text):
    """/proc/stat's first line -> (idle, total) jiffies since boot. Idle includes iowait."""
    fields = [int(x) for x in stat_text.splitlines()[0].split()[1:]]
    idle = fields[3] + (fields[4] if len(fields) > 4 else 0)
    return idle, sum(fields)


def cpu_percent(before, after):
    """Busy share [%] of all cores between two parse_cpu_times() samples."""
    idle = after[0] - before[0]
    total = after[1] - before[1]
    return 0.0 if total <= 0 else 100.0 * (total - idle) / total


def parse_meminfo(text):
    """/proc/meminfo -> (used MB, total MB). "Used" = total minus MemAvailable (what programs could still get)."""
    kb = {}
    for line in text.splitlines():
        name, _, rest = line.partition(":")
        if rest.split():
            kb[name] = int(rest.split()[0])
    total = kb.get("MemTotal", 0)
    return (total - kb.get("MemAvailable", 0)) / 1024.0, total / 1024.0


def _read(path):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return None


def read_cpu_times():
    text = _read("/proc/stat")
    return parse_cpu_times(text) if text else None


def read_memory_mb():
    text = _read("/proc/meminfo")
    return parse_meminfo(text) if text else None


def read_temperature_c():
    """The SoC temperature [deg C] (thermal zone 0 on a Pi), or None."""
    text = _read("/sys/class/thermal/thermal_zone0/temp")
    return int(text) / 1000.0 if text and text.strip().isdigit() else None


def read_uptime_s():
    text = _read("/proc/uptime")
    return float(text.split()[0]) if text else None


def read_disk_gb(path="/"):
    """{"used_gb", "total_gb", "free_gb"} of the disk holding `path` (on the Pi, "/" is the SD card), or None.
    free_gb is what an ordinary program may still write; used_gb + free_gb can be a little under total_gb, because
    Linux keeps a few percent back for itself."""
    try:
        usage = shutil.disk_usage(path)
    except OSError:
        return None
    return {"used_gb": usage.used / 1e9, "total_gb": usage.total / 1e9, "free_gb": usage.free / 1e9}


def disk_low(free_gb):
    """True when a disk has less room than the recorder needs: below it, RecordRun.py refuses to start."""
    return free_gb * 1000 < DEFAULT_MIN_FREE_MB


SIOCGIFADDR = 0x8915   # Linux ioctl: "get this interface's IPv4 address"


def ipv4_addresses():
    """{interface: "a.b.c.d"} for every interface with an IPv4 address, except loopback. Linux only."""
    found = {}
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        for _, name in socket.if_nameindex():
            if name == "lo":
                continue
            try:
                packed = fcntl.ioctl(s.fileno(), SIOCGIFADDR, struct.pack("256s", name.encode()[:15]))
            except OSError:
                continue   # the interface is down or has no IPv4 address
            found[name] = socket.inet_ntoa(packed[20:24])
    return found


def format_uptime(seconds):
    if seconds is None:
        return "?"
    minutes = int(seconds // 60)
    return f"{minutes // 60} h {minutes % 60:02d} min"
