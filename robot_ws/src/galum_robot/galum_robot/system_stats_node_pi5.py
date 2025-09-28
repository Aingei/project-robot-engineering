#!/usr/bin/env python3
#อย่าลืมย้ายไป in pi5
import os, json, time, subprocess, glob
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# psutil อาจไม่เจอ temp บน Ubuntu/บางเคอร์เนล แต่ก็ยังใช้สำหรับ CPU%/MEM ได้ดี
try:
    import psutil
except Exception:
    psutil = None

def read_sysfs_temps():
    """อ่านอุณหภูมิจาก /sys/class/thermal โดย map type -> temp(°C)."""
    temps = {}
    for zpath in glob.glob("/sys/class/thermal/thermal_zone*"):
        try:
            with open(os.path.join(zpath, "type"), "r") as f:
                tname = f.read().strip().lower()
            with open(os.path.join(zpath, "temp"), "r") as f:
                tmc = int(f.read().strip()) / 1000.0
            temps[tname] = tmc
        except Exception:
            continue
    # เลือกที่ “สื่อความหมาย” ที่สุด
    cpu_temp = None
    for key in ("cpu_thermal", "soc_thermal", "cpu-thermal", "soc-thermal", "cpus", "cpu"):
        if key in temps:
            cpu_temp = temps[key]
            break
    # ถ้าไม่พบที่คุ้นเคย ใช้อันแรกเป็น fallback
    if cpu_temp is None and temps:
        cpu_temp = list(temps.values())[0]
    return cpu_temp, temps  # temp เดี่ยว + mapping ทั้งหมด (debug)

def read_cpu_freqs_mhz():
    """อ่านความถี่ CPU ต่อคอร์ (MHz) จาก sysfs; ถ้าไม่พบ คืน []"""
    freqs = []
    for cpath in sorted(glob.glob("/sys/devices/system/cpu/cpu[0-9]*")):
        fpath = os.path.join(cpath, "cpufreq", "scaling_cur_freq")
        try:
            with open(fpath, "r") as f:
                hz = int(f.read().strip())
                freqs.append(hz / 1_000_000.0)  # MHz
        except Exception:
            # บางเคอร์เนล/containers อาจไม่มี cpufreq
            continue
    return freqs

def have_cmd(cmd):
    from shutil import which
    return which(cmd) is not None

def vcgencmd(*args):
    """เรียก vcgencmd ถ้ามี; คืน stdout หรือ None"""
    if not have_cmd("vcgencmd"):
        return None
    try:
        out = subprocess.check_output(["vcgencmd", *args], text=True).strip()
        return out
    except Exception:
        return None

def parse_vcgencmd_throttled(s):
    """
    แปลงสตริงเช่น 'throttled=0x0' เป็น dict ธงต่าง ๆ (ตาม docs ของ RPi)
    อ้างอิง bit (0..31) ที่ใช้บ่อย:
      0: under-voltage
      1: capped
      2: throttled
      16: under-voltage has occurred
      17: capped has occurred
      18: throttled has occurred
    """
    try:
        x = int(s.split('=')[1], 16)
    except Exception:
        return {}
    def bit(n): return 1 if (x & (1 << n)) else 0
    return {
        "raw_hex": hex(x),
        "under_voltage_now": bit(0),
        "freq_capped_now": bit(1),
        "throttled_now": bit(2),
        "under_voltage_ever": bit(16),
        "freq_capped_ever": bit(17),
        "throttled_ever": bit(18),
    }

class SystemStatsNode(Node):
    def __init__(self):
        super().__init__("system_stats_node_pi5")
        # ความถี่ publish (Hz) ปรับได้ผ่านพารามิเตอร์ 'hz'
        self.declare_parameter("hz", 1.0)
        hz = float(self.get_parameter("hz").value)
        self.pub = self.create_publisher(String, "/system/stats", 10)
        self.timer = self.create_timer(1.0/max(hz, 0.1), self.tick)

        # แจ้งว่าพร้อมอ่านแบบไหนได้บ้าง (log ครั้งเดียว)
        if psutil:
            self.get_logger().info("psutil: OK")
        else:
            self.get_logger().warn("psutil: NOT available, falling back to sysfs only")
        if have_cmd("vcgencmd"):
            self.get_logger().info("vcgencmd: available")
        else:
            self.get_logger().info("vcgencmd: not found (optional)")

    def get_stats(self):
        # --- load average ---
        try:
            l1, l5, l15 = os.getloadavg()
        except Exception:
            l1 = l5 = l15 = 0.0

        stats = {
            "ts": time.time(),
            "loadavg": {"1min": l1, "5min": l5, "15min": l15},
            "pi5": {}
        }

        # --- CPU %, per-core %, และ temp ---
        total_percent = 0.0
        per_core_percent = []
        cpu_temp = None
        if psutil:
            try:
                total_percent = psutil.cpu_percent(interval=None)
                per_core_percent = psutil.cpu_percent(interval=None, percpu=True)
            except Exception:
                pass
            try:
                tmp = psutil.sensors_temperatures()
                # หาอันที่น่าจะเป็น CPU / SoC
                for k, entries in tmp.items():
                    for e in entries:
                        label = (e.label or "").lower()
                        if "cpu" in label or "package" in label or k.lower().startswith("cpu"):
                            cpu_temp = float(e.current)
                            break
                    if cpu_temp is not None:
                        break
            except Exception:
                pass

        # ถ้า temp ยังไม่ได้ → ลอง sysfs
        if cpu_temp is None:
            cpu_temp, _all = read_sysfs_temps()

        stats["cpu"] = {
            "total_percent": total_percent,
            "per_core_percent": per_core_percent,
            "temp_c": cpu_temp
        }

        # --- Memory ---
        mem_block = {"total": 0, "available": 0, "used": 0, "percent": 0.0}
        if psutil:
            try:
                vm = psutil.virtual_memory()
                mem_block = {
                    "total": vm.total,
                    "available": vm.available,
                    "used": vm.used,
                    "percent": vm.percent
                }
            except Exception:
                pass
        stats["memory"] = mem_block

        # --- CPU freq ต่อคอร์ (MHz) ---
        freqs = read_cpu_freqs_mhz()
        if freqs:
            stats["pi5"]["cpu_freq_mhz"] = freqs

        # --- vcgencmd (ถ้ามี) ---
        th = vcgencmd("get_throttled")
        if th:
            stats["pi5"]["throttled"] = parse_vcgencmd_throttled(th)
        v = vcgencmd("measure_volts")
        if v:
            # รูปแบบ: 'volt=0.8300V'
            try:
                stats["pi5"]["volts"] = float(v.split('=')[1].strip('V'))
            except Exception:
                stats["pi5"]["volts_raw"] = v
        arm_clk = vcgencmd("measure_clock", "arm")
        if arm_clk:
            # 'frequency(48)=2400000000'
            try:
                hz = int(arm_clk.split('=')[1])
                stats["pi5"]["arm_clock_mhz"] = hz / 1_000_000.0
            except Exception:
                stats["pi5"]["arm_clock_raw"] = arm_clk

        return stats

    def tick(self):
        msg = String()
        msg.data = json.dumps(self.get_stats(), ensure_ascii=False)
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = SystemStatsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
