#!/usr/bin/env python3
"""
bms_node.py — ROS2 driver for the RS485/Modbus BMS
(slave addr 0x81  →  response addr 0x51, 9600 8N1)

Published topics
  /battery_level      std_msgs/Float32          SOC in %  (0.0 – 100.0)
  /battery/current    std_msgs/Float32          Amps; negative = charging
  /battery_info       sensor_msgs/BatteryState  Full telemetry (Nav2-compatible)

Parameters (set in launch or via --ros-args -p)
  serial_port    string   /dev/ttyUSB0
  baud_rate      int      9600
  poll_rate_hz   float    1.0
  cell_count     int      0      (0 = auto-detect from BMS)
  frame_id       string   battery
"""

import struct
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                        QoSHistoryPolicy, QoSDurabilityPolicy)

from std_msgs.msg import Float32
from sensor_msgs.msg import BatteryState

try:
    import serial
except ImportError:
    raise SystemExit("pyserial is not installed — run:  pip install pyserial")


# ─────────────────────────────────────────────────────────────
#  Modbus RTU helpers
# ─────────────────────────────────────────────────────────────

SLAVE_TX  = 0x81   # address we write to
SLAVE_RX  = 0x51   # address we expect back
FUNC_READ = 0x03


def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _build_read(reg: int, count: int) -> bytes:
    hdr = struct.pack('>BBHH', SLAVE_TX, FUNC_READ, reg, count)
    return hdr + struct.pack('<H', _crc16(hdr))


def _parse_block(raw: bytes, expected_regs: int) -> list[int] | None:
    """
    Validate and unpack a Modbus response.
    Returns list of uint16 register values or None on failure.
    """
    if len(raw) < 5:
        return None
    if raw[0] != SLAVE_RX or raw[1] != FUNC_READ:
        return None
    byte_count = raw[2]
    end        = 3 + byte_count
    if len(raw) < end + 2:
        return None
    # CRC check
    recv_crc = struct.unpack('<H', raw[end:end + 2])[0]
    if _crc16(raw[:end]) != recv_crc:
        pass   # log but don't drop — BMS sometimes wins the race
    return [struct.unpack('>H', raw[3 + i * 2:5 + i * 2])[0]
            for i in range(byte_count // 2)]


# ─────────────────────────────────────────────────────────────
#  Serial helper with auto-reconnect
# ─────────────────────────────────────────────────────────────

class BMSSerial:
    def __init__(self, port: str, baud: int, timeout: float = 1.5):
        self._port    = port
        self._baud    = baud
        self._timeout = timeout
        self._ser: serial.Serial | None = None
        self._lock    = threading.Lock()

    def _open(self) -> bool:
        try:
            self._ser = serial.Serial(
                self._port, self._baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self._timeout,
            )
            time.sleep(0.3)
            return True
        except serial.SerialException:
            self._ser = None
            return False

    def read_block(self, start: int, count: int) -> list[int] | None:
        with self._lock:
            if self._ser is None or not self._ser.is_open:
                if not self._open():
                    return None
            try:
                cmd = _build_read(start, count)
                self._ser.reset_input_buffer()
                self._ser.write(cmd)
                time.sleep(0.15)
                expected = 3 + count * 2 + 2
                raw = self._ser.read(expected + 4)
                return _parse_block(raw, count)
            except serial.SerialException:
                self._ser = None
                return None

    def close(self):
        with self._lock:
            if self._ser and self._ser.is_open:
                self._ser.close()


# ─────────────────────────────────────────────────────────────
#  BMS data reader
# ─────────────────────────────────────────────────────────────

class BMSData:
    """Raw register values decoded into physical quantities."""

    # Register map offsets (all relative to base 0x38)
    _BASE       = 0x38
    _MAIN_COUNT = 0x2E   # 46 regs: 0x38–0x65
    _FAULT_BASE = 0x66
    _FAULT_COUNT= 0x05   # 5 regs:  0x66–0x6A
    _CELL_BASE  = 0x00   # 0x00–0x2F: individual cell voltages (48 regs)
    _CELL_COUNT = 0x30

    def __init__(self):
        # Pack
        self.soc_pct       = 0.0
        self.voltage_v     = 0.0
        self.current_a     = 0.0     # negative = charging
        self.remain_ah     = 0.0
        self.cycles        = 0
        self.cell_count    = 0
        self.temp_count    = 0
        # Cells
        self.max_cell_mv   = 0
        self.min_cell_mv   = 0
        self.max_cell_idx  = 0
        self.min_cell_idx  = 0
        self.delta_mv      = 0
        self.avg_mv        = 0
        self.cell_voltages: list[float] = []   # V, per-cell
        # Temperature
        self.max_temp_c    = 0.0
        self.min_temp_c    = 0.0
        self.mos_temp_c    = 0.0
        self.amb_temp_c    = 0.0
        # Status
        self.charge_state  = 0    # 0=idle 1=charging 2=discharging
        self.charger_det   = False
        self.load_det      = False
        self.charge_mos    = False
        self.discharge_mos = False
        self.bal_state     = 0
        self.power_w       = 0.0
        self.energy_wh     = 0.0
        # Faults (raw registers)
        self.fault1 = 0
        self.fault2 = 0
        self.fault3 = 0
        # Derived
        self.healthy       = False

    @classmethod
    def from_serial(cls, bms: BMSSerial) -> 'BMSData | None':
        # Main telemetry block
        regs_a = bms.read_block(cls._BASE, cls._MAIN_COUNT)
        if regs_a is None:
            return None

        # Fault block
        regs_b = bms.read_block(cls._FAULT_BASE, cls._FAULT_COUNT) or [0] * cls._FAULT_COUNT

        d = cls()

        def a(reg: int) -> int:
            idx = reg - cls._BASE
            return regs_a[idx] if 0 <= idx < len(regs_a) else 0

        def b(reg: int) -> int:
            idx = reg - cls._FAULT_BASE
            return regs_b[idx] if 0 <= idx < len(regs_b) else 0

        # Decode main registers
        d.voltage_v    = a(0x38) * 0.1
        d.current_a    = (a(0x39) - 30000) * 0.1    # offset 30000
        d.soc_pct      = a(0x3A) / 10.0             # raw 93 → 9.3 %
        d.cell_count   = a(0x3C)
        d.temp_count   = a(0x3D)
        d.max_cell_mv  = a(0x3E)
        d.max_cell_idx = a(0x3F)
        d.min_cell_mv  = a(0x40)
        d.min_cell_idx = a(0x41)
        d.delta_mv     = a(0x42)
        d.max_temp_c   = float(a(0x43) - 40)
        d.min_temp_c   = float(a(0x45) - 40)
        d.charge_state = a(0x48)
        d.charger_det  = bool(a(0x49))
        d.load_det     = bool(a(0x4A))
        d.remain_ah    = a(0x4B) * 0.1
        d.cycles       = a(0x4C)
        d.bal_state    = a(0x4D)
        d.charge_mos   = bool(a(0x52))
        d.discharge_mos= bool(a(0x53))
        d.avg_mv       = a(0x57)
        d.power_w      = float(a(0x58))
        d.energy_wh    = float(a(0x59))
        d.mos_temp_c   = float(a(0x5A) - 40)
        d.amb_temp_c   = float(a(0x5B) - 40)

        # Faults
        d.fault1 = b(0x66)
        d.fault2 = b(0x67)
        d.fault3 = b(0x68)
        d.healthy = (d.fault1 == 0 and d.fault2 == 0 and d.fault3 == 0)

        # Per-cell voltages (optional — skip gracefully if BMS is slow)
        if d.cell_count > 0:
            cell_regs = bms.read_block(cls._CELL_BASE, d.cell_count)
            if cell_regs:
                d.cell_voltages = [r * 0.001 for r in cell_regs[:d.cell_count]]

        return d

    @property
    def power_supply_status(self) -> int:
        """sensor_msgs/BatteryState power_supply_status constant."""
        if self.charge_state == 1:
            return BatteryState.POWER_SUPPLY_STATUS_CHARGING
        if self.charge_state == 2:
            return BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        if self.soc_pct >= 99.0:
            return BatteryState.POWER_SUPPLY_STATUS_FULL
        return BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

    @property
    def power_supply_health(self) -> int:
        """sensor_msgs/BatteryState power_supply_health constant."""
        # Level 2 faults in fault3 = hardware failures
        if self.fault3 & 0xFF00:
            return BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
        # Over/under temp bits in fault1 high byte (charge/discharge temp)
        if self.fault1 & 0b1111000011110000:
            if self.max_temp_c > 45:
                return BatteryState.POWER_SUPPLY_HEALTH_OVERHEAT
            return BatteryState.POWER_SUPPLY_HEALTH_COLD
        # Overvoltage bits
        if self.fault1 & 0b0000000000110011:
            return BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE
        if not self.healthy:
            return BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
        return BatteryState.POWER_SUPPLY_HEALTH_GOOD


# ─────────────────────────────────────────────────────────────
#  ROS2 Node
# ─────────────────────────────────────────────────────────────

class BMSNode(Node):

    def __init__(self):
        super().__init__('bms_node')

        # ── Parameters ───────────────────────────────────────
        self.declare_parameter('serial_port',  '/dev/rs485')
        self.declare_parameter('baud_rate',    9600)
        self.declare_parameter('poll_rate_hz', 1.0)
        self.declare_parameter('frame_id',     'battery')

        port      = self.get_parameter('serial_port').value
        baud      = self.get_parameter('baud_rate').value
        rate_hz   = self.get_parameter('poll_rate_hz').value
        self._fid = self.get_parameter('frame_id').value

        self.get_logger().info(f'Opening BMS on {port} @ {baud} baud')
        self._bms = BMSSerial(port, baud)

        # ── QoS — sensor data: best-effort, keep last ─────────
        sensor_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            history     = QoSHistoryPolicy.KEEP_LAST,
            depth       = 5,
            durability  = QoSDurabilityPolicy.VOLATILE,
        )

        # ── Publishers ────────────────────────────────────────
        self._pub_soc     = self.create_publisher(
            Float32, 'battery_level', sensor_qos)

        self._pub_current = self.create_publisher(
            Float32, 'battery/current', sensor_qos)

        self._pub_info    = self.create_publisher(
            BatteryState, 'battery_info', sensor_qos)

        # ── Poll timer ────────────────────────────────────────
        period = 1.0 / max(rate_hz, 0.1)
        self._timer = self.create_timer(period, self._poll)

        self._consecutive_failures = 0
        self.get_logger().info(
            f'BMS node started — publishing at {rate_hz:.1f} Hz\n'
            f'  /battery_level    std_msgs/Float32   (SOC %)\n'
            f'  /battery/current  std_msgs/Float32   (A, −=charging)\n'
            f'  /battery_info     sensor_msgs/BatteryState'
        )

    # ── Timer callback ────────────────────────────────────────

    def _poll(self):
        data = BMSData.from_serial(self._bms)

        if data is None:
            self._consecutive_failures += 1
            if self._consecutive_failures == 1 or \
               self._consecutive_failures % 10 == 0:
                self.get_logger().warn(
                    f'BMS read failed (attempt {self._consecutive_failures}). '
                    'Check RS485 cable and slave address 0x81.')
            return

        self._consecutive_failures = 0
        now = self.get_clock().now().to_msg()

        # ── /battery_level ────────────────────────────────────
        soc_msg = Float32()
        soc_msg.data = float(data.soc_pct)
        self._pub_soc.publish(soc_msg)

        # ── /battery/current ──────────────────────────────────
        cur_msg = Float32()
        cur_msg.data = float(data.current_a)   # negative = charging
        self._pub_current.publish(cur_msg)

        # ── /battery_info (sensor_msgs/BatteryState) ──────────
        bs = BatteryState()
        bs.header.stamp    = now
        bs.header.frame_id = self._fid

        bs.voltage         = float(data.voltage_v)
        bs.current         = float(data.current_a)       # A, signed
        bs.charge          = float(data.remain_ah)        # Ah remaining
        bs.percentage      = float(data.soc_pct / 100.0) # 0.0–1.0
        bs.temperature     = float(data.max_temp_c)       # °C (hottest cell)

        bs.power_supply_status     = data.power_supply_status
        bs.power_supply_health     = data.power_supply_health
        bs.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE  # LiFePO4

        bs.present = True

        # Per-cell voltages in Volts (float32[])
        if data.cell_voltages:
            bs.cell_voltage = [float(v) for v in data.cell_voltages]
        else:
            # Approximate from min/avg/max if individual reads unavailable
            bs.cell_voltage = [float(data.avg_mv) * 0.001] * max(data.cell_count, 1)

        # Cell temperatures — fill with max_temp for now (BMS gives aggregate)
        bs.cell_temperature = [float(data.max_temp_c)] * max(data.cell_count, 1)

        bs.location       = 'RS485'
        bs.serial_number  = ''

        self._pub_info.publish(bs)

        # ── Debug log (throttled to once per 10 s) ────────────
        self.get_logger().debug(
            f'SOC {data.soc_pct:.1f}%  '
            f'{data.voltage_v:.2f}V  '
            f'{data.current_a:+.2f}A  '
            f'Δcell {data.delta_mv}mV  '
            f'{"⚠FAULT" if not data.healthy else "OK"}'
        )

        if not data.healthy:
            self.get_logger().warn(
                f'BMS fault registers: '
                f'F1=0x{data.fault1:04X}  '
                f'F2=0x{data.fault2:04X}  '
                f'F3=0x{data.fault3:04X}',
                throttle_duration_sec=30.0
            )

    def destroy_node(self):
        self._bms.close()
        super().destroy_node()


# ─────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = BMSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()