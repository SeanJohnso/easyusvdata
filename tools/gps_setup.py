"""Set up every u-blox GPS in rig.yaml for recording. Run with the ROS GPS nodes stopped.

    ~/easyusvdata/.venv/bin/python ~/easyusvdata/tools/gps_setup.py [id ...]

For each module: find the baud it is talking at, switch UART1 to 115200 with
UBX + NMEA in and out (PUBX,41, works from factory state), then set 5 Hz, the
sea dynamic model, UBX NAV-PVT every solution, NAV-SAT once a second, NMEA
GGA/RMC/GST only (GSA/GSV/GLL/VTG off), and save. Constellations are left at
the module's factory set (2026-11-22 decision: each module's own best set).

M10/F10 (protocol >= 23) are set with CFG-VALSET (RAM + BBR, then flash if the
module has one); the M8 with the legacy CFG-RATE/-NAV5/-MSG and CFG-CFG to BBR.
BBR is kept only while the board's backup cell is charged, so run this before
each recording session; running it again is harmless. The ROS driver runs with
config_on_startup: false and only listens.

Needs pyserial and pyubx2 (in the venv, see README).
"""
import pathlib
import re
import sys
import time

from pyubx2 import POLL, SET, UBX_PROTOCOL, UBXMessage, UBXReader
import serial
import yaml

RIG = pathlib.Path(__file__).resolve().parents[1] / 'ros2_ws/src/usv_bringup/config/rig.yaml'
TARGET_BAUD = 115200
TRY_BAUDS = (115200, 38400, 9600, 230400)
MEAS_MS = 200          # 5 Hz
SAT_EVERY = 5          # NAV-SAT once per 5 solutions = 1 Hz
SEA = 5                # dynamic model "sea"
NMEA_ON = ('GGA', 'RMC', 'GST')
NMEA_OFF = ('GSA', 'GSV', 'GLL', 'VTG')
NMEA_MSG_ID = {'GGA': 0x00, 'GLL': 0x01, 'GSA': 0x02, 'GSV': 0x03, 'RMC': 0x04, 'VTG': 0x05,
               'GST': 0x07}
NMEA_RE = re.compile(rb'\$([A-Z]{5})([^*\r\n$]*)\*([0-9A-F]{2})')


def nmea_cmd(body):
    cs = 0
    for c in body.encode():
        cs ^= c
    return f'${body}*{cs:02X}\r\n'.encode()


def valid_nmea(data):
    """Sentence names in data whose checksum is right."""
    out = []
    for m in NMEA_RE.finditer(data):
        cs = 0
        for c in m.group(1) + m.group(2):
            cs ^= c
        if cs == int(m.group(3), 16):
            out.append(m.group(1).decode())
    return out


def find_baud(port):
    """First baud with valid NMEA or UBX traffic, or None."""
    for baud in TRY_BAUDS:
        with serial.Serial(port, baud, timeout=0.2) as s:
            s.reset_input_buffer()
            data, end = b'', time.time() + 1.5
            while time.time() < end:
                data += s.read(4096)
        if valid_nmea(data) or count_ubx(data):
            return baud
    return None


def count_ubx(data):
    """Number of UBX frames in data with a correct checksum."""
    n, i = 0, data.find(b'\xb5\x62')
    while 0 <= i and i + 8 <= len(data):
        length = data[i + 4] | data[i + 5] << 8
        frame = data[i + 2:i + 6 + length]
        if len(frame) == 4 + length and i + 8 + length <= len(data):
            a = b = 0
            for c in frame:
                a = (a + c) & 0xFF
                b = (b + a) & 0xFF
            if bytes((a, b)) == data[i + 6 + length:i + 8 + length]:
                n += 1
        i = data.find(b'\xb5\x62', i + 2)
    return n


def send(s, msg, timeout=1.5):
    """Write msg, return 'ACK', 'NAK' or 'no reply'."""
    cls, mid = msg.serialize()[2:4]
    s.reset_input_buffer()
    s.write(msg.serialize())
    rdr = UBXReader(s, protfilter=UBX_PROTOCOL)
    end = time.time() + timeout
    while time.time() < end:
        try:
            _, p = rdr.read()
        except Exception:
            continue
        if p is not None and p.identity in ('ACK-ACK', 'ACK-NAK') \
                and p.clsID == cls and p.msgID == mid:
            return 'ACK' if p.identity == 'ACK-ACK' else 'NAK'
    return 'no reply'


def mon_ver(s, timeout=1.5):
    """(module, protocol version) from MON-VER, or None."""
    s.reset_input_buffer()
    s.write(UBXMessage('MON', 'MON-VER', POLL).serialize())
    rdr = UBXReader(s, protfilter=UBX_PROTOCOL)
    end = time.time() + timeout
    while time.time() < end:
        try:
            _, p = rdr.read()
        except Exception:
            continue
        if p is not None and p.identity == 'MON-VER':
            ext = [v.decode(errors='ignore').strip('\x00') for k, v in vars(p).items()
                   if k.startswith('extension')]
            prot = next((float(e.split('=')[1]) for e in ext if e.startswith('PROTVER=')), 0.0)
            mod = next((e.split('=')[1] for e in ext if e.startswith('MOD=')), '?')
            sw = p.swVersion.decode(errors='ignore').strip('\x00')
            return f'{mod} ({sw})', prot
    return None


def config_valset(s):
    """M10/F10: one CFG-VALSET to RAM + BBR, then the same to flash if present."""
    nmea = [(f'CFG_MSGOUT_NMEA_ID_{n}_UART1', 1) for n in NMEA_ON] + \
           [(f'CFG_MSGOUT_NMEA_ID_{n}_UART1', 0) for n in NMEA_OFF]
    cfg = [('CFG_UART1_BAUDRATE', TARGET_BAUD),
           ('CFG_UART1INPROT_UBX', 1), ('CFG_UART1INPROT_NMEA', 1),
           ('CFG_UART1OUTPROT_UBX', 1), ('CFG_UART1OUTPROT_NMEA', 1),
           ('CFG_RATE_MEAS', MEAS_MS), ('CFG_RATE_NAV', 1),
           ('CFG_NAVSPG_DYNMODEL', SEA),
           ('CFG_MSGOUT_UBX_NAV_PVT_UART1', 1),
           ('CFG_MSGOUT_UBX_NAV_SAT_UART1', SAT_EVERY)] + nmea
    ram_bbr = send(s, UBXMessage.config_set(0x03, 0, cfg))
    flash = send(s, UBXMessage.config_set(0x04, 0, cfg)) if ram_bbr == 'ACK' else '-'
    return [f'VALSET RAM+BBR {ram_bbr}', f'flash {flash}'], ram_bbr == 'ACK'


def config_legacy(s):
    """M8: legacy CFG messages, then CFG-CFG save to BBR."""
    steps = [('RATE', UBXMessage('CFG', 'CFG-RATE', SET, measRate=MEAS_MS, navRate=1, timeRef=1)),
             ('NAV5', UBXMessage('CFG', 'CFG-NAV5', SET, dyn=1, dynModel=SEA)),
             ('NAV-PVT', UBXMessage('CFG', 'CFG-MSG', SET, msgClass=0x01, msgID=0x07, rateUART1=1)),
             ('NAV-SAT', UBXMessage('CFG', 'CFG-MSG', SET, msgClass=0x01, msgID=0x35,
                                    rateUART1=SAT_EVERY))]
    for n in NMEA_ON + NMEA_OFF:
        steps.append((n, UBXMessage('CFG', 'CFG-MSG', SET, msgClass=0xF0, msgID=NMEA_MSG_ID[n],
                                    rateUART1=1 if n in NMEA_ON else 0)))
    steps.append(('save BBR', UBXMessage('CFG', 'CFG-CFG', SET, clearMask=b'\x00\x00\x00\x00',
                                         saveMask=b'\x1f\x1f\x00\x00',
                                         loadMask=b'\x00\x00\x00\x00', devBBR=1)))
    results = [(name, send(s, msg)) for name, msg in steps]
    bad = [f'{n} {r}' for n, r in results if r != 'ACK']
    return ([f'{len(results) - len(bad)}/{len(results)} ACK'] + bad), not bad


def verify(port, seconds=3.0):
    """Message counts per second and the last fix at TARGET_BAUD."""
    with serial.Serial(port, TARGET_BAUD, timeout=0.2) as s:
        s.reset_input_buffer()
        data, end = b'', time.time() + seconds
        while time.time() < end:
            data += s.read(8192)
    counts = {}
    for n in valid_nmea(data):
        counts[n[2:]] = counts.get(n[2:], 0) + 1
    fix = ''
    i = data.find(b'\xb5\x62')
    while i >= 0:
        cls, mid = data[i + 2:i + 4]
        name = {(1, 7): 'NAV-PVT', (1, 0x35): 'NAV-SAT'}.get((cls, mid))
        if name:
            counts[name] = counts.get(name, 0) + 1
        if name == 'NAV-PVT' and i + 6 + 92 <= len(data):
            p = data[i + 6:i + 6 + 92]
            fix = f'fixType {p[20]}, numSV {p[23]}, hAcc {int.from_bytes(p[40:44], "little") / 1000:.1f} m'
        i = data.find(b'\xb5\x62', i + 2)
    rates = ', '.join(f'{k} {v / seconds:.1f}/s' for k, v in sorted(counts.items()))
    return rates or 'nothing', fix


def setup(gid, port):
    print(f'== {gid} ({port})')
    baud = find_baud(port)
    if baud is None:
        print('   no valid NMEA/UBX at any baud: check wiring and power')
        return False
    print(f'   talking at {baud}')
    if baud != TARGET_BAUD:
        with serial.Serial(port, baud, timeout=0.2) as s:
            s.write(nmea_cmd(f'PUBX,41,1,0003,0003,{TARGET_BAUD},0'))
            s.flush()
        time.sleep(0.5)
    with serial.Serial(port, TARGET_BAUD, timeout=0.2) as s:
        ver = mon_ver(s)
        if ver is None:
            print('   no MON-VER reply at 115200 (UBX input off, or UART RX disabled: power-cycle it)')
            return False
        print(f'   {ver[0]}, protocol {ver[1]:.2f}')
        lines, ok = config_valset(s) if ver[1] >= 23 else config_legacy(s)
        print('   ' + '; '.join(lines))
    time.sleep(0.5)
    rates, fix = verify(port)
    print(f'   now: {rates}' + (f'; {fix}' if fix else ''))
    return ok


def main():
    with open(RIG) as f:
        gps = yaml.safe_load(f)['gps']
    # Modules on the NMEA driver run in their factory state and aren't set up.
    ids = sys.argv[1:] or [g for g in gps if gps[g].get('driver') != 'nmea']
    results = {gid: setup(gid, gps[gid]['device']) for gid in ids}
    print('\n' + '  '.join(f'{g}: {"OK" if ok else "FAILED"}' for g, ok in results.items()))


if __name__ == '__main__':
    main()
