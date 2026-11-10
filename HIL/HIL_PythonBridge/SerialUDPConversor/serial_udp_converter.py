"""
serial_udp_converter.py - Conversor Universal Serial <-> UDP para X-Plane
HIL (Hardware-in-the-Loop) generico e parametrizavel via JSON.

Este script conecta-se ao X-Plane (via plugin XPlaneConnect na porta UDP 49009)
e a uma ou mais portas seriais de microcontroladores (MCU).
Todas as configuracoes (portas, taxas, DataRefs, multiplicadores, headers)
sao fornecidas exclusivamente pelo arquivo de configuracao .json.
"""

import argparse
import csv
import json
import math
import os
import socket
import struct
import sys
import time
from typing import Any, Dict, List, Optional

try:
    import serial
except ImportError:
    serial = None


class XPCClient:
    """Cliente UDP autonomo compativel com protocolo XPlaneConnect (XPC)."""

    def __init__(self, host: str = "127.0.0.1", port: int = 49009, timeout: float = 0.05):
        self.dest = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass

    def get_drefs(self, drefs: List[str]) -> List[float]:
        if not drefs:
            return []
        buf = struct.pack("<4sxB", b"GETD", len(drefs))
        for d in drefs:
            b_d = d.encode("ascii")
            buf += struct.pack(f"<B{len(b_d)}s", len(b_d), b_d)
        try:
            self.sock.sendto(buf, self.dest)
            resp, _ = self.sock.recvfrom(4096)
        except Exception:
            return [0.0] * len(drefs)

        if len(resp) < 6 or resp[:4] != b"RESP":
            return [0.0] * len(drefs)

        count = struct.unpack_from("B", resp, 5)[0]
        offset = 6
        values = []
        for _ in range(count):
            if offset >= len(resp):
                break
            row_len = struct.unpack_from("B", resp, offset)[0]
            offset += 1
            row = struct.unpack_from(f"<{row_len}f", resp, offset)
            offset += row_len * 4
            values.append(row[0] if len(row) > 0 else 0.0)
        while len(values) < len(drefs):
            values.append(0.0)
        return values

    def send_drefs(self, dref_values: List[tuple]):
        """Envia lista de tuplas (dataref_str, value_or_list)."""
        if not dref_values:
            return
        buf = struct.pack("<4sx", b"DREF")
        for dref, val in dref_values:
            b_d = dref.encode("ascii")
            if isinstance(val, (list, tuple)):
                fmt = f"<B{len(b_d)}sB{len(val)}f"
                buf += struct.pack(fmt, len(b_d), b_d, len(val), *[float(x) for x in val])
            else:
                fmt = f"<B{len(b_d)}sBf"
                buf += struct.pack(fmt, len(b_d), b_d, 1, float(val))
        try:
            self.sock.sendto(buf, self.dest)
        except Exception:
            pass


def update_status(status_file_path: str, running: bool, status: int, message: str, elapsed: float, duration: float):
    data = {
        "running": running,
        "status": status,
        "message": message,
        "elapsed_sec": round(elapsed, 2),
        "duration_sec": round(duration, 2),
        "timestamp": time.time(),
    }
    try:
        tmp_path = status_file_path + ".tmp"
        with open(tmp_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        os.replace(tmp_path, status_file_path)
    except Exception:
        pass


def wrap_to_pi(val: float) -> float:
    return math.fmod(val + math.pi, 2.0 * math.pi) - math.pi


def run_converter(config_path: str):
    if not os.path.isfile(config_path):
        print(f"[ERRO] Arquivo de configuracao nao encontrado: {config_path}", file=sys.stderr)
        sys.exit(1)

    base_dir = os.path.dirname(os.path.abspath(config_path))
    with open(config_path, "r", encoding="utf-8") as f:
        cfg_all = json.load(f)

    cfg = cfg_all.get("config", {})
    xp_ip = cfg.get("xplane_ip", "127.0.0.1")
    xp_port = int(cfg.get("xplane_port", 49009))
    duration = float(cfg.get("duration_sec", 80.0))
    rate_hz = float(cfg.get("freq_xplane_to_mcu_hz", 20.0))
    dt_target = 1.0 / max(1.0, rate_hz)

    status_filename = cfg.get("status_file", "converter_status.json")
    status_path = os.path.join(base_dir, status_filename) if not os.path.isabs(status_filename) else status_filename

    log_filename = cfg.get("log_file", "converter_u_log.csv")
    log_path = os.path.join(base_dir, log_filename) if not os.path.isabs(log_filename) else log_filename

    if serial is None:
        err = "Pacote 'pyserial' nao instalado no ambiente Python."
        print(f"[ERRO] {err}", file=sys.stderr)
        update_status(status_path, False, 0, err, 0.0, duration)
        sys.exit(1)

    update_status(status_path, True, -1, "Abrindo portas e conectando ao X-Plane...", 0.0, duration)

    # 1) Abrir conexao UDP com X-Plane
    xpc = XPCClient(host=xp_ip, port=xp_port, timeout=0.04)

    # 2) Abrir portas seriais (com DTR/RTS desativados para prevenir reset do MCU)
    serials: Dict[str, serial.Serial] = {}
    serial_cfgs = cfg.get("serial_ports", [])
    if isinstance(serial_cfgs, dict):
        serial_cfgs = [serial_cfgs]
    try:
        for p_info in serial_cfgs:
            if not isinstance(p_info, dict):
                continue
            p_name = p_info.get("name", "COM9")
            baud = int(p_info.get("baudrate", 115200))
            to = float(p_info.get("timeout", 0.05))
            use_dtr = bool(p_info.get("dtr", False))
            use_rts = bool(p_info.get("rts", False))

            s = serial.Serial()
            s.port = p_name
            s.baudrate = baud
            s.timeout = to
            s.dtr = use_dtr
            s.rts = use_rts
            s.open()
            s.reset_input_buffer()
            s.reset_output_buffer()
            serials[p_name] = s
            p_id = p_info.get("id")
            if p_id:
                serials[p_id] = s
            print(f"[OK] Porta serial aberta com sucesso: {p_name} @ {baud} bps (DTR={use_dtr})", flush=True)
    except Exception as e:
        err = f"Falha ao abrir porta serial: {e}"
        print(f"[ERRO] {err}", file=sys.stderr)
        update_status(status_path, False, 0, err, 0.0, duration)
        xpc.close()
        sys.exit(1)

    # 3) Preparar mapeamento XPlane -> MCU
    xp_to_mcu_list = cfg_all.get("xplane_to_mcu", [])
    if isinstance(xp_to_mcu_list, dict):
        xp_to_mcu_list = [xp_to_mcu_list]
    offsets_at_start: Dict[str, float] = {}

    # 4) Preparar mapeamento MCU -> XPlane
    mcu_to_xp_list = cfg_all.get("mcu_to_xplane", [])
    if isinstance(mcu_to_xp_list, dict):
        mcu_to_xp_list = [mcu_to_xp_list]

    # 5) Preparar arquivo de log CSV
    log_f = open(log_path, "w", newline="", encoding="utf-8")
    log_writer = csv.writer(log_f)
    log_writer.writerow(["time", "throttle", "elevator", "aileron", "rudder"])

    print(f"[CONVERSOR] Iniciando transmissao deterministica a {rate_hz:.1f} Hz (duracao: {duration:.1f} s)...", flush=True)
    update_status(status_path, True, -1, "Executando conversao HIL...", 0.0, duration)

    t_start = time.perf_counter()
    next_tick = t_start
    last_status_write = t_start

    try:
        while True:
            t_now = time.perf_counter()
            elapsed = t_now - t_start
            if elapsed >= duration:
                break

            # A) Processar canais XPlane -> MCU
            for ch in xp_to_mcu_list:
                if not isinstance(ch, dict):
                    continue
                p_name = ch.get("port", "COM9")
                ser = serials.get(p_name)
                if not ser or not ser.is_open:
                    continue

                fields = ch.get("fields", [])
                if isinstance(fields, dict):
                    fields = [fields]
                hdr = bytes(ch.get("header_bytes", [170, 85]))

                # Coletar DataRefs necessários
                drefs_to_query = []
                dref_indices = []
                for idx, fld in enumerate(fields):
                    if "dataref" in fld:
                        drefs_to_query.append(fld["dataref"])
                        dref_indices.append(idx)

                queried_vals = xpc.get_drefs(drefs_to_query) if drefs_to_query else []

                # Montar vetor final de floats
                float_vals = []
                q_ptr = 0
                for idx, fld in enumerate(fields):
                    if "value" in fld:
                        v = float(fld["value"])
                    elif "dataref" in fld:
                        v = queried_vals[q_ptr] if q_ptr < len(queried_vals) else 0.0
                        q_ptr += 1

                        # Tratar offset_at_start (referencia relativa a t=0)
                        d_name = fld["dataref"]
                        if fld.get("offset_at_start", False):
                            if d_name not in offsets_at_start:
                                offsets_at_start[d_name] = v
                            v = v - offsets_at_start[d_name]

                        # Tratar scale
                        scale = float(fld.get("scale", 1.0))
                        v = v * scale

                        # Tratar wrap_pi
                        if fld.get("wrap_pi", False):
                            v = wrap_to_pi(v)
                    else:
                        v = 0.0
                    float_vals.append(v)

                # Empacotar header + N floats LE (4 bytes cada)
                payload = hdr + struct.pack(f"<{len(float_vals)}f", *float_vals)
                ser.write(payload)

            # B) Processar canais MCU -> XPlane
            for ch in mcu_to_xp_list:
                if not isinstance(ch, dict):
                    continue
                p_name = ch.get("port", "COM9")
                ser = serials.get(p_name)
                if not ser or not ser.is_open:
                    continue

                fields = ch.get("fields", [])
                if isinstance(fields, dict):
                    fields = [fields]
                hdr = bytes(ch.get("header_bytes", [85, 170]))
                expected_payload_len = len(fields) * 4
                total_expected = len(hdr) + expected_payload_len

                if ser.in_waiting >= total_expected:
                    buf = ser.read(total_expected)
                    if len(buf) == total_expected and buf[:len(hdr)] == hdr:
                        raw_floats = struct.unpack(f"<{len(fields)}f", buf[len(hdr):])
                        drefs_to_send = []
                        log_row = [f"{elapsed:.3f}"]

                        for fld, raw_val in zip(fields, raw_floats):
                            val = float(raw_val) * float(fld.get("scale", 1.0))
                            if "clamp" in fld:
                                c_min, c_max = fld["clamp"]
                                val = max(float(c_min), min(float(c_max), val))

                            log_row.append(f"{val:.4f}")
                            d_name = fld.get("dataref")
                            if d_name:
                                if fld.get("is_array8", False):
                                    drefs_to_send.append((d_name, [val] * 8))
                                else:
                                    drefs_to_send.append((d_name, val))

                        if drefs_to_send:
                            xpc.send_drefs(drefs_to_send)
                        log_writer.writerow(log_row)

            # Atualizar status JSON a cada 1 segundo
            if t_now - last_status_write >= 1.0:
                log_f.flush()
                last_status_write = t_now
                update_status(status_path, True, -1, f"Executando HIL: {elapsed:.1f}s / {duration:.1f}s", elapsed, duration)

            # Manter taxa deterministica
            next_tick += dt_target
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[INFO] Conversor interrompido pelo usuario.", flush=True)
    finally:
        log_f.close()
        for s in set(serials.values()):
            try:
                s.close()
            except Exception:
                pass
        xpc.close()

    total_time = time.perf_counter() - t_start
    print(f"[OK] Conversor HIL finalizado com sucesso ({total_time:.1f} s).", flush=True)
    update_status(status_path, False, 1, "Execucao concluida com sucesso", total_time, duration)


def main():
    parser = argparse.ArgumentParser(description="Conversor Universal Serial <-> UDP X-Plane")
    parser.add_argument("--config", type=str, default="config_converter.json", help="Caminho para arquivo .json de configuracao")
    args = parser.parse_args()

    cfg_file = args.config
    if not os.path.isabs(cfg_file):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        cfg_file = os.path.join(script_dir, cfg_file)

    run_converter(cfg_file)


if __name__ == "__main__":
    main()
