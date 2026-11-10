"""
test_connection.py - Script de Teste e Diagnostico da Ponte HIL
(X-Plane UDP <---> Conversor Python <---> MCU Serial)

Permite testar de forma isolada e interativa:
1) Comunicacao UDP com o X-Plane (leitura e envio de DataRefs via XPlaneConnect).
2) Comunicacao Serial com o Microcontrolador (Arduino Mega/ESP32 com dtr=False).
3) Protocolo de Waypoints do MCU (envio e ACK [0xBB 0x66]).
4) Protocolo de Sensores/Atuadores do MCU ([0xAA 0x55] -> [0x55 0xAA]).
5) Teste continuo (Dry-Run de 10 segundos) sem precisar abrir o Simulink.

Uso:
    python test_connection.py           (Menu interativo ou teste completo)
    python test_connection.py --xplane  (Apenas teste do X-Plane)
    python test_connection.py --mcu     (Apenas teste do MCU)
    python test_connection.py --loop    (Dry-run do loop HIL em tempo real)
    python test_connection.py --all     (Executa todos os testes sequencialmente)
"""

import argparse
import json
import math
import os
import socket
import struct
import sys
import time

try:
    import serial
except ImportError:
    serial = None


class XPCClient:
    """Cliente UDP autonomo compativel com protocolo XPlaneConnect (XPC)."""

    def __init__(self, host: str = "127.0.0.1", port: int = 49009, timeout: float = 0.5):
        self.dest = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass

    def get_drefs(self, drefs: list) -> list:
        if not drefs:
            return []
        buf = struct.pack("<4sxB", b"GETD", len(drefs))
        for d in drefs:
            b_d = d.encode("ascii")
            buf += struct.pack(f"<B{len(b_d)}s", len(b_d), b_d)
        try:
            self.sock.sendto(buf, self.dest)
            resp, _ = self.sock.recvfrom(4096)
        except Exception as e:
            return None

        if len(resp) < 6 or resp[:4] != b"RESP":
            return None

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
        return values


def load_config(config_path: str) -> dict:
    if not os.path.isabs(config_path):
        this_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(this_dir, config_path)
    if not os.path.isfile(config_path):
        print(f"[AVISO] Arquivo {config_path} nao encontrado. Usando defaults.")
        return {}
    with open(config_path, "r", encoding="utf-8") as f:
        return json.load(f)


def test_xplane(cfg: dict) -> bool:
    print("\n" + "="*60)
    print(" [1/3] TESTE DE COMUNICACAO UDP COM X-PLANE")
    print("="*60)
    c = cfg.get("config", {})
    xp_ip = c.get("xplane_ip", "127.0.0.1")
    xp_port = int(c.get("xplane_port", 49009))

    print(f"Tentando conectar ao X-Plane em {xp_ip}:{xp_port}...")
    xpc = XPCClient(xp_ip, xp_port, timeout=1.0)

    drefs_test = [
        "sim/flightmodel/position/y_agl",
        "sim/flightmodel/position/true_airspeed",
        "sim/flightmodel/position/phi",
        "sim/flightmodel/position/theta",
        "sim/flightmodel/position/psi",
    ]

    t0 = time.perf_counter()
    res = xpc.get_drefs(drefs_test)
    dt_ms = (time.perf_counter() - t0) * 1000.0
    xpc.close()

    if res is None:
        print("[FALHA] Sem resposta do X-Plane!")
        print("  Verifique se:")
        print("  1) O X-Plane esta aberto com um voo carregado.")
        print("  2) O plugin XPlaneConnect (XPC) esta instalado na pasta Resources/plugins.")
        print("  3) A porta UDP 49009 nao esta bloqueada por firewall.")
        return False

    alt, vt, phi, theta, psi = res
    print(f"[SUCESSO] X-Plane respondeu em {dt_ms:.1f} ms:")
    print(f"  - Altitude AGL (y_agl):      {alt:.2f} m")
    print(f"  - Velocidade (true_airspeed): {vt:.2f} m/s ({vt*3.6:.1f} km/h)")
    print(f"  - Roll (phi):                 {phi:.2f} deg")
    print(f"  - Pitch (theta):              {theta:.2f} deg")
    print(f"  - Heading (psi):              {psi:.2f} deg")
    return True


def test_mcu_serial(cfg: dict) -> bool:
    print("\n" + "="*60)
    print(" [2/3] TESTE DE COMUNICACAO SERIAL COM O MCU")
    print("="*60)

    if serial is None:
        print("[ERRO] Biblioteca 'pyserial' nao instalada. Instale com: pip install pyserial")
        return False

    c = cfg.get("config", {})
    s_cfgs = c.get("serial_ports", [])
    if isinstance(s_cfgs, dict):
        s_cfgs = [s_cfgs]

    p_name = "COM9"
    baud = 115200
    if s_cfgs and isinstance(s_cfgs[0], dict):
        p_name = s_cfgs[0].get("name", "COM9")
        baud = int(s_cfgs[0].get("baudrate", 115200))

    print(f"Abrindo porta {p_name} @ {baud} bps (com DTR=False, RTS=False)...")
    try:
        s = serial.Serial()
        s.port = p_name
        s.baudrate = baud
        s.timeout = 1.0
        s.dtr = False
        s.rts = False
        s.open()
        time.sleep(0.5)
        s.reset_input_buffer()
        s.reset_output_buffer()
        print(f"[OK] Porta {p_name} aberta com sucesso.")
    except Exception as e:
        print(f"[FALHA] Nao foi possivel abrir a porta {p_name}: {e}")
        print("  Verifique se a porta esta correta ou se outro programa (Simulink/IDE Arduino) esta usando-a.")
        return False

    # Sub-teste A: Upload de Waypoints [0xBB 0x66]
    print("\n  A) Testando Upload de 2 Waypoints de teste [0xBB 0x66]...")
    wps_test = [
        (0.0, 0.0, 100.0, 15.0),
        (300.0, 0.0, 100.0, 15.0)
    ]
    N = len(wps_test)
    wp_payload = bytearray([187, 102, N])
    for wp in wps_test:
        wp_payload += struct.pack("<4f", *wp)

    s.write(wp_payload)
    ack = s.read(3)
    if len(ack) == 3 and ack[0] == 102 and ack[1] == 187 and ack[2] == N:
        print(f"     [SUCESSO] MCU respondeu com ACK valido [0x66 0xBB {N}]!")
    else:
        print(f"     [AVISO] ACK de waypoints invalido ou ausente (recebido {len(ack)} bytes: {list(ack)}).")
        print("     (Se o firmware ja estiver em loop de controle, pode ser normal ignorar re-upload sem reset).")

    # Sub-teste B: Envio de Sensores [0xAA 0x55] e recepcao de Atuadores [0x55 0xAA]
    print("\n  B) Testando Envio de Sensores [0xAA 0x55] e Resposta de Atuadores [0x55 0xAA]...")
    # 15 floats: [p, q, r, u, v, w, phi, theta, psi, VT, alpha, beta, xN, xE, alt]
    test_sensors = [0.0, 0.0, 0.0, 15.0, 0.0, -1.83, 0.0, 0.0, 0.0, 15.0, 0.0, 0.0, 0.0, 0.0, 100.0]
    sensor_packet = bytearray([170, 85]) + struct.pack("<15f", *test_sensors)

    s.reset_input_buffer()
    t_send = time.perf_counter()
    s.write(sensor_packet)

    # Esperar resposta de 18 bytes: [0x55 0xAA] + 4 floats
    resp = s.read(18)
    dt_serial = (time.perf_counter() - t_send) * 1000.0

    if len(resp) == 18 and resp[0] == 85 and resp[1] == 170:
        thr, ele, ail, rud = struct.unpack("<4f", resp[2:])
        print(f"     [SUCESSO] Resposta recebida em {dt_serial:.1f} ms:")
        print(f"       - Throttle: {thr:.4f}  (nominal ~0.49)")
        print(f"       - Elevator: {ele:.4f}  (nominal ~0.015)")
        print(f"       - Aileron:  {ail:.4f}  (nominal ~0.00)")
        print(f"       - Rudder:   {rud:.4f}  (nominal ~0.00)")
        s.close()
        return True
    else:
        print(f"     [FALHA] MCU nao respondeu pacote de atuadores esperado.")
        print(f"       Bytes recebidos: {len(resp)} -> {list(resp)}")
        s.close()
        return False


def test_loop_dry_run(cfg: dict, duration_sec: float = 5.0):
    print("\n" + "="*60)
    print(f" [3/3] DRY-RUN HIL EM TEMPO REAL ({duration_sec:.0f} SEGUNDOS)")
    print("="*60)
    print("Executando troca real de pacotes X-Plane <-> Python <-> MCU no terminal...\n")

    c = cfg.get("config", {})
    xp_ip = c.get("xplane_ip", "127.0.0.1")
    xp_port = int(c.get("xplane_port", 49009))

    s_cfgs = c.get("serial_ports", [])
    if isinstance(s_cfgs, dict):
        s_cfgs = [s_cfgs]
    p_name = s_cfgs[0].get("name", "COM9") if s_cfgs else "COM9"
    baud = int(s_cfgs[0].get("baudrate", 115200)) if s_cfgs else 115200

    xpc = XPCClient(xp_ip, xp_port, timeout=0.04)

    try:
        s = serial.Serial()
        s.port = p_name
        s.baudrate = baud
        s.timeout = 0.05
        s.dtr = False
        s.rts = False
        s.open()
        time.sleep(0.5)
        s.reset_input_buffer()
        s.reset_output_buffer()
    except Exception as e:
        print(f"[ERRO] Falha ao abrir porta serial: {e}")
        xpc.close()
        return

    # DataRefs a coletar do X-Plane
    drefs = [
        "sim/flightmodel/position/Prad",
        "sim/flightmodel/position/Qrad",
        "sim/flightmodel/position/Rrad",
        "sim/flightmodel/position/true_airspeed",
        "sim/flightmodel/position/phi",
        "sim/flightmodel/position/theta",
        "sim/flightmodel/position/psi",
        "sim/flightmodel/position/local_z",
        "sim/flightmodel/position/local_x",
        "sim/flightmodel/position/y_agl"
    ]

    t_start = time.perf_counter()
    next_tick = t_start
    dt_target = 0.05  # 20 Hz
    step = 0

    print("Tempo(s) | Alt(m) | VT(m/s) | Thr    Elev    Ail     Rud     | Status")
    print("-" * 65)

    try:
        while True:
            t_now = time.perf_counter()
            elapsed = t_now - t_start
            if elapsed >= duration_sec:
                break

            # 1) Ler do X-Plane
            vals = xpc.get_drefs(drefs)
            if vals is None:
                vals = [0.0] * len(drefs)

            p, q, r, u_spd, phi_deg, th_deg, psi_deg, z_pos, x_pos, alt = vals
            deg2rad = 0.0174532925
            phi = phi_deg * deg2rad
            theta = th_deg * deg2rad
            psi = math.fmod(psi_deg * deg2rad + math.pi, 2.0 * math.pi) - math.pi
            xN = -z_pos
            xE = x_pos

            # 2) Montar pacote para MCU
            sensors = [p, q, r, u_spd, 0.0, -1.8343, phi, theta, psi, u_spd, 0.0, 0.0, xN, xE, alt]
            pkt = bytearray([170, 85]) + struct.pack("<15f", *sensors)
            s.write(pkt)

            # 3) Ler atuadores do MCU
            thr, ele, ail, rud = 0.0, 0.0, 0.0, 0.0
            status_str = "OK"
            if s.in_waiting >= 18:
                buf = s.read(18)
                if len(buf) == 18 and buf[0] == 85 and buf[1] == 170:
                    thr, ele, ail, rud = struct.unpack("<4f", buf[2:])
                else:
                    status_str = "SyncErr"
            else:
                status_str = "NoResp"

            if step % 4 == 0:  # Imprime a cada 0.2s (5 Hz) para nao poluir o terminal
                print(f"{elapsed:6.2f}s | {alt:6.1f} | {u_spd:7.1f} | {thr:6.3f}  {ele:6.3f}  {ail:6.3f}  {rud:6.3f} | {status_str}")

            step += 1
            next_tick += dt_target
            sleep_t = next_tick - time.perf_counter()
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\nInterrompido pelo usuario.")
    finally:
        s.close()
        xpc.close()
    print("-" * 65)
    print(f"[CONCLUIDO] Dry-run finalizado com sucesso ({step} ciclos executados).\n")


def main():
    parser = argparse.ArgumentParser(description="Teste de Conexao HIL (X-Plane <-> MCU)")
    parser.add_argument("--config", default="config_converter.json", help="Arquivo de configuracao JSON")
    parser.add_argument("--xplane", action="store_true", help="Testar apenas o X-Plane")
    parser.add_argument("--mcu", action="store_true", help="Testar apenas o MCU (Serial)")
    parser.add_argument("--loop", action="store_true", help="Executar apenas o Dry-Run do loop HIL")
    parser.add_argument("--all", action="store_true", help="Executar todos os testes sequencialmente")
    parser.add_argument("--duration", type=float, default=5.0, help="Duracao do dry-run em segundos (default: 5)")
    args = parser.parse_args()

    cfg = load_config(args.config)

    if args.xplane:
        test_xplane(cfg)
        return
    if args.mcu:
        test_mcu_serial(cfg)
        return
    if args.loop:
        test_loop_dry_run(cfg, args.duration)
        return
    if args.all:
        ok_xp = test_xplane(cfg)
        ok_mcu = test_mcu_serial(cfg)
        if ok_xp and ok_mcu:
            test_loop_dry_run(cfg, args.duration)
        return

    # Se nenhum argumento for passado, executa menu ou teste interativo
    print("\n" + "="*60)
    print("      DIAGNOSTICO E TESTE DA PONTE HIL (PYTHON BRIDGE)")
    print("="*60)
    print("Selecione o teste que deseja executar:")
    print("  [1] Testar conexao com X-Plane (UDP 49009)")
    print("  [2] Testar conexao com MCU (Serial COM)")
    print("  [3] Dry-Run do loop HIL em tempo real (5 segundos no terminal)")
    print("  [4] Executar TODOS os testes sequencialmente")
    print("  [0] Sair")
    
    try:
        choice = input("\nOpcao [1-4, default=4]: ").strip()
    except Exception:
        choice = "4"

    if choice == "1":
        test_xplane(cfg)
    elif choice == "2":
        test_mcu_serial(cfg)
    elif choice == "3":
        test_loop_dry_run(cfg, args.duration)
    elif choice in ("4", ""):
        ok_xp = test_xplane(cfg)
        ok_mcu = test_mcu_serial(cfg)
        if ok_xp and ok_mcu:
            test_loop_dry_run(cfg, args.duration)
    else:
        print("Saindo.")


if __name__ == "__main__":
    main()
