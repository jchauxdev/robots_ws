"""Protocolo "Packet Serial" de los controladores RoboClaw (Basicmicro), Python 3.

Subconjunto minimo y verificado que usa el robot movil. Formatos tomados de:
  * Basicmicro, "RoboClaw Series User Manual" (Packet Serial / Advanced Packet Serial).
  * Libreria oficial basicmicro_python 2.0.10 (MIT) y libreria heredada roboclaw_python (para V5 y anteriores).

Trama de comando:  [direccion][comando][datos...][CRC16 alto][CRC16 bajo]
  - CRC16-CCITT (polinomio 0x1021, valor inicial 0) calculado sobre direccion+comando+datos.
  - Los comandos de escritura responden con un byte 0xFF (ACK) si el CRC es correcto.
  - Los comandos de lectura responden [datos...][CRC16] calculado sobre direccion+comando+datos.
  - Enteros en big-endian. Direcciones validas 0x80..0x87 (128..135).

Diferencias corregidas respecto al paquete ROS 1 'roboclaw_ros' usado en la version anterior del curso:
  * Python 3 (bytes) en lugar de chr()/ord() de Python 2.
  * "Read Status" (cmd 90) devuelve 4 bytes en firmware actual (la version anterior leia 2 y fallaba el CRC).
  * SpeedAccelM1M2 usa el comando 40 (la version anterior enviaba el 38, que es de un solo motor).
"""
from __future__ import annotations

import struct
import threading
import time

try:
    import serial  # pyserial
except ImportError:  # permite importar el modulo (p. ej. para pruebas de CRC) sin pyserial
    serial = None


class Cmd:
    M1FORWARD = 0
    M2FORWARD = 4
    GETM1ENC = 16
    GETM2ENC = 17
    GETM1SPEED = 18
    GETM2SPEED = 19
    RESETENC = 20
    GETVERSION = 21
    GETMBATT = 24
    GETLBATT = 25
    MIXEDDUTY = 34
    MIXEDSPEED = 37
    MIXEDSPEEDACCEL = 40
    GETCURRENTS = 49
    READM1PID = 55
    READM2PID = 56
    GETTEMP = 82
    GETERROR = 90


# "Read Status" (cmd 90). El significado de los bits depende de la generacion del controlador:
#  * "v5": RoboClaw User Manual rev. 5.6 (firmware 4.1.20+, hardware V3-V6; unidades del laboratorio, "RC15AV5").
#  * "actual": libreria basicmicro_python 2.0.10 (16 bits bajos = errores, 16 altos = avisos).
# Ambas librerias oficiales leen 4 bytes. Se publica tambien el valor crudo para contrastarlo con Motion Studio.
STATUS_V5 = {  # bit: (texto, nivel) con nivel 2 = error, 1 = aviso, 0 = informativo
    0x0001: ("Sobrecorriente M1 (aviso)", 1), 0x0002: ("Sobrecorriente M2 (aviso)", 1), 0x0004: ("E-Stop", 2),
    0x0008: ("Temperatura (error)", 2), 0x0010: ("Temperatura 2 (error)", 2),
    0x0020: ("Bateria principal alta (error)", 2),
    0x0040: ("Bateria logica alta (error)", 2), 0x0080: ("Bateria logica baja (error)", 2),
    0x0100: ("Falla del driver M1", 2), 0x0200: ("Falla del driver M2", 2),
    0x0400: ("Bateria principal alta (aviso)", 1), 0x0800: ("Bateria principal baja (aviso)", 1),
    0x1000: ("Temperatura (aviso)", 1), 0x2000: ("Temperatura 2 (aviso)", 1), 0x4000: ("M1 en home", 0),
    0x8000: ("M2 en home", 0),
}
ERROR_BITS = {
    0x0001: "Paro de emergencia (E-Stop)", 0x0002: "Temperatura 1", 0x0004: "Temperatura 2",
    0x0008: "Bateria principal alta", 0x0010: "Bateria logica alta", 0x0020: "Bateria logica baja",
    0x0100: "Error de velocidad M1", 0x0200: "Error de velocidad M2", 0x0400: "Error de posicion M1",
    0x0800: "Error de posicion M2", 0x1000: "Corriente M1", 0x2000: "Corriente M2",
    0x4000: "Bateria principal baja", 0x8000: "Bateria principal alta (histeresis)",
}
WARNING_BITS = {
    0x0001: "Sobrecorriente M1", 0x0002: "Sobrecorriente M2", 0x0004: "Bateria principal alta",
    0x0008: "Bateria principal baja", 0x0010: "Temperatura 1", 0x0020: "Temperatura 2",
    0x0040: "Senal S4", 0x0080: "Senal S5", 0x0100: "Velocidad M1", 0x0200: "Velocidad M2",
    0x0400: "Posicion M1", 0x0800: "Posicion M2", 0x1000: "Bus CAN", 0x2000: "Arranque",
    0x4000: "Sobre-regeneracion M1", 0x8000: "Sobre-regeneracion M2",
}


def decode_status(status: int, generation: str = "v5") -> tuple[list[str], list[str]]:
    """Devuelve (errores, avisos) a partir de la palabra del comando 90."""
    if generation == "v5":
        errs = [t for b, (t, lvl) in STATUS_V5.items() if status & b and lvl == 2]
        warns = [t for b, (t, lvl) in STATUS_V5.items() if status & b and lvl == 1]
        return errs, warns
    err, warn = status & 0xFFFF, (status >> 16) & 0xFFFF
    return ([t for b, t in ERROR_BITS.items() if err & b],
            [t for b, t in WARNING_BITS.items() if warn & b])


def crc16(data: bytes, crc: int = 0) -> int:
    """CRC16-CCITT (XModem): poly 0x1021, init 0, sin reflexion."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def to_s32(v: int) -> int:
    return v - 0x100000000 if v & 0x80000000 else v


class RoboClawError(IOError):
    pass


class RoboClaw:
    """Cliente sincronico y seguro entre hilos para un RoboClaw en modo Packet Serial.

    Por USB el RoboClaw aparece como /dev/ttyACM*; el baudrate se ignora en USB pero se
    usa si se conecta por los pines S1/S2 (UART TTL).
    """

    def __init__(self, port: str, baud: int = 38400, address: int = 0x80,
                 timeout: float = 0.05, retries: int = 3):
        if not 0x80 <= address <= 0x87:
            raise ValueError("Direccion fuera de rango (128..135)")
        self.port_name, self.baud, self.address = port, baud, address
        self.timeout, self.retries = timeout, retries
        self._lock = threading.Lock()
        self._ser = None
        self.crc_errors = 0

    # ------------------------------------------------------------------ conexion
    def open(self):
        if serial is None:
            raise RoboClawError("pyserial no esta instalado (sudo apt install python3-serial)")
        self._ser = serial.Serial(self.port_name, self.baud, timeout=self.timeout,
                                  write_timeout=self.timeout * 4)
        time.sleep(0.05)
        self._ser.reset_input_buffer()
        return self

    def close(self):
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    # ------------------------------------------------------------------ nivel bajo
    def _transact_write(self, cmd: int, payload: bytes = b"") -> bool:
        pkt = bytes([self.address, cmd]) + payload
        pkt += struct.pack(">H", crc16(pkt))
        with self._lock:
            for _ in range(self.retries):
                self._ser.reset_input_buffer()
                self._ser.write(pkt)
                ack = self._ser.read(1)
                if ack == b"\xff":
                    return True
            self.crc_errors += 1
            return False

    def _transact_read(self, cmd: int, nbytes: int) -> bytes | None:
        head = bytes([self.address, cmd])
        with self._lock:
            for _ in range(self.retries):
                self._ser.reset_input_buffer()
                self._ser.write(head)
                data = self._ser.read(nbytes + 2)
                if len(data) == nbytes + 2:
                    body, rx_crc = data[:nbytes], struct.unpack(">H", data[nbytes:])[0]
                    if crc16(head + body) == rx_crc:
                        return body
                self.crc_errors += 1
            return None

    # ------------------------------------------------------------------ lectura
    def read_version(self) -> str | None:
        head = bytes([self.address, Cmd.GETVERSION])
        with self._lock:
            for _ in range(self.retries):
                self._ser.reset_input_buffer()
                self._ser.write(head)
                buf = bytearray()
                while len(buf) < 48:
                    c = self._ser.read(1)
                    if not c:
                        break
                    buf += c
                    if c == b"\x00":
                        break
                crc = self._ser.read(2)
                if buf.endswith(b"\x00") and len(crc) == 2 and \
                        crc16(head + bytes(buf)) == struct.unpack(">H", crc)[0]:
                    return buf[:-1].decode(errors="replace").strip()
            return None

    def read_encoder(self, motor: int) -> tuple[int, int] | None:
        """Cuenta de cuadratura (con signo, 32 bits) y byte de estado del motor 1 o 2."""
        body = self._transact_read(Cmd.GETM1ENC if motor == 1 else Cmd.GETM2ENC, 5)
        if body is None:
            return None
        return to_s32(struct.unpack(">I", body[:4])[0]), body[4]

    def read_speed(self, motor: int) -> int | None:
        """Velocidad medida en pulsos/s (QPPS) con signo."""
        body = self._transact_read(Cmd.GETM1SPEED if motor == 1 else Cmd.GETM2SPEED, 5)
        if body is None:
            return None
        v = to_s32(struct.unpack(">I", body[:4])[0])
        if body[4] & 0x01 and v > 0:  # bit 0 del estado = sentido inverso
            v = -v
        return v

    def read_main_battery(self) -> float | None:
        body = self._transact_read(Cmd.GETMBATT, 2)
        return None if body is None else struct.unpack(">H", body)[0] / 10.0

    def read_logic_battery(self) -> float | None:
        body = self._transact_read(Cmd.GETLBATT, 2)
        return None if body is None else struct.unpack(">H", body)[0] / 10.0

    def read_temperature(self) -> float | None:
        body = self._transact_read(Cmd.GETTEMP, 2)
        return None if body is None else struct.unpack(">H", body)[0] / 10.0

    def read_currents(self) -> tuple[float, float] | None:
        body = self._transact_read(Cmd.GETCURRENTS, 4)
        if body is None:
            return None
        c1, c2 = struct.unpack(">hh", body)
        return c1 / 100.0, c2 / 100.0  # unidades de 10 mA -> A

    def read_status(self) -> int | None:
        body = self._transact_read(Cmd.GETERROR, 4)
        return None if body is None else struct.unpack(">I", body)[0]

    def read_velocity_pid(self, motor: int) -> dict | None:
        """P, I, D (punto fijo 16.16) y QPPS configurados en Motion Studio."""
        body = self._transact_read(Cmd.READM1PID if motor == 1 else Cmd.READM2PID, 16)
        if body is None:
            return None
        p, i, d, qpps = struct.unpack(">IIII", body)
        return {"p": p / 65536.0, "i": i / 65536.0, "d": d / 65536.0, "qpps": qpps}

    # ------------------------------------------------------------------ escritura
    def reset_encoders(self) -> bool:
        return self._transact_write(Cmd.RESETENC)

    def speed_m1m2(self, qpps1: int, qpps2: int) -> bool:
        return self._transact_write(Cmd.MIXEDSPEED, struct.pack(">ii", int(qpps1), int(qpps2)))

    def speed_accel_m1m2(self, accel: int, qpps1: int, qpps2: int) -> bool:
        """Velocidad con rampa de aceleracion 'accel' [pulsos/s^2] usando el PID de velocidad."""
        return self._transact_write(Cmd.MIXEDSPEEDACCEL,
                                    struct.pack(">Iii", int(abs(accel)), int(qpps1), int(qpps2)))

    def duty_m1m2(self, duty1: int, duty2: int) -> bool:
        """Ciclo util con signo, -32767..32767 (0 = motores libres, sin zumbido del PID)."""
        d1 = max(-32767, min(32767, int(duty1)))
        d2 = max(-32767, min(32767, int(duty2)))
        return self._transact_write(Cmd.MIXEDDUTY, struct.pack(">hh", d1, d2))

    def stop(self) -> bool:
        return self.duty_m1m2(0, 0)
