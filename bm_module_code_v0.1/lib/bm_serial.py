# /lib/bm_serial.py — CircuitPython 9.x, QT Py RP2040
# RAW RX read, TX with COBS; supports bristlemouth_sub, bristlemouth_tap, spotter_print

import board, busio, time

class BristlemouthSerial:
    BM_SERIAL_DEBUG = 0x00
    BM_SERIAL_ACK = 0x01
    BM_SERIAL_PUB = 0x02
    BM_SERIAL_SUB = 0x03
    BM_SERIAL_UNSUB = 0x04
    BM_SERIAL_LOG = 0x05
    BM_SERIAL_NET_MSG = 0x06
    BM_SERIAL_RTC_SET = 0x07
    BM_SERIAL_SELF_TEST = 0x08
    BM_SERIAL_NETWORK_INFO = 0x09
    BM_SERIAL_REBOOT_INFO = 0x0A
    BM_SERIAL_DFU_START = 0x30
    BM_SERIAL_DFU_CHUNK = 0x31
    BM_SERIAL_DFU_RESULT = 0x32
    BM_SERIAL_CFG_GET = 0x40
    BM_SERIAL_CFG_SET = 0x41
    BM_SERIAL_CFG_VALUE = 0x42
    BM_SERIAL_CFG_COMMIT = 0x43
    BM_SERIAL_CFG_STATUS_REQ = 0x44
    BM_SERIAL_CFG_STATUS_RESP = 0x45
    BM_SERIAL_CFG_DEL_REQ = 0x46
    BM_SERIAL_CFG_DEL_RESP = 0x47
    BM_SERIAL_CFG_CLEAR_REQ = 0x48
    BM_SERIAL_CFG_CLEAR_RESP = 0x49
    BM_SERIAL_DEVICE_INFO_REQ = 0x50
    BM_SERIAL_DEVICE_INFO_REPLY = 0x51
    BM_SERIAL_RESOURCE_REQ = 0x52
    BM_SERIAL_RESOURCE_REPLY = 0x53
    BM_SERIAL_NODE_ID_REQ = 0x60
    BM_SERIAL_NODE_ID_REPLY = 0x61
    BM_SERIAL_BAUD_RATE_REQ = 0x70
    BM_SERIAL_BAUD_RATE_REPLY = 0x71

    def __init__(self, uart=None, node_id: int = 0xC0FFEEEEF0CACC1A, baudrate: int = 115200, rx_bufsize: int = 512) -> None:
        self.node_id = node_id
        self._rx_buf = bytearray(rx_bufsize)
        self._taps = []  # callbacks that see ALL PUBs

        if uart is None:
            try:
                self.uart = busio.UART(board.TX, board.RX, baudrate=baudrate, timeout=0.01, receiver_buffer_size=rx_bufsize)
            except TypeError:
                self.uart = busio.UART(board.TX, board.RX, baudrate=baudrate, timeout=0.01)
        else:
            self.uart = uart

    # ---------- Public API ----------
    # def bristlemouth_sub(self, topic: str, fn):
    #     """Send SUB frame; we route via TAP so fn is unused (may be a no-op)."""
    #     topic_b = topic.encode("utf-8")
    #     packet = bytearray.fromhex("03000000") + len(topic_b).to_bytes(2, "little") + topic_b
    #     self._uart_write(self._finalize_packet(packet))
    def bristlemouth_sub(self, topic: str, fn):
        """Send SUB frame; note: this class does not route per-topic callbacks.
           Use bristlemouth_tap() in code.py to receive PUBs."""
        topic_b = topic.encode("utf-8")
        packet = bytearray.fromhex("03000000") + len(topic_b).to_bytes(2, "little") + topic_b
        self._uart_write(self.finalize_packet(packet))  # <— fix: no underscore

    def bristlemouth_tap(self, fn):
        """Register a callback that sees ALL PUBs."""
        if fn not in self._taps:
            self._taps.append(fn)

    # def spotter_print(self, data: str):
    #     topic = b"spotter/printf"
    #     packet = (
    #         self._get_pub_header()
    #         + len(topic).to_bytes(2, "little")
    #         + topic
    #         + (b"\x00" * 8)
    #         + (0).to_bytes(2, "little")               # filename length = 0
    #         + (len(data) + 1).to_bytes(2, "little")   # payload + '\n'
    #         + data.encode("utf-8")
    #         + b"\n"
    #     )
    #     return self._uart_write(self._finalize_packet(packet))
    def spotter_print(self, data: str):
        topic = b"spotter/printf"
        payload = data.encode("utf-8")
        packet = (
                self.get_pub_header()  # <— fix: no underscore
                + len(topic).to_bytes(2, "little")
                + topic
                + (b"\x00" * 8)
                + (0).to_bytes(2, "little")  # filename length = 0
                + (len(payload) + 1).to_bytes(2, "little")  # payload + '\n'
                + payload
                + b"\n"
        )
        return self._uart_write(self.finalize_packet(packet))  # <— fix: no underscore

    def spotter_tx(self, data: bytes) -> int | None:
        """
        Enqueue a network payload for sat/cell via topic 'spotter/transmit-data'.
        'data' must be bytes (encode your string with UTF-8 before calling).
        """
        topic = b"spotter/transmit-data"
        packet = (
            self.get_pub_header()
            + len(topic).to_bytes(2, "little")
            + topic
            + b"\x01"              # API flag used by BM stack
            + data
        )
        cobs = self.finalize_packet(packet)
        return self.uart.write(cobs)


    def spotter_log(self, filename: str, data: str) -> int | None:
        topic = b"spotter/fprintf"
        fname_b = filename.encode("utf-8")
        payload = data.encode("utf-8")
        packet = (
                self.get_pub_header()  # <— fix: use the right header
                + len(topic).to_bytes(2, "little")
                + topic
                + (b"\x00" * 8)
                + len(fname_b).to_bytes(2, "little")
                + (len(payload) + 1).to_bytes(2, "little")
                + fname_b
                + payload
                + b"\n"
        )
        cobs = self.finalize_packet(packet)  # <— fix: correct method
        return self.uart.write(cobs)

    def bristlemouth_process(self, timeout_s: float = 0.5) -> None:
        try:
            frames = self._read_burst_until_idle(timeout_s)
        except KeyboardInterrupt:
            return
        except Exception:
            return
        for frame in frames:
            if len(frame) < 4:
                continue
            msg_type = frame[0]
            payload = frame[4:]
            if msg_type == self.BM_SERIAL_PUB:
                self._process_publish_message(payload)

    # ---------- Internal ----------
    def _uart_write(self, b: bytes) -> int:
        return self.uart.write(b)

    def _read_burst_until_idle(self, idle_timeout: float = 0.5):
        deadline = time.monotonic() + idle_timeout
        data = bytearray()
        while True:
            try:
                b = self.uart.read(len(self._rx_buf))
            except KeyboardInterrupt:
                raise
            except Exception:
                break
            if b:
                data.extend(b)
                deadline = time.monotonic() + idle_timeout
            else:
                if time.monotonic() >= deadline:
                    break
                time.sleep(0.005)
        return [bytes(data)] if data else []

    def _process_publish_message(self, payload: bytes) -> None:
        if len(payload) < 12:
            return
        try:
            node_id   = int.from_bytes(payload[0:8], "little")
            msg_type  = payload[8]
            version   = payload[9]
            topic_len = int.from_bytes(payload[10:12], "little")
            end_topic = 12 + topic_len
            if end_topic > len(payload):
                return
            topic_b = payload[12:end_topic]
            try:
                topic = topic_b.decode("utf-8")
            except Exception:
                topic = str(topic_b)
            data = payload[end_topic:]
            data_len = len(data)

            for tap in self._taps:
                try:
                    tap(node_id, msg_type, version, topic_len, topic, data_len, data)
                except Exception:
                    pass
        except Exception:
            pass

    def finalize_packet(self, packet: bytearray) -> bytes:
        checksum = self._crc(0, packet)
        packet[2] = checksum & 0xFF
        packet[3] = (checksum >> 8) & 0xFF
        return self._cobs_encode(packet) + b"\x00"

    def get_pub_header(self) -> bytearray:
        return bytearray.fromhex("02000000") + self.node_id.to_bytes(8, "little") + bytearray.fromhex("0101")

    # --- COBS (TX only) ---
    def _cobs_encode(self, in_bytes: bytes) -> bytes:
        final_zero = True
        out_bytes = bytearray()
        idx = 0
        search_start_idx = 0
        for in_char in in_bytes:
            if in_char == 0:
                final_zero = True
                out_bytes.append(idx - search_start_idx + 1)
                out_bytes += in_bytes[search_start_idx:idx]
                search_start_idx = idx + 1
            else:
                if idx - search_start_idx == 0xFD:
                    final_zero = False
                    out_bytes.append(0xFF)
                    out_bytes += in_bytes[search_start_idx: idx + 1]
                    search_start_idx = idx + 1
            idx += 1
        if idx != search_start_idx or final_zero:
            out_bytes.append(idx - search_start_idx + 1)
            out_bytes += in_bytes[search_start_idx:idx]
        return bytes(out_bytes)

    # --- CRC ---
    def _crc(self, seed: int, src: bytes) -> int:
        e, f = 0, 0
        for i in src:
            e = (seed ^ i) & 0xFF
            f = e ^ ((e << 4) & 0xFF)
            seed = (seed >> 8) ^ (((f << 8) & 0xFFFF) ^ ((f << 3) & 0xFFFF)) ^ (f >> 4)
        return seed
