"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

import cStringIO as StringIO
import struct

class lcmt_motor_state(object):
    __slots__ = ["timestamp", "current", "position", "position2", "fault"]

    def __init__(self):
        self.timestamp = 0
        self.current = 0.0
        self.position = 0.0
        self.position2 = 0.0
        self.fault = 0

    def encode(self):
        buf = StringIO.StringIO()
        buf.write(lcmt_motor_state._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qdddb", self.timestamp, self.current, self.position, self.position2, self.fault))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = StringIO.StringIO(data)
        if buf.read(8) != lcmt_motor_state._get_packed_fingerprint():
            raise ValueError("Decode error")
        return lcmt_motor_state._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = lcmt_motor_state()
        self.timestamp, self.current, self.position, self.position2, self.fault = struct.unpack(">qdddb", buf.read(33))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if lcmt_motor_state in parents: return 0
        tmphash = (0x897e54ba4eb6e80f) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if lcmt_motor_state._packed_fingerprint is None:
            lcmt_motor_state._packed_fingerprint = struct.pack(">Q", lcmt_motor_state._get_hash_recursive([]))
        return lcmt_motor_state._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

