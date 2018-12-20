"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class robot_state_t(object):
    __slots__ = ["timestamp", "num_joints", "joint_position", "joint_velocity"]

    __typenames__ = ["int64_t", "int32_t", "double", "double"]

    __dimensions__ = [None, None, ["num_joints"], ["num_joints"]]

    def __init__(self):
        self.timestamp = 0
        self.num_joints = 0
        self.joint_position = []
        self.joint_velocity = []

    def encode(self):
        buf = BytesIO()
        buf.write(robot_state_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qi", self.timestamp, self.num_joints))
        buf.write(struct.pack('>%dd' % self.num_joints, *self.joint_position[:self.num_joints]))
        buf.write(struct.pack('>%dd' % self.num_joints, *self.joint_velocity[:self.num_joints]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != robot_state_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return robot_state_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = robot_state_t()
        self.timestamp, self.num_joints = struct.unpack(">qi", buf.read(12))
        self.joint_position = struct.unpack('>%dd' % self.num_joints, buf.read(self.num_joints * 8))
        self.joint_velocity = struct.unpack('>%dd' % self.num_joints, buf.read(self.num_joints * 8))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if robot_state_t in parents: return 0
        tmphash = (0xf97feaac43d4aed1) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if robot_state_t._packed_fingerprint is None:
            robot_state_t._packed_fingerprint = struct.pack(">Q", robot_state_t._get_hash_recursive([]))
        return robot_state_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
