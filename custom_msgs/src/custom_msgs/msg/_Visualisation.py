"""autogenerated by genpy from custom_msgs/Visualisation.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class Visualisation(genpy.Message):
  _md5sum = "b425dd08e96d44f498f69a6461e54acd"
  _type = "custom_msgs/Visualisation"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """Header header1
Header header2
float32[] points1
float32[] points2
float32[] reprojected1
float32[] reprojected2

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

"""
  __slots__ = ['header1','header2','points1','points2','reprojected1','reprojected2']
  _slot_types = ['std_msgs/Header','std_msgs/Header','float32[]','float32[]','float32[]','float32[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header1,header2,points1,points2,reprojected1,reprojected2

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Visualisation, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header1 is None:
        self.header1 = std_msgs.msg.Header()
      if self.header2 is None:
        self.header2 = std_msgs.msg.Header()
      if self.points1 is None:
        self.points1 = []
      if self.points2 is None:
        self.points2 = []
      if self.reprojected1 is None:
        self.reprojected1 = []
      if self.reprojected2 is None:
        self.reprojected2 = []
    else:
      self.header1 = std_msgs.msg.Header()
      self.header2 = std_msgs.msg.Header()
      self.points1 = []
      self.points2 = []
      self.reprojected1 = []
      self.reprojected2 = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header1.seq, _x.header1.stamp.secs, _x.header1.stamp.nsecs))
      _x = self.header1.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.header2.seq, _x.header2.stamp.secs, _x.header2.stamp.nsecs))
      _x = self.header2.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.points1)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.points1))
      length = len(self.points2)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.points2))
      length = len(self.reprojected1)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.reprojected1))
      length = len(self.reprojected2)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.reprojected2))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header1 is None:
        self.header1 = std_msgs.msg.Header()
      if self.header2 is None:
        self.header2 = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header1.seq, _x.header1.stamp.secs, _x.header1.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header1.frame_id = str[start:end].decode('utf-8')
      else:
        self.header1.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.header2.seq, _x.header2.stamp.secs, _x.header2.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header2.frame_id = str[start:end].decode('utf-8')
      else:
        self.header2.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.points1 = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.points2 = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.reprojected1 = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.reprojected2 = struct.unpack(pattern, str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header1.seq, _x.header1.stamp.secs, _x.header1.stamp.nsecs))
      _x = self.header1.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.header2.seq, _x.header2.stamp.secs, _x.header2.stamp.nsecs))
      _x = self.header2.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.points1)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.points1.tostring())
      length = len(self.points2)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.points2.tostring())
      length = len(self.reprojected1)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.reprojected1.tostring())
      length = len(self.reprojected2)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.reprojected2.tostring())
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header1 is None:
        self.header1 = std_msgs.msg.Header()
      if self.header2 is None:
        self.header2 = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header1.seq, _x.header1.stamp.secs, _x.header1.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header1.frame_id = str[start:end].decode('utf-8')
      else:
        self.header1.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.header2.seq, _x.header2.stamp.secs, _x.header2.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header2.frame_id = str[start:end].decode('utf-8')
      else:
        self.header2.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.points1 = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.points2 = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.reprojected1 = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.reprojected2 = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
