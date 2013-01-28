"""autogenerated by genpy from custom_msgs/StampedMatchesWithImages.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import custom_msgs.msg
import sensor_msgs.msg
import std_msgs.msg

class StampedMatchesWithImages(genpy.Message):
  _md5sum = "ee218eba4fa8a4f54760afca804eb3b2"
  _type = "custom_msgs/StampedMatchesWithImages"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header
custom_msgs/StampedMatchesWithImage frame1
custom_msgs/StampedMatchesWithImage frame2


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

================================================================================
MSG: custom_msgs/StampedMatchesWithImage
Header header
float32[] pts
sensor_msgs/Image image

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of cameara
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in src/image_encodings.cpp

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

"""
  __slots__ = ['header','frame1','frame2']
  _slot_types = ['std_msgs/Header','custom_msgs/StampedMatchesWithImage','custom_msgs/StampedMatchesWithImage']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,frame1,frame2

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(StampedMatchesWithImages, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.frame1 is None:
        self.frame1 = custom_msgs.msg.StampedMatchesWithImage()
      if self.frame2 is None:
        self.frame2 = custom_msgs.msg.StampedMatchesWithImage()
    else:
      self.header = std_msgs.msg.Header()
      self.frame1 = custom_msgs.msg.StampedMatchesWithImage()
      self.frame2 = custom_msgs.msg.StampedMatchesWithImage()

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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.frame1.header.seq, _x.frame1.header.stamp.secs, _x.frame1.header.stamp.nsecs))
      _x = self.frame1.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.frame1.pts)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.frame1.pts))
      _x = self
      buff.write(_struct_3I.pack(_x.frame1.image.header.seq, _x.frame1.image.header.stamp.secs, _x.frame1.image.header.stamp.nsecs))
      _x = self.frame1.image.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.frame1.image.height, _x.frame1.image.width))
      _x = self.frame1.image.encoding
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_BI.pack(_x.frame1.image.is_bigendian, _x.frame1.image.step))
      _x = self.frame1.image.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.frame2.header.seq, _x.frame2.header.stamp.secs, _x.frame2.header.stamp.nsecs))
      _x = self.frame2.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.frame2.pts)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.frame2.pts))
      _x = self
      buff.write(_struct_3I.pack(_x.frame2.image.header.seq, _x.frame2.image.header.stamp.secs, _x.frame2.image.header.stamp.nsecs))
      _x = self.frame2.image.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.frame2.image.height, _x.frame2.image.width))
      _x = self.frame2.image.encoding
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_BI.pack(_x.frame2.image.is_bigendian, _x.frame2.image.step))
      _x = self.frame2.image.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.frame1 is None:
        self.frame1 = custom_msgs.msg.StampedMatchesWithImage()
      if self.frame2 is None:
        self.frame2 = custom_msgs.msg.StampedMatchesWithImage()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.frame1.header.seq, _x.frame1.header.stamp.secs, _x.frame1.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.frame1.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.frame1.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.frame1.pts = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 12
      (_x.frame1.image.header.seq, _x.frame1.image.header.stamp.secs, _x.frame1.image.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.frame1.image.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.frame1.image.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.frame1.image.height, _x.frame1.image.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.frame1.image.encoding = str[start:end].decode('utf-8')
      else:
        self.frame1.image.encoding = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.frame1.image.is_bigendian, _x.frame1.image.step,) = _struct_BI.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.frame1.image.data = str[start:end].decode('utf-8')
      else:
        self.frame1.image.data = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.frame2.header.seq, _x.frame2.header.stamp.secs, _x.frame2.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.frame2.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.frame2.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.frame2.pts = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 12
      (_x.frame2.image.header.seq, _x.frame2.image.header.stamp.secs, _x.frame2.image.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.frame2.image.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.frame2.image.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.frame2.image.height, _x.frame2.image.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.frame2.image.encoding = str[start:end].decode('utf-8')
      else:
        self.frame2.image.encoding = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.frame2.image.is_bigendian, _x.frame2.image.step,) = _struct_BI.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.frame2.image.data = str[start:end].decode('utf-8')
      else:
        self.frame2.image.data = str[start:end]
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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.frame1.header.seq, _x.frame1.header.stamp.secs, _x.frame1.header.stamp.nsecs))
      _x = self.frame1.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.frame1.pts)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.frame1.pts.tostring())
      _x = self
      buff.write(_struct_3I.pack(_x.frame1.image.header.seq, _x.frame1.image.header.stamp.secs, _x.frame1.image.header.stamp.nsecs))
      _x = self.frame1.image.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.frame1.image.height, _x.frame1.image.width))
      _x = self.frame1.image.encoding
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_BI.pack(_x.frame1.image.is_bigendian, _x.frame1.image.step))
      _x = self.frame1.image.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.frame2.header.seq, _x.frame2.header.stamp.secs, _x.frame2.header.stamp.nsecs))
      _x = self.frame2.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.frame2.pts)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.frame2.pts.tostring())
      _x = self
      buff.write(_struct_3I.pack(_x.frame2.image.header.seq, _x.frame2.image.header.stamp.secs, _x.frame2.image.header.stamp.nsecs))
      _x = self.frame2.image.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.frame2.image.height, _x.frame2.image.width))
      _x = self.frame2.image.encoding
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_BI.pack(_x.frame2.image.is_bigendian, _x.frame2.image.step))
      _x = self.frame2.image.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.frame1 is None:
        self.frame1 = custom_msgs.msg.StampedMatchesWithImage()
      if self.frame2 is None:
        self.frame2 = custom_msgs.msg.StampedMatchesWithImage()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.frame1.header.seq, _x.frame1.header.stamp.secs, _x.frame1.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.frame1.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.frame1.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.frame1.pts = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      _x = self
      start = end
      end += 12
      (_x.frame1.image.header.seq, _x.frame1.image.header.stamp.secs, _x.frame1.image.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.frame1.image.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.frame1.image.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.frame1.image.height, _x.frame1.image.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.frame1.image.encoding = str[start:end].decode('utf-8')
      else:
        self.frame1.image.encoding = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.frame1.image.is_bigendian, _x.frame1.image.step,) = _struct_BI.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.frame1.image.data = str[start:end].decode('utf-8')
      else:
        self.frame1.image.data = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.frame2.header.seq, _x.frame2.header.stamp.secs, _x.frame2.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.frame2.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.frame2.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.frame2.pts = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      _x = self
      start = end
      end += 12
      (_x.frame2.image.header.seq, _x.frame2.image.header.stamp.secs, _x.frame2.image.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.frame2.image.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.frame2.image.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.frame2.image.height, _x.frame2.image.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.frame2.image.encoding = str[start:end].decode('utf-8')
      else:
        self.frame2.image.encoding = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.frame2.image.is_bigendian, _x.frame2.image.step,) = _struct_BI.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.frame2.image.data = str[start:end].decode('utf-8')
      else:
        self.frame2.image.data = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_2I = struct.Struct("<2I")
_struct_BI = struct.Struct("<BI")
