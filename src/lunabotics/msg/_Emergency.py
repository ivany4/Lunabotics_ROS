"""autogenerated by genpy from lunabotics/Emergency.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class Emergency(genpy.Message):
  _md5sum = "0ea102f932a599527a510d77dc158fb2"
  _type = "lunabotics/Emergency"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# This contains information about emergency type

uint8 reason					# 0 - unknown, 1 - proximity, 2 - crater

# Emergency caused by proximity alert
float32 proximity_distance		# distance to closest point
float32 proximity_angle			# angle of closest point

# Emergency caused by detection of crater
uint8 is_crater_alert					# 1 if yes 0 if no
geometry_msgs/Point crater_position		# Position of a crater on the map

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

"""
  __slots__ = ['reason','proximity_distance','proximity_angle','is_crater_alert','crater_position']
  _slot_types = ['uint8','float32','float32','uint8','geometry_msgs/Point']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       reason,proximity_distance,proximity_angle,is_crater_alert,crater_position

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Emergency, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.reason is None:
        self.reason = 0
      if self.proximity_distance is None:
        self.proximity_distance = 0.
      if self.proximity_angle is None:
        self.proximity_angle = 0.
      if self.is_crater_alert is None:
        self.is_crater_alert = 0
      if self.crater_position is None:
        self.crater_position = geometry_msgs.msg.Point()
    else:
      self.reason = 0
      self.proximity_distance = 0.
      self.proximity_angle = 0.
      self.is_crater_alert = 0
      self.crater_position = geometry_msgs.msg.Point()

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
      buff.write(_struct_B2fB3d.pack(_x.reason, _x.proximity_distance, _x.proximity_angle, _x.is_crater_alert, _x.crater_position.x, _x.crater_position.y, _x.crater_position.z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.crater_position is None:
        self.crater_position = geometry_msgs.msg.Point()
      end = 0
      _x = self
      start = end
      end += 34
      (_x.reason, _x.proximity_distance, _x.proximity_angle, _x.is_crater_alert, _x.crater_position.x, _x.crater_position.y, _x.crater_position.z,) = _struct_B2fB3d.unpack(str[start:end])
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
      buff.write(_struct_B2fB3d.pack(_x.reason, _x.proximity_distance, _x.proximity_angle, _x.is_crater_alert, _x.crater_position.x, _x.crater_position.y, _x.crater_position.z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.crater_position is None:
        self.crater_position = geometry_msgs.msg.Point()
      end = 0
      _x = self
      start = end
      end += 34
      (_x.reason, _x.proximity_distance, _x.proximity_angle, _x.is_crater_alert, _x.crater_position.x, _x.crater_position.y, _x.crater_position.z,) = _struct_B2fB3d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_B2fB3d = struct.Struct("<B2fB3d")
