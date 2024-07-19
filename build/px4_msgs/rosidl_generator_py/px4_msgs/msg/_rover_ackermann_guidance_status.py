# generated from rosidl_generator_py/resource/_idl.py.em
# with input from px4_msgs:msg/RoverAckermannGuidanceStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RoverAckermannGuidanceStatus(type):
    """Metaclass of message 'RoverAckermannGuidanceStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('px4_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'px4_msgs.msg.RoverAckermannGuidanceStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__rover_ackermann_guidance_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__rover_ackermann_guidance_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__rover_ackermann_guidance_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__rover_ackermann_guidance_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__rover_ackermann_guidance_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RoverAckermannGuidanceStatus(metaclass=Metaclass_RoverAckermannGuidanceStatus):
    """Message class 'RoverAckermannGuidanceStatus'."""

    __slots__ = [
        '_timestamp',
        '_actual_speed',
        '_desired_speed',
        '_lookahead_distance',
        '_heading_error',
        '_pid_throttle_integral',
        '_crosstrack_error',
    ]

    _fields_and_field_types = {
        'timestamp': 'uint64',
        'actual_speed': 'float',
        'desired_speed': 'float',
        'lookahead_distance': 'float',
        'heading_error': 'float',
        'pid_throttle_integral': 'float',
        'crosstrack_error': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.timestamp = kwargs.get('timestamp', int())
        self.actual_speed = kwargs.get('actual_speed', float())
        self.desired_speed = kwargs.get('desired_speed', float())
        self.lookahead_distance = kwargs.get('lookahead_distance', float())
        self.heading_error = kwargs.get('heading_error', float())
        self.pid_throttle_integral = kwargs.get('pid_throttle_integral', float())
        self.crosstrack_error = kwargs.get('crosstrack_error', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.timestamp != other.timestamp:
            return False
        if self.actual_speed != other.actual_speed:
            return False
        if self.desired_speed != other.desired_speed:
            return False
        if self.lookahead_distance != other.lookahead_distance:
            return False
        if self.heading_error != other.heading_error:
            return False
        if self.pid_throttle_integral != other.pid_throttle_integral:
            return False
        if self.crosstrack_error != other.crosstrack_error:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def timestamp(self):
        """Message field 'timestamp'."""
        return self._timestamp

    @timestamp.setter
    def timestamp(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'timestamp' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'timestamp' field must be an unsigned integer in [0, 18446744073709551615]"
        self._timestamp = value

    @property
    def actual_speed(self):
        """Message field 'actual_speed'."""
        return self._actual_speed

    @actual_speed.setter
    def actual_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'actual_speed' field must be of type 'float'"
        self._actual_speed = value

    @property
    def desired_speed(self):
        """Message field 'desired_speed'."""
        return self._desired_speed

    @desired_speed.setter
    def desired_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'desired_speed' field must be of type 'float'"
        self._desired_speed = value

    @property
    def lookahead_distance(self):
        """Message field 'lookahead_distance'."""
        return self._lookahead_distance

    @lookahead_distance.setter
    def lookahead_distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lookahead_distance' field must be of type 'float'"
        self._lookahead_distance = value

    @property
    def heading_error(self):
        """Message field 'heading_error'."""
        return self._heading_error

    @heading_error.setter
    def heading_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'heading_error' field must be of type 'float'"
        self._heading_error = value

    @property
    def pid_throttle_integral(self):
        """Message field 'pid_throttle_integral'."""
        return self._pid_throttle_integral

    @pid_throttle_integral.setter
    def pid_throttle_integral(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pid_throttle_integral' field must be of type 'float'"
        self._pid_throttle_integral = value

    @property
    def crosstrack_error(self):
        """Message field 'crosstrack_error'."""
        return self._crosstrack_error

    @crosstrack_error.setter
    def crosstrack_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'crosstrack_error' field must be of type 'float'"
        self._crosstrack_error = value
