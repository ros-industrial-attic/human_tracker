; Auto-generated. Do not edit!


(cl:in-package Acuity-msg)


;//! \htmlinclude LaserRange.msg.html

(cl:defclass <LaserRange> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (range
    :reader range
    :initarg :range
    :type cl:float
    :initform 0.0))
)

(cl:defclass LaserRange (<LaserRange>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LaserRange>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LaserRange)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Acuity-msg:<LaserRange> is deprecated: use Acuity-msg:LaserRange instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LaserRange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Acuity-msg:header-val is deprecated.  Use Acuity-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'range-val :lambda-list '(m))
(cl:defmethod range-val ((m <LaserRange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Acuity-msg:range-val is deprecated.  Use Acuity-msg:range instead.")
  (range m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LaserRange>) ostream)
  "Serializes a message object of type '<LaserRange>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LaserRange>) istream)
  "Deserializes a message object of type '<LaserRange>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'range) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LaserRange>)))
  "Returns string type for a message object of type '<LaserRange>"
  "Acuity/LaserRange")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LaserRange)))
  "Returns string type for a message object of type 'LaserRange"
  "Acuity/LaserRange")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LaserRange>)))
  "Returns md5sum for a message object of type '<LaserRange>"
  "cded8b3954e4ed742b2dd2e4170304d4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LaserRange)))
  "Returns md5sum for a message object of type 'LaserRange"
  "cded8b3954e4ed742b2dd2e4170304d4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LaserRange>)))
  "Returns full string definition for message of type '<LaserRange>"
  (cl:format cl:nil "# Single laser range reading from an active ranger.  NOTE: the sensor_msgs/Range.msg~%# was not used because that message is for sensors with a field of view, as opposed~%# to the laser range sensor which provides a point range.~%~%Header header    	# timestamp in the header is the time the ranger~%		 	# returned the distance reading~%~%float32 range       # range value (position) [m]~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LaserRange)))
  "Returns full string definition for message of type 'LaserRange"
  (cl:format cl:nil "# Single laser range reading from an active ranger.  NOTE: the sensor_msgs/Range.msg~%# was not used because that message is for sensors with a field of view, as opposed~%# to the laser range sensor which provides a point range.~%~%Header header    	# timestamp in the header is the time the ranger~%		 	# returned the distance reading~%~%float32 range       # range value (position) [m]~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LaserRange>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LaserRange>))
  "Converts a ROS message object to a list"
  (cl:list 'LaserRange
    (cl:cons ':header (header msg))
    (cl:cons ':range (range msg))
))
