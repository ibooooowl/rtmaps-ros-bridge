; Auto-generated. Do not edit!


(cl:in-package my_data_types-msg)


;//! \htmlinclude my_data_type.msg.html

(cl:defclass <my_data_type> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (my_data
    :reader my_data
    :initarg :my_data
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass my_data_type (<my_data_type>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <my_data_type>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'my_data_type)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_data_types-msg:<my_data_type> is deprecated: use my_data_types-msg:my_data_type instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <my_data_type>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_data_types-msg:header-val is deprecated.  Use my_data_types-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <my_data_type>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_data_types-msg:id-val is deprecated.  Use my_data_types-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'my_data-val :lambda-list '(m))
(cl:defmethod my_data-val ((m <my_data_type>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_data_types-msg:my_data-val is deprecated.  Use my_data_types-msg:my_data instead.")
  (my_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <my_data_type>) ostream)
  "Serializes a message object of type '<my_data_type>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'my_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'my_data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <my_data_type>) istream)
  "Deserializes a message object of type '<my_data_type>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'my_data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'my_data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<my_data_type>)))
  "Returns string type for a message object of type '<my_data_type>"
  "my_data_types/my_data_type")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'my_data_type)))
  "Returns string type for a message object of type 'my_data_type"
  "my_data_types/my_data_type")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<my_data_type>)))
  "Returns md5sum for a message object of type '<my_data_type>"
  "fb71f34f8ba59a227d3fb9a24273a811")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'my_data_type)))
  "Returns md5sum for a message object of type 'my_data_type"
  "fb71f34f8ba59a227d3fb9a24273a811")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<my_data_type>)))
  "Returns full string definition for message of type '<my_data_type>"
  (cl:format cl:nil "Header header~%int32  id~%float32[] my_data~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'my_data_type)))
  "Returns full string definition for message of type 'my_data_type"
  (cl:format cl:nil "Header header~%int32  id~%float32[] my_data~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <my_data_type>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'my_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <my_data_type>))
  "Converts a ROS message object to a list"
  (cl:list 'my_data_type
    (cl:cons ':header (header msg))
    (cl:cons ':id (id msg))
    (cl:cons ':my_data (my_data msg))
))
