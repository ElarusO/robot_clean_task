; Auto-generated. Do not edit!


(cl:in-package restaurant_clean_task-srv)


;//! \htmlinclude GripperControl-request.msg.html

(cl:defclass <GripperControl-request> (roslisp-msg-protocol:ros-message)
  ((angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass GripperControl-request (<GripperControl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperControl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperControl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name restaurant_clean_task-srv:<GripperControl-request> is deprecated: use restaurant_clean_task-srv:GripperControl-request instead.")))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <GripperControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader restaurant_clean_task-srv:angle-val is deprecated.  Use restaurant_clean_task-srv:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperControl-request>) ostream)
  "Serializes a message object of type '<GripperControl-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperControl-request>) istream)
  "Deserializes a message object of type '<GripperControl-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperControl-request>)))
  "Returns string type for a service object of type '<GripperControl-request>"
  "restaurant_clean_task/GripperControlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperControl-request)))
  "Returns string type for a service object of type 'GripperControl-request"
  "restaurant_clean_task/GripperControlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperControl-request>)))
  "Returns md5sum for a message object of type '<GripperControl-request>"
  "5813b689e45b3f1ea25d03fd1a946596")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperControl-request)))
  "Returns md5sum for a message object of type 'GripperControl-request"
  "5813b689e45b3f1ea25d03fd1a946596")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperControl-request>)))
  "Returns full string definition for message of type '<GripperControl-request>"
  (cl:format cl:nil "float64 angle  # 爪部目标角度~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperControl-request)))
  "Returns full string definition for message of type 'GripperControl-request"
  (cl:format cl:nil "float64 angle  # 爪部目标角度~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperControl-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperControl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperControl-request
    (cl:cons ':angle (angle msg))
))
;//! \htmlinclude GripperControl-response.msg.html

(cl:defclass <GripperControl-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (current_angle
    :reader current_angle
    :initarg :current_angle
    :type cl:float
    :initform 0.0)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass GripperControl-response (<GripperControl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperControl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperControl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name restaurant_clean_task-srv:<GripperControl-response> is deprecated: use restaurant_clean_task-srv:GripperControl-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GripperControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader restaurant_clean_task-srv:success-val is deprecated.  Use restaurant_clean_task-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'current_angle-val :lambda-list '(m))
(cl:defmethod current_angle-val ((m <GripperControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader restaurant_clean_task-srv:current_angle-val is deprecated.  Use restaurant_clean_task-srv:current_angle instead.")
  (current_angle m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <GripperControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader restaurant_clean_task-srv:message-val is deprecated.  Use restaurant_clean_task-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperControl-response>) ostream)
  "Serializes a message object of type '<GripperControl-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'current_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperControl-response>) istream)
  "Deserializes a message object of type '<GripperControl-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_angle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperControl-response>)))
  "Returns string type for a service object of type '<GripperControl-response>"
  "restaurant_clean_task/GripperControlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperControl-response)))
  "Returns string type for a service object of type 'GripperControl-response"
  "restaurant_clean_task/GripperControlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperControl-response>)))
  "Returns md5sum for a message object of type '<GripperControl-response>"
  "5813b689e45b3f1ea25d03fd1a946596")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperControl-response)))
  "Returns md5sum for a message object of type 'GripperControl-response"
  "5813b689e45b3f1ea25d03fd1a946596")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperControl-response>)))
  "Returns full string definition for message of type '<GripperControl-response>"
  (cl:format cl:nil "bool success   # 执行结果~%float64 current_angle~%string message # 状态信息~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperControl-response)))
  "Returns full string definition for message of type 'GripperControl-response"
  (cl:format cl:nil "bool success   # 执行结果~%float64 current_angle~%string message # 状态信息~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperControl-response>))
  (cl:+ 0
     1
     8
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperControl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperControl-response
    (cl:cons ':success (success msg))
    (cl:cons ':current_angle (current_angle msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GripperControl)))
  'GripperControl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GripperControl)))
  'GripperControl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperControl)))
  "Returns string type for a service object of type '<GripperControl>"
  "restaurant_clean_task/GripperControl")