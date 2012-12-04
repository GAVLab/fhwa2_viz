; Auto-generated. Do not edit!


(cl:in-package fhwa2_gui-srv)


;//! \htmlinclude SolutionSelect-request.msg.html

(cl:defclass <SolutionSelect-request> (roslisp-msg-protocol:ros-message)
  ((target
    :reader target
    :initarg :target
    :type cl:string
    :initform ""))
)

(cl:defclass SolutionSelect-request (<SolutionSelect-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SolutionSelect-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SolutionSelect-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fhwa2_gui-srv:<SolutionSelect-request> is deprecated: use fhwa2_gui-srv:SolutionSelect-request instead.")))

(cl:ensure-generic-function 'target-val :lambda-list '(m))
(cl:defmethod target-val ((m <SolutionSelect-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fhwa2_gui-srv:target-val is deprecated.  Use fhwa2_gui-srv:target instead.")
  (target m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SolutionSelect-request>) ostream)
  "Serializes a message object of type '<SolutionSelect-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'target))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'target))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SolutionSelect-request>) istream)
  "Deserializes a message object of type '<SolutionSelect-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'target) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SolutionSelect-request>)))
  "Returns string type for a service object of type '<SolutionSelect-request>"
  "fhwa2_gui/SolutionSelectRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SolutionSelect-request)))
  "Returns string type for a service object of type 'SolutionSelect-request"
  "fhwa2_gui/SolutionSelectRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SolutionSelect-request>)))
  "Returns md5sum for a message object of type '<SolutionSelect-request>"
  "ec12f023325f9f4d99019044ac32e2c0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SolutionSelect-request)))
  "Returns md5sum for a message object of type 'SolutionSelect-request"
  "ec12f023325f9f4d99019044ac32e2c0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SolutionSelect-request>)))
  "Returns full string definition for message of type '<SolutionSelect-request>"
  (cl:format cl:nil "~%string target~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SolutionSelect-request)))
  "Returns full string definition for message of type 'SolutionSelect-request"
  (cl:format cl:nil "~%string target~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SolutionSelect-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'target))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SolutionSelect-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SolutionSelect-request
    (cl:cons ':target (target msg))
))
;//! \htmlinclude SolutionSelect-response.msg.html

(cl:defclass <SolutionSelect-response> (roslisp-msg-protocol:ros-message)
  ((sucess
    :reader sucess
    :initarg :sucess
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SolutionSelect-response (<SolutionSelect-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SolutionSelect-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SolutionSelect-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fhwa2_gui-srv:<SolutionSelect-response> is deprecated: use fhwa2_gui-srv:SolutionSelect-response instead.")))

(cl:ensure-generic-function 'sucess-val :lambda-list '(m))
(cl:defmethod sucess-val ((m <SolutionSelect-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fhwa2_gui-srv:sucess-val is deprecated.  Use fhwa2_gui-srv:sucess instead.")
  (sucess m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SolutionSelect-response>) ostream)
  "Serializes a message object of type '<SolutionSelect-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'sucess) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SolutionSelect-response>) istream)
  "Deserializes a message object of type '<SolutionSelect-response>"
    (cl:setf (cl:slot-value msg 'sucess) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SolutionSelect-response>)))
  "Returns string type for a service object of type '<SolutionSelect-response>"
  "fhwa2_gui/SolutionSelectResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SolutionSelect-response)))
  "Returns string type for a service object of type 'SolutionSelect-response"
  "fhwa2_gui/SolutionSelectResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SolutionSelect-response>)))
  "Returns md5sum for a message object of type '<SolutionSelect-response>"
  "ec12f023325f9f4d99019044ac32e2c0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SolutionSelect-response)))
  "Returns md5sum for a message object of type 'SolutionSelect-response"
  "ec12f023325f9f4d99019044ac32e2c0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SolutionSelect-response>)))
  "Returns full string definition for message of type '<SolutionSelect-response>"
  (cl:format cl:nil "bool sucess~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SolutionSelect-response)))
  "Returns full string definition for message of type 'SolutionSelect-response"
  (cl:format cl:nil "bool sucess~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SolutionSelect-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SolutionSelect-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SolutionSelect-response
    (cl:cons ':sucess (sucess msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SolutionSelect)))
  'SolutionSelect-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SolutionSelect)))
  'SolutionSelect-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SolutionSelect)))
  "Returns string type for a service object of type '<SolutionSelect>"
  "fhwa2_gui/SolutionSelect")