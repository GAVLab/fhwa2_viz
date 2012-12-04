; Auto-generated. Do not edit!


(cl:in-package fhwa2_gui-srv)


;//! \htmlinclude SolutionEnable-request.msg.html

(cl:defclass <SolutionEnable-request> (roslisp-msg-protocol:ros-message)
  ((enable
    :reader enable
    :initarg :enable
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SolutionEnable-request (<SolutionEnable-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SolutionEnable-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SolutionEnable-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fhwa2_gui-srv:<SolutionEnable-request> is deprecated: use fhwa2_gui-srv:SolutionEnable-request instead.")))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <SolutionEnable-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fhwa2_gui-srv:enable-val is deprecated.  Use fhwa2_gui-srv:enable instead.")
  (enable m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SolutionEnable-request>) ostream)
  "Serializes a message object of type '<SolutionEnable-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SolutionEnable-request>) istream)
  "Deserializes a message object of type '<SolutionEnable-request>"
    (cl:setf (cl:slot-value msg 'enable) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SolutionEnable-request>)))
  "Returns string type for a service object of type '<SolutionEnable-request>"
  "fhwa2_gui/SolutionEnableRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SolutionEnable-request)))
  "Returns string type for a service object of type 'SolutionEnable-request"
  "fhwa2_gui/SolutionEnableRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SolutionEnable-request>)))
  "Returns md5sum for a message object of type '<SolutionEnable-request>"
  "3ea372bdd9923da8a6c7ae2db934a6cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SolutionEnable-request)))
  "Returns md5sum for a message object of type 'SolutionEnable-request"
  "3ea372bdd9923da8a6c7ae2db934a6cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SolutionEnable-request>)))
  "Returns full string definition for message of type '<SolutionEnable-request>"
  (cl:format cl:nil "~%bool enable~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SolutionEnable-request)))
  "Returns full string definition for message of type 'SolutionEnable-request"
  (cl:format cl:nil "~%bool enable~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SolutionEnable-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SolutionEnable-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SolutionEnable-request
    (cl:cons ':enable (enable msg))
))
;//! \htmlinclude SolutionEnable-response.msg.html

(cl:defclass <SolutionEnable-response> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SolutionEnable-response (<SolutionEnable-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SolutionEnable-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SolutionEnable-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fhwa2_gui-srv:<SolutionEnable-response> is deprecated: use fhwa2_gui-srv:SolutionEnable-response instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <SolutionEnable-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fhwa2_gui-srv:state-val is deprecated.  Use fhwa2_gui-srv:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SolutionEnable-response>) ostream)
  "Serializes a message object of type '<SolutionEnable-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'state) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SolutionEnable-response>) istream)
  "Deserializes a message object of type '<SolutionEnable-response>"
    (cl:setf (cl:slot-value msg 'state) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SolutionEnable-response>)))
  "Returns string type for a service object of type '<SolutionEnable-response>"
  "fhwa2_gui/SolutionEnableResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SolutionEnable-response)))
  "Returns string type for a service object of type 'SolutionEnable-response"
  "fhwa2_gui/SolutionEnableResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SolutionEnable-response>)))
  "Returns md5sum for a message object of type '<SolutionEnable-response>"
  "3ea372bdd9923da8a6c7ae2db934a6cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SolutionEnable-response)))
  "Returns md5sum for a message object of type 'SolutionEnable-response"
  "3ea372bdd9923da8a6c7ae2db934a6cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SolutionEnable-response>)))
  "Returns full string definition for message of type '<SolutionEnable-response>"
  (cl:format cl:nil "~%bool state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SolutionEnable-response)))
  "Returns full string definition for message of type 'SolutionEnable-response"
  (cl:format cl:nil "~%bool state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SolutionEnable-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SolutionEnable-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SolutionEnable-response
    (cl:cons ':state (state msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SolutionEnable)))
  'SolutionEnable-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SolutionEnable)))
  'SolutionEnable-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SolutionEnable)))
  "Returns string type for a service object of type '<SolutionEnable>"
  "fhwa2_gui/SolutionEnable")