; Auto-generated. Do not edit!


(cl:in-package shm_led_mouth-msg)


;//! \htmlinclude LEDMatrixRow.msg.html

(cl:defclass <LEDMatrixRow> (roslisp-msg-protocol:ros-message)
  ((column
    :reader column
    :initarg :column
    :type shm_led_mouth-msg:LEDMatrixRGB
    :initform (cl:make-instance 'shm_led_mouth-msg:LEDMatrixRGB)))
)

(cl:defclass LEDMatrixRow (<LEDMatrixRow>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LEDMatrixRow>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LEDMatrixRow)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name shm_led_mouth-msg:<LEDMatrixRow> is deprecated: use shm_led_mouth-msg:LEDMatrixRow instead.")))

(cl:ensure-generic-function 'column-val :lambda-list '(m))
(cl:defmethod column-val ((m <LEDMatrixRow>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader shm_led_mouth-msg:column-val is deprecated.  Use shm_led_mouth-msg:column instead.")
  (column m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LEDMatrixRow>) ostream)
  "Serializes a message object of type '<LEDMatrixRow>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'column) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LEDMatrixRow>) istream)
  "Deserializes a message object of type '<LEDMatrixRow>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'column) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LEDMatrixRow>)))
  "Returns string type for a message object of type '<LEDMatrixRow>"
  "shm_led_mouth/LEDMatrixRow")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LEDMatrixRow)))
  "Returns string type for a message object of type 'LEDMatrixRow"
  "shm_led_mouth/LEDMatrixRow")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LEDMatrixRow>)))
  "Returns md5sum for a message object of type '<LEDMatrixRow>"
  "5a0313057dae76e530e4ef99ceb0eca4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LEDMatrixRow)))
  "Returns md5sum for a message object of type 'LEDMatrixRow"
  "5a0313057dae76e530e4ef99ceb0eca4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LEDMatrixRow>)))
  "Returns full string definition for message of type '<LEDMatrixRow>"
  (cl:format cl:nil "LEDMatrixRGB column~%================================================================================~%MSG: shm_led_mouth/LEDMatrixRGB~%uint32 r~%uint32 g~%uint32 b~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LEDMatrixRow)))
  "Returns full string definition for message of type 'LEDMatrixRow"
  (cl:format cl:nil "LEDMatrixRGB column~%================================================================================~%MSG: shm_led_mouth/LEDMatrixRGB~%uint32 r~%uint32 g~%uint32 b~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LEDMatrixRow>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'column))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LEDMatrixRow>))
  "Converts a ROS message object to a list"
  (cl:list 'LEDMatrixRow
    (cl:cons ':column (column msg))
))
