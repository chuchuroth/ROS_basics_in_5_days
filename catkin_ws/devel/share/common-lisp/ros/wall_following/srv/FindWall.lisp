; Auto-generated. Do not edit!


(cl:in-package wall_following-srv)


;//! \htmlinclude FindWall-request.msg.html

(cl:defclass <FindWall-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass FindWall-request (<FindWall-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FindWall-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FindWall-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wall_following-srv:<FindWall-request> is deprecated: use wall_following-srv:FindWall-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FindWall-request>) ostream)
  "Serializes a message object of type '<FindWall-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FindWall-request>) istream)
  "Deserializes a message object of type '<FindWall-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FindWall-request>)))
  "Returns string type for a service object of type '<FindWall-request>"
  "wall_following/FindWallRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FindWall-request)))
  "Returns string type for a service object of type 'FindWall-request"
  "wall_following/FindWallRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FindWall-request>)))
  "Returns md5sum for a message object of type '<FindWall-request>"
  "66229cd821efb4126c80f35757f3c73e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FindWall-request)))
  "Returns md5sum for a message object of type 'FindWall-request"
  "66229cd821efb4126c80f35757f3c73e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FindWall-request>)))
  "Returns full string definition for message of type '<FindWall-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FindWall-request)))
  "Returns full string definition for message of type 'FindWall-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FindWall-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FindWall-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FindWall-request
))
;//! \htmlinclude FindWall-response.msg.html

(cl:defclass <FindWall-response> (roslisp-msg-protocol:ros-message)
  ((wallfound
    :reader wallfound
    :initarg :wallfound
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FindWall-response (<FindWall-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FindWall-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FindWall-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wall_following-srv:<FindWall-response> is deprecated: use wall_following-srv:FindWall-response instead.")))

(cl:ensure-generic-function 'wallfound-val :lambda-list '(m))
(cl:defmethod wallfound-val ((m <FindWall-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wall_following-srv:wallfound-val is deprecated.  Use wall_following-srv:wallfound instead.")
  (wallfound m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FindWall-response>) ostream)
  "Serializes a message object of type '<FindWall-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'wallfound) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FindWall-response>) istream)
  "Deserializes a message object of type '<FindWall-response>"
    (cl:setf (cl:slot-value msg 'wallfound) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FindWall-response>)))
  "Returns string type for a service object of type '<FindWall-response>"
  "wall_following/FindWallResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FindWall-response)))
  "Returns string type for a service object of type 'FindWall-response"
  "wall_following/FindWallResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FindWall-response>)))
  "Returns md5sum for a message object of type '<FindWall-response>"
  "66229cd821efb4126c80f35757f3c73e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FindWall-response)))
  "Returns md5sum for a message object of type 'FindWall-response"
  "66229cd821efb4126c80f35757f3c73e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FindWall-response>)))
  "Returns full string definition for message of type '<FindWall-response>"
  (cl:format cl:nil "bool wallfound~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FindWall-response)))
  "Returns full string definition for message of type 'FindWall-response"
  (cl:format cl:nil "bool wallfound~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FindWall-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FindWall-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FindWall-response
    (cl:cons ':wallfound (wallfound msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FindWall)))
  'FindWall-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FindWall)))
  'FindWall-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FindWall)))
  "Returns string type for a service object of type '<FindWall>"
  "wall_following/FindWall")