;; Auto-generated. Do not edit!


(when (boundp 'wall_following::FindWall)
  (if (not (find-package "WALL_FOLLOWING"))
    (make-package "WALL_FOLLOWING"))
  (shadow 'FindWall (find-package "WALL_FOLLOWING")))
(unless (find-package "WALL_FOLLOWING::FINDWALL")
  (make-package "WALL_FOLLOWING::FINDWALL"))
(unless (find-package "WALL_FOLLOWING::FINDWALLREQUEST")
  (make-package "WALL_FOLLOWING::FINDWALLREQUEST"))
(unless (find-package "WALL_FOLLOWING::FINDWALLRESPONSE")
  (make-package "WALL_FOLLOWING::FINDWALLRESPONSE"))

(in-package "ROS")





(defclass wall_following::FindWallRequest
  :super ros::object
  :slots ())

(defmethod wall_following::FindWallRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass wall_following::FindWallResponse
  :super ros::object
  :slots (_wallfound ))

(defmethod wall_following::FindWallResponse
  (:init
   (&key
    ((:wallfound __wallfound) nil)
    )
   (send-super :init)
   (setq _wallfound __wallfound)
   self)
  (:wallfound
   (&optional (__wallfound :null))
   (if (not (eq __wallfound :null)) (setq _wallfound __wallfound)) _wallfound)
  (:serialization-length
   ()
   (+
    ;; bool _wallfound
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _wallfound
       (if _wallfound (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _wallfound
     (setq _wallfound (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass wall_following::FindWall
  :super ros::object
  :slots ())

(setf (get wall_following::FindWall :md5sum-) "66229cd821efb4126c80f35757f3c73e")
(setf (get wall_following::FindWall :datatype-) "wall_following/FindWall")
(setf (get wall_following::FindWall :request) wall_following::FindWallRequest)
(setf (get wall_following::FindWall :response) wall_following::FindWallResponse)

(defmethod wall_following::FindWallRequest
  (:response () (instance wall_following::FindWallResponse :init)))

(setf (get wall_following::FindWallRequest :md5sum-) "66229cd821efb4126c80f35757f3c73e")
(setf (get wall_following::FindWallRequest :datatype-) "wall_following/FindWallRequest")
(setf (get wall_following::FindWallRequest :definition-)
      "---
bool wallfound
")

(setf (get wall_following::FindWallResponse :md5sum-) "66229cd821efb4126c80f35757f3c73e")
(setf (get wall_following::FindWallResponse :datatype-) "wall_following/FindWallResponse")
(setf (get wall_following::FindWallResponse :definition-)
      "---
bool wallfound
")



(provide :wall_following/FindWall "66229cd821efb4126c80f35757f3c73e")


