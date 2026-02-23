;; Auto-generated. Do not edit!


(when (boundp 'restaurant_clean_task::GripperControl)
  (if (not (find-package "RESTAURANT_CLEAN_TASK"))
    (make-package "RESTAURANT_CLEAN_TASK"))
  (shadow 'GripperControl (find-package "RESTAURANT_CLEAN_TASK")))
(unless (find-package "RESTAURANT_CLEAN_TASK::GRIPPERCONTROL")
  (make-package "RESTAURANT_CLEAN_TASK::GRIPPERCONTROL"))
(unless (find-package "RESTAURANT_CLEAN_TASK::GRIPPERCONTROLREQUEST")
  (make-package "RESTAURANT_CLEAN_TASK::GRIPPERCONTROLREQUEST"))
(unless (find-package "RESTAURANT_CLEAN_TASK::GRIPPERCONTROLRESPONSE")
  (make-package "RESTAURANT_CLEAN_TASK::GRIPPERCONTROLRESPONSE"))

(in-package "ROS")





(defclass restaurant_clean_task::GripperControlRequest
  :super ros::object
  :slots (_angle ))

(defmethod restaurant_clean_task::GripperControlRequest
  (:init
   (&key
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _angle (float __angle))
   self)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; float64 _angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _angle
     (setq _angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass restaurant_clean_task::GripperControlResponse
  :super ros::object
  :slots (_success _current_angle _message ))

(defmethod restaurant_clean_task::GripperControlResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:current_angle __current_angle) 0.0)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _current_angle (float __current_angle))
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:current_angle
   (&optional __current_angle)
   (if __current_angle (setq _current_angle __current_angle)) _current_angle)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; float64 _current_angle
    8
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; float64 _current_angle
       (sys::poke _current_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _current_angle
     (setq _current_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass restaurant_clean_task::GripperControl
  :super ros::object
  :slots ())

(setf (get restaurant_clean_task::GripperControl :md5sum-) "5813b689e45b3f1ea25d03fd1a946596")
(setf (get restaurant_clean_task::GripperControl :datatype-) "restaurant_clean_task/GripperControl")
(setf (get restaurant_clean_task::GripperControl :request) restaurant_clean_task::GripperControlRequest)
(setf (get restaurant_clean_task::GripperControl :response) restaurant_clean_task::GripperControlResponse)

(defmethod restaurant_clean_task::GripperControlRequest
  (:response () (instance restaurant_clean_task::GripperControlResponse :init)))

(setf (get restaurant_clean_task::GripperControlRequest :md5sum-) "5813b689e45b3f1ea25d03fd1a946596")
(setf (get restaurant_clean_task::GripperControlRequest :datatype-) "restaurant_clean_task/GripperControlRequest")
(setf (get restaurant_clean_task::GripperControlRequest :definition-)
      "float64 angle  # 爪部目标角度
---
bool success   # 执行结果
float64 current_angle
string message # 状态信息
")

(setf (get restaurant_clean_task::GripperControlResponse :md5sum-) "5813b689e45b3f1ea25d03fd1a946596")
(setf (get restaurant_clean_task::GripperControlResponse :datatype-) "restaurant_clean_task/GripperControlResponse")
(setf (get restaurant_clean_task::GripperControlResponse :definition-)
      "float64 angle  # 爪部目标角度
---
bool success   # 执行结果
float64 current_angle
string message # 状态信息
")



(provide :restaurant_clean_task/GripperControl "5813b689e45b3f1ea25d03fd1a946596")


