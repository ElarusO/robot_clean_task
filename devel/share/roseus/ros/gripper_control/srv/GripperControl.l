;; Auto-generated. Do not edit!


(when (boundp 'gripper_control::GripperControl)
  (if (not (find-package "GRIPPER_CONTROL"))
    (make-package "GRIPPER_CONTROL"))
  (shadow 'GripperControl (find-package "GRIPPER_CONTROL")))
(unless (find-package "GRIPPER_CONTROL::GRIPPERCONTROL")
  (make-package "GRIPPER_CONTROL::GRIPPERCONTROL"))
(unless (find-package "GRIPPER_CONTROL::GRIPPERCONTROLREQUEST")
  (make-package "GRIPPER_CONTROL::GRIPPERCONTROLREQUEST"))
(unless (find-package "GRIPPER_CONTROL::GRIPPERCONTROLRESPONSE")
  (make-package "GRIPPER_CONTROL::GRIPPERCONTROLRESPONSE"))

(in-package "ROS")





(defclass gripper_control::GripperControlRequest
  :super ros::object
  :slots (_gripper_angle ))

(defmethod gripper_control::GripperControlRequest
  (:init
   (&key
    ((:gripper_angle __gripper_angle) 0.0)
    )
   (send-super :init)
   (setq _gripper_angle (float __gripper_angle))
   self)
  (:gripper_angle
   (&optional __gripper_angle)
   (if __gripper_angle (setq _gripper_angle __gripper_angle)) _gripper_angle)
  (:serialization-length
   ()
   (+
    ;; float32 _gripper_angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _gripper_angle
       (sys::poke _gripper_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _gripper_angle
     (setq _gripper_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass gripper_control::GripperControlResponse
  :super ros::object
  :slots (_result _current_angle _info ))

(defmethod gripper_control::GripperControlResponse
  (:init
   (&key
    ((:result __result) nil)
    ((:current_angle __current_angle) 0.0)
    ((:info __info) "")
    )
   (send-super :init)
   (setq _result __result)
   (setq _current_angle (float __current_angle))
   (setq _info (string __info))
   self)
  (:result
   (&optional (__result :null))
   (if (not (eq __result :null)) (setq _result __result)) _result)
  (:current_angle
   (&optional __current_angle)
   (if __current_angle (setq _current_angle __current_angle)) _current_angle)
  (:info
   (&optional __info)
   (if __info (setq _info __info)) _info)
  (:serialization-length
   ()
   (+
    ;; bool _result
    1
    ;; float32 _current_angle
    4
    ;; string _info
    4 (length _info)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;; float32 _current_angle
       (sys::poke _current_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _info
       (write-long (length _info) s) (princ _info s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _current_angle
     (setq _current_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _info
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _info (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass gripper_control::GripperControl
  :super ros::object
  :slots ())

(setf (get gripper_control::GripperControl :md5sum-) "55a3885cd6a0cb48fd314894a776c441")
(setf (get gripper_control::GripperControl :datatype-) "gripper_control/GripperControl")
(setf (get gripper_control::GripperControl :request) gripper_control::GripperControlRequest)
(setf (get gripper_control::GripperControl :response) gripper_control::GripperControlResponse)

(defmethod gripper_control::GripperControlRequest
  (:response () (instance gripper_control::GripperControlResponse :init)))

(setf (get gripper_control::GripperControlRequest :md5sum-) "55a3885cd6a0cb48fd314894a776c441")
(setf (get gripper_control::GripperControlRequest :datatype-) "gripper_control/GripperControlRequest")
(setf (get gripper_control::GripperControlRequest :definition-)
      "# 请求：目标开合角度（单位：度）
float32 gripper_angle
---
# 响应：执行结果、当前实际角度、说明信息
bool result
float32 current_angle
string info
")

(setf (get gripper_control::GripperControlResponse :md5sum-) "55a3885cd6a0cb48fd314894a776c441")
(setf (get gripper_control::GripperControlResponse :datatype-) "gripper_control/GripperControlResponse")
(setf (get gripper_control::GripperControlResponse :definition-)
      "# 请求：目标开合角度（单位：度）
float32 gripper_angle
---
# 响应：执行结果、当前实际角度、说明信息
bool result
float32 current_angle
string info
")



(provide :gripper_control/GripperControl "55a3885cd6a0cb48fd314894a776c441")


