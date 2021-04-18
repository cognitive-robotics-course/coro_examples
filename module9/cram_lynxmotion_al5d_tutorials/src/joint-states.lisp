;; Code adapted from https://github.com/cram2/cram/blob/boxy-melodic/cram_external_interfaces/cram_joint_states/src/joint-states.lisp

(in-package :al5d)


(defvar *joint-state-sub* nil
  "Subscriber for robot's joint state topic.")

(defvar *pos-pub* nil
  "Publisher used to send joint positions to the robot.")

(defparameter *joint-state-frequency* 10.0d0
  "How often to update the fluent in Hz")
(defvar *joint-state-timestamp* 0.0d0
  "Timestamp of the last fluent update in secs.")

(defvar *robot-joint-states-msg* (cpl:make-fluent :name :robot-joint-states)
  "ROS message containing robot's current joint states.")

(defvar *current-positions* '(0 0 0 0 0 0)
  "Current known position of the robot")

(defun init-joint-state-sub ()
  "Initializes *joint-state-sub*,
updating `*robot-joint-states-msg*' with frequency given in `*joint-state-frequency*'."
  (let ((update-every-?-secs (the double-float (/ 1.0d0 *joint-state-frequency*))))
    (declare (double-float update-every-?-secs))
    (flet ((joint-state-sub-cb (joint-state-msg)
             (when (> (the double-float (- (roslisp:ros-time) *joint-state-timestamp*))
                      update-every-?-secs)
               (setf *joint-state-timestamp* (the double-float (roslisp:ros-time)))
               (setf (cpl:value *robot-joint-states-msg*) joint-state-msg))))
      (setf *joint-state-sub*
            (roslisp:subscribe "lynxmotion_al5d/joint_states"
                               "sensor_msgs/JointState"
                               #'joint-state-sub-cb)))))

(defun init-joint-pub () 
  "Initializes *pos-pub*"
  (setf *pos-pub* (advertise "/lynxmotion_al5d/joints_positions/command" "std_msgs/Float64MultiArray")))

(defun destroy-joint-state-sub ()
  "Destroys the subscriber"
  (setf *joint-state-sub* nil))

(defun destroy-joint-pub ()
  (setf *pos-pub* nil))

; Set of function calls to make those functions run when the user launches (roslisp-utilities:startup-ros)
(roslisp-utilities:register-ros-init-function init-joint-state-sub)
(roslisp-utilities:register-ros-init-function init-joint-pub)
(roslisp-utilities:register-ros-cleanup-function destroy-joint-state-sub)
(roslisp-utilities:register-ros-cleanup-function destroy-joint-pub)

(defclass joint-state ()
  ((name :reader joint-state-name
         :initarg :name
         :type string)
   (position :reader joint-state-position
             :initarg :position
             :type float)
   (velocity :reader joint-state-velocity
             :initarg :velocity
             :type float)
   (effort :reader joint-state-effort
           :initarg :effort
           :type float)))

(defun joint-states (names)
  "Returns the joint states of type JOINT-STATE + the corresponding timestamp
as multiple values."
  (let ((last-joint-state-msg (cpl:value *robot-joint-states-msg*)))
    (values
     (mapcar (lambda (name)
               (let ((index (position
                             name
                             (roslisp:msg-slot-value last-joint-state-msg :name)
                             :test #'string-equal)))
                 (when index
                   (make-instance 'joint-state
                     :name name
                     :position (aref (roslisp:msg-slot-value last-joint-state-msg :position)
                                     index)
                     :velocity (aref (roslisp:msg-slot-value last-joint-state-msg :velocity)
                                     index)
                     :effort (aref (roslisp:msg-slot-value last-joint-state-msg :effort)
                                   index)))))
             names)
     (roslisp:msg-slot-value
      (roslisp:msg-slot-value last-joint-state-msg :header)
      :stamp))))

(defun joint-positions (names &optional state-fluent)
  "Returns the joint positions as a list + timestamp"
  (let ((last-joint-state-msg (cpl:value (or state-fluent *robot-joint-states-msg*))))
    (when last-joint-state-msg
      (values
       (mapcar (lambda (name)
                 (let ((index (position
                               name
                               (roslisp:msg-slot-value last-joint-state-msg :name)
                               :test #'string-equal)))
                   (when index
                     (aref (roslisp:msg-slot-value last-joint-state-msg :position)
                           index))))
               names)
       (roslisp:msg-slot-value
        (roslisp:msg-slot-value last-joint-state-msg :header)
        :stamp)))))

(defun joint-velocities (names &optional state-fluent)
  "Returns the joint velocities as a list + timestamp"
  (let ((last-joint-state-msg (cpl:value (or state-fluent *robot-joint-states-msg*))))
    (values
     (mapcar (lambda (name)
               (let ((index (position
                             name
                             (roslisp:msg-slot-value last-joint-state-msg :name)
                             :test #'string-equal)))
                 (when index
                   (aref (roslisp:msg-slot-value last-joint-state-msg :velocity)
                         index))))
             names)
     (roslisp:msg-slot-value
      (roslisp:msg-slot-value last-joint-state-msg :header)
      :stamp))))


(defun full-joint-states-as-hash-table (&optional state-fluent)
  (let ((last-joint-state-msg (cpl:value (or state-fluent *robot-joint-states-msg*))))
    (when last-joint-state-msg
      (let ((result-hash-table (make-hash-table :test 'equal)))
        (map 'list
             (lambda (name position)
               (setf (gethash name result-hash-table) position))
             (roslisp:msg-slot-value last-joint-state-msg :name)
             (roslisp:msg-slot-value last-joint-state-msg :position))
        ;; hpn needs the odom joints
        ;; perhaps CRAM will work with odom joints as well one day
        (let* ((robot-pose (cram-tf:robot-current-pose))
               (robot-x (cl-transforms:x (cl-transforms:origin robot-pose)))
               (robot-y (cl-transforms:y (cl-transforms:origin robot-pose))))
             (multiple-value-bind (axis angle)
                 (cl-transforms:quaternion->axis-angle
                  (cl-transforms:orientation robot-pose))
               (when (< (cl-transforms:z axis) 0)
                 (setf angle (- angle)))
               (setf (gethash "odom_x_joint" result-hash-table) robot-x)
               (setf (gethash "odom_y_joint" result-hash-table) robot-y)
               (setf (gethash "odom_z_joint" result-hash-table) angle)
               result-hash-table))))))
