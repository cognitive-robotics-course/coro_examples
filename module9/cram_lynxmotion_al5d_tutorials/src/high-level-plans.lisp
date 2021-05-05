(in-package :al5d)

(defparameter *gripper-open* 0.03d0 "Gripper opening distance in m")
(defparameter *gripper-closed* 0.0d0 "Gripper closed distance")
(defparameter *object-approach-distance* 0.02d0 "Approched distance in the -z direction")
(defparameter *grasp-z* 0.005d0 "Grasp z value relative to object and destination poses")
(defparameter *grasp-theta* (* -1 pi) "Grasp angle in the y direction")
(defparameter *end-effector-length* 0.086d0 "The length of the end effector")

(defun pick (?destination)
        ; First we open the gripper
        (let  ((?goal (cl-transforms:make-pose
                        (cl-transforms:v+ (cl-transforms:origin ?destination) 
                            (cl-transforms:make-3d-vector 0 0 (+ *end-effector-length* *grasp-z*)))
                        (cl-tf:q* (cl-tf:orientation ?destination) (cl-tf:euler->quaternion :ay pi :az 0 ))))
                (?gripper-open *gripper-open*)
                (?gripper-closed *gripper-closed*))

            ; Open the end effector
            (exe:perform (a motion (type grasping) (distance ?gripper-open)))
            ; Move to the robot approach pose
            (exe:perform (an action (type approaching) (at ?goal)))
            ; Now we go to the grasp pose
            (exe:perform (a motion (type moving) (destination ?goal)))
            ; Close the fingers
            (exe:perform (a motion (type grasping) (distance ?gripper-closed)))
            ; Go back to the approach pose
            (exe:perform (an action (type approaching) (at ?goal)))))

(defun place (?destination)
        ; First we open the gripper
        (let  ((?goal (cl-transforms:make-pose
                        (cl-transforms:v+ (cl-transforms:origin ?destination) 
                            (cl-transforms:make-3d-vector 0 0 (+ *end-effector-length* *grasp-z*)))
                        (cl-tf:q* (cl-tf:orientation ?destination) (cl-tf:euler->quaternion :ay pi :az 0 ))))
                (?gripper-open *gripper-open*)
                (?gripper-closed *gripper-closed*))

            ; Move to the robot approach pose
            (exe:perform (an action (type approaching) (at ?goal)))
            ; Now we go to the grasp pose
            (exe:perform (a motion (type moving) (destination ?goal)))
            ; Open the end effector
            (exe:perform (a motion (type grasping) (distance ?gripper-open)))
            ; Go back to the approach pose
            (exe:perform (an action (type approaching) (at ?goal)))
            ; Close the fingers
            (exe:perform (a motion (type grasping) (distance ?gripper-closed)))))

(defun approach (?target)
    ; Sends the robot to a specific approach pose
    (let ((?approach-pose (cl-transforms:make-pose 
                            (cl-transforms:v+ (cl-transforms:origin ?target)
                                (cl-transforms:make-3d-vector 0 0 *object-approach-distance*))
                            (cl-transforms:orientation ?target))))

        (exe:perform (a motion (type moving) (destination ?approach-pose)))))

(defun pick-and-place (?from ?to)
    ; First pick
    (exe:perform (an action (type picking) (from ?from)))
    ; Then place
    (exe:perform (an action (type placing) (to ?to))))

(defun demo()
    (go-home)
    (set-sleep-time 1)
    (dolist (?i '(1 2 3 4 5 6 7 8 9 10 11 12 13))
      (let ((goal (a location (type location) (step-no ?i))))
        (loop for x = goal then (next-solution x) while x
          do (let ((?target (reference x)))
              (exe:perform (a motion (type moving) (destination ?target)))
              (if (equal ?i 1) (sleep 3)))))
      (sleep 3)
      (if (member (+ ?i 1) '(1 5 7 9 12))
        (exe:perform (a motion (type grasping) (distance 0.03))))
      (if (member (+ ?i 1) '(2 8))
        (progn 
          (exe:perform (a motion (type grasping) (distance 0.015)))
          (sleep 1))))
    (go-home))
