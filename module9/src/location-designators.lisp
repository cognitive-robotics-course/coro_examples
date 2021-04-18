(in-package :al5d)

(defparameter *effector-length* 0.086d0)

(defun robot-programming-goals-generator (designator)
  (declare (type location-designator designator))
  ;; Step no so that we know where we are
  (with-desig-props (step-no) designator
    (let* ((?object_x 0.0d0)
           (?object_y 0.187)
           (?object_z 0.0d0)
           (?object_theta (/ pi 2))
           (?example_x ?object_x)
           (?example_y ?object_y)
           (?example_z 0.216d0)
           (?side_x 0.100d0)
           (?tray_x 0.150d0)
           (?tray_y 0.100d0)
           (?tray_z 0.100d0)
           (?initial-approach-distance 0.05d0)
           (?approach-distance ?initial-approach-distance)
           (?delta 0.002)
           (?next-pose nil))

      (ecase step-no
        (1 
          (loop for i in '(1 2 3 4 5 6)
            do (ecase i
              (1 
                (setf ?next-pose (cl-tf:make-pose
                                  (cl-tf:v+ 
                                    (cl-tf:make-3d-vector 0 0 (* -1 *effector-length*)) 
                                  (cl-tf:make-3d-vector ?example_x ?example_y (+ ?example_z *effector-length*)))
                                  (cl-tf:euler->quaternion :ay 0 :az 0))))
              (2 
                (setf ?next-pose (cl-tf:make-pose 
                                  (cl-tf:v+ 
                                    (cl-tf:make-3d-vector 0 0 (* -1 *effector-length*)) 
                                  (cl-tf:make-3d-vector ?example_x ?example_y (+ ?example_z *effector-length*)))
                                  (cl-tf:euler->quaternion :ay 0 :az (/ pi 2)))))
              (3
                (setf ?next-pose (cl-tf:make-pose 
                                  (cl-tf:v+ 
                                    (cl-tf:make-3d-vector 0 (* -1 *effector-length*) 0) 
                                  (cl-tf:make-3d-vector ?example_x (+ ?example_y *effector-length*) ?example_z))
                                  (cl-tf:euler->quaternion :ay (/ pi 2) :az (/ pi 2)))))
              (4 
                (setf ?next-pose (cl-tf:make-pose 
                                  (cl-tf:v+ 
                                    (cl-tf:make-3d-vector 0 (* -1 *effector-length*) 0) 
                                  (cl-tf:make-3d-vector ?example_x (+ ?example_y *effector-length*) ?example_z))
                                  (cl-tf:euler->quaternion :ay (/ pi 2) :az (/ pi 2)))))
              (5 
                (setf ?next-pose (cl-tf:make-pose 
                                  (cl-tf:v+ 
                                    (cl-tf:make-3d-vector 0 (* -1 *effector-length*) 0) 
                                  (cl-tf:make-3d-vector ?example_x (+ ?example_y *effector-length*) 0.040))
                                  (cl-tf:euler->quaternion :ay (/ pi 2) :az (/ pi 2)))))
              (6 
                (setf ?next-pose (cl-tf:make-pose 
                                  (cl-tf:v+ 
                                    (cl-tf:make-3d-vector 0 0 *effector-length*) 
                                    (cl-tf:make-3d-vector ?object_x ?object_y (+ ?object_z)) 
                                    (cl-tf:make-3d-vector 0 0 ?approach-distance))
                                  (cl-tf:euler->quaternion :ay pi :az ?object_theta)))))
            collect ?next-pose))
        (2 
          (loop for approach-distance = ?approach-distance then (- approach-distance ?delta) while (>= approach-distance 0)
              collect (cl-tf:make-pose
                        (cl-tf:v+ 
                            (cl-tf:make-3d-vector 0 0 *effector-length*) 
                            (cl-tf:make-3d-vector ?object_x ?object_y ?object_z) 
                            (cl-tf:make-3d-vector 0 0 approach-distance))
                        (cl-tf:euler->quaternion :ay pi :az ?object_theta))))
        (3 
          (setf ?approach-distance 0)
          (loop for approach-distance = ?approach-distance then (+ approach-distance ?delta) while (<= approach-distance ?initial-approach-distance)
              collect (cl-tf:make-pose
                        (cl-tf:v+ 
                            (cl-tf:make-3d-vector 0 0 *effector-length*) 
                            (cl-tf:make-3d-vector ?object_x ?object_y ?object_z) 
                            (cl-tf:make-3d-vector 0 0 approach-distance))
                        (cl-tf:euler->quaternion :ay pi :az ?object_theta))))
        (4
          (loop for value = ?example_x then (+ value ?delta) while (<= value (+ ?example_x ?side_x))
            collect (cl-tf:make-pose
                        (cl-tf:v+ 
                            (cl-tf:make-3d-vector 0 0 *effector-length*) 
                            (cl-tf:make-3d-vector value ?example_y (- ?example_z *effector-length*)))
                        (cl-tf:euler->quaternion :ay pi :az (/ pi 2)))))
        ; move above the tray
        (5
            (loop repeat 1
                collect (cl-tf:make-pose
                        (cl-tf:v+ 
                            (cl-tf:make-3d-vector 0 0 *effector-length*) 
                            (cl-tf:make-3d-vector ?tray_x ?tray_y ?tray_z))
                            (cl-tf:euler->quaternion :ay pi :az (/ pi 2)))))
        (6
          (loop for value = (+ ?example_x ?side_x) then (- value ?delta) while (>= value ?example_x)
            collect (cl-tf:make-pose
                        (cl-tf:v+ 
                            (cl-tf:make-3d-vector 0 0 *effector-length*) 
                        (cl-tf:make-3d-vector value ?example_y (- ?example_z *effector-length*)))
                        (cl-tf:euler->quaternion :ay pi :az (/ pi 2)))))
        (7
          (setf ?approach-distance ?initial-approach-distance)
          (loop repeat 1
            collect (cl-tf:make-pose
                        (cl-tf:v+ 
                            (cl-tf:make-3d-vector 0 0 *effector-length*) 
                            (cl-tf:make-3d-vector ?object_x ?object_y ?object_z)
                            (cl-tf:make-3d-vector 0 0 ?approach-distance))
                        (cl-tf:euler->quaternion :ay pi :az ?object_theta))))
        (8
          (setf ?approach-distance ?initial-approach-distance)
          (loop repeat 1
            collect (cl-tf:make-pose
                        (cl-tf:v+ 
                            (cl-tf:make-3d-vector 0 0 *effector-length*) 
                            (cl-tf:make-3d-vector ?object_x ?object_y ?object_z))
                        (cl-tf:euler->quaternion :ay pi :az ?object_theta))))
        (9
          (setf ?approach-distance ?initial-approach-distance)
          (loop repeat 1
            collect (cl-tf:make-pose
                        (cl-tf:v+ 
                            (cl-tf:make-3d-vector 0 0 *effector-length*) 
                            (cl-tf:make-3d-vector ?object_x ?object_y ?object_z)
                            (cl-tf:make-3d-vector 0 0 ?approach-distance))
                        (cl-tf:euler->quaternion :ay pi :az ?object_theta))))
        (10 
            ; move to example pose
            (loop repeat 1
                collect (cl-tf:make-pose
                        (cl-tf:v+ 
                            (cl-tf:make-3d-vector 0 0 *effector-length*) 
                            (cl-tf:make-3d-vector ?example_x ?example_y (- ?example_z *effector-length*)))
                            (cl-tf:euler->quaternion :ay pi :az (/ pi 2)))))
        (11 
            ; move horizontally in X direction
            (loop repeat 1
                collect (cl-tf:make-pose
                        (cl-tf:v+ 
                            (cl-tf:make-3d-vector 0 0 *effector-length*) 
                            (cl-tf:make-3d-vector (+ ?example_x ?side_x) ?example_y (- ?example_z *effector-length*)))
                            (cl-tf:euler->quaternion :ay pi :az (/ pi 2)))))
        (12
            ; move above the tray
            (loop repeat 1
                collect (cl-tf:make-pose
                        (cl-tf:v+ 
                            (cl-tf:make-3d-vector 0 0 *effector-length*) 
                            (cl-tf:make-3d-vector ?tray_x ?tray_y ?tray_z))
                            (cl-tf:euler->quaternion :ay pi :az (/ pi 2)))))
        (13
            ; move back to example pose
            (loop repeat 1
                collect (cl-tf:make-pose
                        (cl-tf:v+ 
                            (cl-tf:make-3d-vector 0 0 *effector-length*) 
                            (cl-tf:make-3d-vector ?example_x ?example_y (- ?example_z *effector-length*)))
                            (cl-tf:euler->quaternion :ay pi :az (/ pi 2)))))

))))

(register-location-generator 5 robot-programming-goals-generator)

