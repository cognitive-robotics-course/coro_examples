(in-package :al5d)

(defun publish-joint-positions (joint-positions)
	"Takes the desired joint positions as an argument and pass them to the robot.
		The functions accepts a list of 5 or 6 joint values"
	(if *debug* 
		(format t "Checking for a connection to the susbscriber~%"))
	; First wait for getting a valid connection to the subscriber
	(do ((i 0 (* i 0)))
		((>= (num-subscribers *pos-pub*) 1))
			(sleep 0.0001))
		(if *debug* 
			(format t "Got a connection to the susbscriber.~%"))
		(let ((num-of-joints (list-length joint-positions)))
			(if (or (< num-of-joints 5) (> num-of-joints 6))
				; Stops execution if the number of value is not correct
				nil
				(progn
					; In case 5 values were passed, used the recorded gripper position.
					(if (equal 5 num-of-joints)
						(setf joint-positions (append joint-positions (last *current-positions*))))
					(publish *pos-pub*
						(make-message "std_msgs/Float64MultiArray" :data (map 'vector (lambda (joint-position) joint-position)
						                        joint-positions)))
					; Updates the current positions of the robot joints
					(setf *current-positions* joint-positions)))))
