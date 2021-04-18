(in-package :al5d)

(defparameter *ros-node-name* "al5d"
	"Name used when starting the ROS node")
(defvar *debug* nil "Used to set the program in debug mode or not")
(defvar *default-sleep-time* 3 "Default time to sleep")

(defun toggle-debug ()
	"Toggles the debug mode in the program"
	(setf *debug* (not *debug*))
	*debug*)

(defun set-sleep-time (seconds)
	"Updates the number of seconds to wait after requesting a robot movement."
	(setf *default-sleep-time* seconds)
	*default-sleep-time*)
