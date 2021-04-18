(in-package :al5d)

(defvar *camera-sub* nil
	"Subscriber to retrive images from the camera sensor attached to the robot")

(defvar *robot-external-camera-msg* (cpl:make-fluent :name :captured-image)
  "ROS message containing the image captured from the camera sensor.")

(defparameter *image-height* 480
	"Height of the image captured by the sensor.")

(defparameter *image-width* 640
	"Width of the image captured by the sensor.")

(defun init-camera-sub ()
	"Initializes the subscriber to the camera topic"
	(setf *camera-sub*
		(roslisp:subscribe "lynxmotion_al5d/al5d_external_vision/image_raw"
							"sensor_msgs/Image"
                               #'camera-sub-cb)))

(defun destroy-camera-sub ()
	"Destroys the subscriber to the camera topic."
	(setf *camera-sub* nil))

(defun camera-sub-cb (image-msg)
	"Callback used by the subscriber. Saves the captured image to the fluent"
	(setf (cpl:value *robot-external-camera-msg*) image-msg))

;;; Set of function calls to make those functions run when the user launches (roslisp-utilities:startup-ros)
(roslisp-utilities:register-ros-init-function init-camera-sub)
(roslisp-utilities:register-ros-cleanup-function destroy-camera-sub)

(defun last-captured-image ()
	"Returns the last captured image as a matrix."
    (slot-value (cpl:value *robot-external-camera-msg*) 'data))
;	(with-fields (data)  (cpl:value *robot-external-camera-msg*)
;        data))
