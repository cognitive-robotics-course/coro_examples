(defsystem module9
    :author "Vinny Adjibi"
    :license "BSD"

    :depends-on (roslisp roslisp-utilities cram-language cram-tf sensor_msgs-msg std_msgs-msg cram-designators cram-prolog cram-process-modules cram-language-designator-support cram-executive)

    :components
        ((:module "src"
            :components
                ((:file "package")
                (:file "camera" :depends-on ("package"))
                (:file "joint-states" :depends-on ("package"))
                (:file "misc" :depends-on ("package"))
                (:file "motion-designators" :depends-on ("package"))
                (:file "location-designators" :depends-on ("package"))
                (:file "action-designators" :depends-on ("package"))
                (:file "position-publisher" :depends-on ("package" "joint-states" "misc"))
                (:file "robot-control" :depends-on ("package" "joint-states" "misc" "position-publisher"))
                (:file "robot-plans" :depends-on ("package" "joint-states" "misc" "position-publisher" "robot-control"))
                (:file "process-modules" :depends-on ("package" "misc" "joint-states" "position-publisher" "robot-control" "robot-plans" "motion-designators"))
                (:file "selecting-process-modules" :depends-on ("package" "motion-designators" "process-modules"))
                (:file "high-level-plans" :depends-on ("package" "motion-designators" "action-designators" "process-modules"))))))
