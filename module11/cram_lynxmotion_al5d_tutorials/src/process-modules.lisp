(in-package :al5d)

(def-process-module lynxmotion-al5d-navigation (motion-designator)
    (roslisp:ros-info (lynxmotion-al5d-process-modules)
                        "Lynxmotion AL5D displacement invoked with motion designator `~a'."                         motion-designator)
    (destructuring-bind (command motion) (reference motion-designator)
        (ecase command
            (move-to
                (move-to motion))
            (grasp 
                (grasp motion)))))

