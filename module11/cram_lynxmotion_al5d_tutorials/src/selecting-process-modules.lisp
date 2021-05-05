(in-package :al5d)

(def-fact-group available-lynxmotion-al5d-process-modules (available-process-module matching-process-module)
    (<- (available-process-module lynxmotion-al5d-navigation))

    (<- (matching-process-module ?desig lynxmotion-al5d-navigation)
        (desig-prop ?desig (:type :moving)))
    (<- (matching-process-module ?desig lynxmotion-al5d-navigation)
        (desig-prop ?desig (:type :grasping))))

(defun perform-some-motion (motion-desig)
    (top-level
        (with-process-modules-running (lynxmotion-al5d-navigation)
            (cram-executive:perform motion-desig))))
