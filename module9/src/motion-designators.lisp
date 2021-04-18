(in-package :al5d)

(def-fact-group robotic-arm-motion-designators (motion-grounding)
    ;; Check for and extract necessary information for each motion

(<- (desig:motion-grounding ?desig (move-to ?destination))
    (desig-prop ?desig (:type :moving))
    (desig-prop ?desig (:destination ?destination)))

(<- (desig:motion-grounding ?desig (grasp ?distance))
    (desig-prop ?desig (:type :grasping))
    (desig-prop ?desig (:distance ?distance))))

