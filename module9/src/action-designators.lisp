(in-package :al5d)

(def-fact-group lynxmotion-al5d-action-designators (action-grounding)
    ; Demo/robotProgramming action designator
    (<- (desig:action-grounding ?desig (demo))
        (desig-prop ?desig (:type :demoing)))	

    ; Action designator for picking an object
    (<- (desig:action-grounding ?desig (pick ?destination))
        (desig-prop ?desig (:type :picking))
        (desig-prop ?desig (:from ?destination)))

    ; Action designator for placing an object
    (<- (desig:action-grounding ?desig (place ?destination))
        (desig-prop ?desig (:type :placing))
        (desig-prop ?desig (:to ?destination)))

    ; Action designator for approaching an object
    (<- (desig:action-grounding ?desig (approach ?target))
        (desig-prop ?desig (:type :approaching))
        (desig-prop ?desig (:at ?target)))

    ; Action designator for a complete pick and place exercice
    (<- (desig:action-grounding ?desig (pick-and-place ?source ?destination))
        (desig-prop ?desig (:type :picking-and-placing))
        (desig-prop ?desig (:from ?source))
        (desig-prop ?desig (:to ?destination))))
