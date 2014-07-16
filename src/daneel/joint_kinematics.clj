(ns daneel.joint-kinematics
  (:refer-clojure :exclude [* - + == /])
  (:use clojure.core.matrix)
  (:use clojure.core.matrix.operators)
  (:use daneel.transforms))

;;;
;;; PREDICATES TO CHECK FOR JOINT TYPES
;;;

(defn- joint-type-predicate
  "Returns a predicate of arity 1 which checks whether a passed joint is
  of type 'joint-type'."
  [joint-type]
  (fn [joint] (= (:type joint) joint-type)))

(def fixed-joint? (joint-type-predicate :fixed))

(def prismatic-joint? (joint-type-predicate :prismatic))

(def revolute-joint? (joint-type-predicate :revolute))

;;;
;;; FORWARD KINEMATICS SOLVERS
;;;

(defn revolute-joint-fk-solver
  "Returns a function of arity 1 which corresponds to the forward kinematics
  of the input joint, assuming that it is a revolute joint."
  [{:keys [axis origin]}]
  (comp
   (partial mmul origin)
   (partial homogeneous-transform :rotation)
   (partial axis-angle->rotation axis)))

(defn prismatic-joint-fk-solver
  "Returns a function of arity 1 which corresponds to the forward kinematics
  of the input joint, assuming that it is a prismatic joint."
  [{:keys [axis origin]}]
  (comp
   (partial mmul origin)
   (partial homogeneous-transform :translation)
   (partial * (normalise axis))))

(defn fixed-joint-fk-solver
  "Returns the transform corresponding to the input joint, assuming that
  it is a fixed joint."
  [{:keys [origin]}]
  origin)

(defn forward-kinematics-solver
  "Returns a forward kinematics function for 'joint'."
  [joint]
  (cond
   (revolute-joint? joint) (revolute-joint-fk-solver joint)
   (prismatic-joint? joint) (prismatic-joint-fk-solver joint)
   (fixed-joint? joint) (fixed-joint-fk-solver joint))
  ;; (case type
  ;;   :revolute (comp
  ;;               (partial mmul origin)
  ;;               (partial homogeneous-transform :rotation)
  ;;               (partial axis-angle->rotation axis))
  ;;   :prismatic (comp
  ;;               (partial mmul origin)
  ;;               (partial homogeneous-transform :translation)
  ;;               (partial * (normalise axis)))
  ;;   :fixed (fn [_] origin))
  )
