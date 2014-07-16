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
  of a single revolute joint. 'axis' denotes the axis of rotation, and 'origin'
  is a possible homogeneous transform offset between parent-frame and joint.

  Example call to obtain the forward kinematics function (rotation around x,
  no offset transformation):

      daneel.joint-kinematics> (def revolute-fk
                                 (revolute-joint-fk-solver
                                   [1 0 0] (identity-matrix 4)))
      #'daneel.joint-kinematics/revolute-fk

  Subsequently, calling the fk function with a joint state of 45 degrees:
      daneel.joint-kinematics> (revolute-fk (/ Math/PI 4))
      [[1.0 0.0 0.0 0.0]
       [0.0 0.7071067811865476 -0.7071067811865475 0.0]
       [0.0 0.7071067811865475 0.7071067811865476 0.0]
       [0.0 0.0 0.0 1.0]]"
  [axis origin]
  (comp
   (partial mmul origin)
   (partial homogeneous-transform :rotation)
   (partial axis-angle->rotation axis)))

(defn prismatic-joint-fk-solver
  "Returns a function of arity 1 which corresponds to the forward kinematics
  of a single prismatic joint. 'axis' denotes the axis of translation, and
  'origin' is a possible homogeneous transform offset between parent-frame
  and the joint.

  Example call to obtain the forward kinematics function (translation along z,
  no offset transformation):

      daneel.joint-kinematics> (def prismatic-fk
                                 (prismatic-joint-fk-solver
                                   [0 0 1] (identity-matrix 4))
      #'daneel.joint-kinematics/prismatic-fk

  Subsequently, calling the fk function with a joint state of 5cm:
      daneel.joint-kinematics> (prismatic-fk 0.05)
      [[1.0 0.0 0.0 0.0]
       [0.0 1.0 0.0 0.0]
       [0.0 0.0 1.0 0.05]
       [0.0 0.0 0.0 1.0]]"
  [axis origin]
  (comp
   (partial mmul origin)
   (partial homogeneous-transform :translation)
   (partial * (normalise axis))))

(defn fixed-joint-fk-solver
  "Returns the transform corresponding to the input joint, assuming that
  it is a fixed joint."
  [{:keys [origin]}]
  origin)
