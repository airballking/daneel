(ns daneel.joint-kinematics
  (:refer-clojure :exclude [* - + == /])
  (:use clojure.core.matrix)
  (:use clojure.core.matrix.operators)
  (:use daneel.transforms))

(defn forward-kinematics-solver
  "Returns a forward kinematics function for 'joint'. The returned function always
  has arity 1."
  [{:keys [type axis origin]}]
  (case type
    :revolute (comp
                (partial mmul origin)
                (partial homogeneous-transform :rotation)
                (partial axis-angle->rotation axis))
    :prismatic (comp
                (partial mmul origin)
                (partial homogeneous-transform :translation)
                (partial * (normalise axis)))
    :fixed (fn [_] origin)))
