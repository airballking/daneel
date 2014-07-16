(ns daneel.twists
  (:refer-clojure :exclude [* - + == /])
  (:use clojure.core.matrix)
  (:use clojure.core.matrix.operators)
  (:use daneel.utils))

;;;
;;; CREATION AND DISSECTION OF TWISTS
;;;

(defn twist
  "Returns a twist representation using optional keyword arguments 'angular'
  and 'translation'. Both arguments are expected to be 3D vectors.

  Example call (creating twist from translation velocity, only):
      daneel.twist> (twist :translation [1 1 1])
      [0.0 0.0 0.0 1 1 1]"
  [& {:keys [angular translation]
      :or {angular (broadcast 0.0 [3])
           translation (broadcast 0.0 [3])}}]
  (join angular translation))

(defn dissect-twist
  "Returns the angular and translation velocity vectors of a twist representation.

  Example call:
      daneel.twist> (dissect-twist [1.0 2.0 3.0 4.0 5.0 6.0])
      {:angular [1.0 2.0 3.0] :translation [4.0 5.0 6.0]}"
  [twist]
  {:angular (subvector twist 0 3) :translation (subvector twist 3 3)})

;;;
;;; PLÜCKER COORDINATE TRANSFROMS
;;;

(defn pluecker-transform
  "Returns the Pluecker transform corresponding to rotation matrix 'rotation' and
  3D translation vector 'translation'."
  [rotation translation]
  (->
   (zero-matrix 6 6)
   (set-selection (range 3) (range 3) rotation)
   (set-selection (range 3 6) (range 3 6) rotation)
   (set-selection (range 3 6) (range 3) (mmul (skew-matrix translation) rotation))))
