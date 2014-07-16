(ns daneel.utils
  (:refer-clojure :exclude [* - + == /])
  (:use clojure.core.matrix)
  (:use clojure.core.matrix.operators))

;;;
;;; VECTOR PREDICATES
;;;

(defn row-vector?
  "Checks whether 'v' is a row vector."
  [v]
  (and (== (dimensionality v) 1) (< 0 (first (shape v)))))

(defn column-vector?
  "Checks whether 'v' is a column vector."
  [v]
  (and (== (dimensionality v) 2) (== 1 (second (shape v)))))

;;;
;;; TRANSPOSING VECTORS
;;;

(defn- transpose-row-vector
  "Transposes a column into a row vector."
  [v]
  (reduce (fn [result entry] (conj result  [entry])) [] v))

(defn- transpose-column-vector
  "Transposes a row vector into a column vector."
  [v]
  (reduce (fn [result entry] (conj result (first entry))) [] v))

(defn transpose-vector
  "Tranposes a vector 'v'."
  [v]
  (cond
    (row-vector? v) (transpose-row-vector v)
    (column-vector? v) (transpose-column-vector v)))

;;;
;;; SKEW MATRIX CREATION
;;;

(defn skew-matrix
  "Returns the skew-matrix of a 3D vector."
  [[x y z]]
  [[0 (- z) y]
   [z 0 (- x)]
   [(- y) x 0]])
