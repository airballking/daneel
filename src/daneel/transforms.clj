(ns daneel.transforms
  (:refer-clojure :exclude [* - + == /])
  (:use clojure.core.matrix)
  (:use clojure.core.matrix.operators)
  (:use daneel.utils))

;;;
;;; ALGORITHM TO CREATE ROTATION FROM AXIS/ANGLE TAKEN FROM:
;;;
;;; http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/
;;;

(defn axis-angle->rotation
  "Creates a rotation matrix which corresponds to a rotation of 'angle' radians
  around the 3d-axis 'axis'."
  [axis rad-angle]
  ;; TODO(Georg): throw exception if axis has length zero
  ;;  (when (not (== 0 (length axis))))
  (let [[x y z] (normalise axis)
        c (Math/cos rad-angle)
        s (Math/sin rad-angle)
        t (- 1 c)]
    [[(+ (* t x x) c) (- (* t x y) (* z s)) (+ (* t x z) (* y s))]
     [(+ (* t x y) (* z s)) (+ (* t y y) c) (- (* t y z) (* x s))]
     [(- (* t x z) (* y s)) (+ (* t y z) (* x s)) (+ (* t z z) c)]]))

;;;
;;; QUATERNIONS are a different representation of rotations.
;;; Internally, we use rotation matrices to perform our computations.
;;; However, quaternions may be used as input and output
;;; representations of rotations. Below, you can find conversion
;;; functions.
;;;
;;; QUATERNION ALGORITHMS TAKEN FROM HERE:
;;;
;;; http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
;;;

(defn quaternion->rotation
  "Creates a rotation matrix from a quaternion."
  [{:keys [x y z w]}]
  (let [xx (* x x)
        yy (* y y)
        zz (* z z)
        ww (* w w)
        xy2 (* x y 2)
        xz2 (* x z 2)
        xw2 (* x w 2)
        yz2 (* y z 2)
        yw2 (* y w 2)
        zw2 (* z w 2)]
    [[(+ xx (- yy) (- zz) ww) (- xy2 zw2) (+ xz2 yw2)]
     [(+ xy2 zw2) (+ (- xx) yy (- zz) ww) (- yz2 xw2)]
     [(- xz2 yw2) (+ yz2 xw2) (+ (- xx) (- yy) zz ww)]]))

(defn rotation->quaternion
  "Creates a quaternion from a rotation matrix."
  [[[m00 m01 m02]
    [m10 m11 m12]
    [m20 m21 m22]]]
  (let [trace (+ m00 m11 m22)]
    (cond
     ;; case 1
     (>= trace 0)
     (let [s (* 2 (Math/sqrt (inc trace)))]
       {:x (/ (- m21 m12) s)
        :y (/ (- m02 m20) s)
        :z (/ (- m10 m01) s)
        :w (* 0.25 s)})
     ;; case 2
     (and (> m00 m11) (> m00 m22))
     (let [s (* 2 (Math/sqrt (+ 1 m00 (- m11) (- m22))))]
       {:x (* 0.25 s)
        :y (/ (+ m01 m10) s)
        :z (/ (+ m02 m20) s)
        :w (/ (- m21 m12) s)})
     ;; case 3
     (> m11 m22)
     (let [s (* 2 (Math/sqrt (+ 1 m11 (- m00) (- m22))))]
       {:x (/ (+ m01 m10) s)
        :y (* 0.25 s)
        :z (/ (+ m12 m21) s)
        :w (/ (- m02 m20) s)})
     ;; default: case 4
     :else
     (let [s (* 2 (Math/sqrt (+ 1 m22 (- m00) (- m11))))]
       {:x (/ (+ m02 m20) s)
        :y (/ (+ m12 m21) s)
        :z (* 0.25 s)
        :w (/ (- m10 m01) s)}))))

;;;
;;; CREATING AND ACCESSING HOMOGENEOUS TRANSFORMS FROM ROTATION MATRIX
;;; AND TRANSLATION VECTOR
;;;

(defn homogeneous-transform
  "Creates a homogeneous transform from 3D rotation matrix and 3D translation
  vector."
  [& {:keys [rotation translation]
      :or {rotation (identity-matrix 3)
           translation (broadcast 0.0 [3])}}]
  (->
   (identity-matrix 4)
   (set-selection (range 3) (range 3) rotation)
   (set-selection (range 3) [3] (transpose-vector translation))))

(defn dissect-homogeneous-transform
  "Returns the rotation matrix and translation vector of a homogeneous transform."
  [transform]
  {:rotation (select transform (range 3) (range 3))
   :translation (transpose-vector (select transform (range 3) [3]))})
