(ns daneel.transforms
  (:refer-clojure :exclude [* - + == /])
  (:use clojure.core.matrix)
  (:use clojure.core.matrix.operators))

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

(defn rotation-from-quaternion
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

(defn quaternion-from-rotation
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
