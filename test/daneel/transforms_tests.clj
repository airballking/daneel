(ns daneel.transforms-test
  (:require [clojure.test :refer :all]
            [daneel.transforms :refer :all]))

(deftest axis-angle-rotations
  (testing "Creating rotation matrices from axis/angle representation."
    ;; some aux functions to create rotation matrices for rotations
    ;; around x-axis, y-axis, or z-axis, respectively
    (letfn [(rot-x
              [angle]
              (let [c (Math/cos angle)
                    s (Math/sin angle)]
                [[1 0 0]
                 [0 c (- s)]
                 [0 s c]]))
            (rot-y
              [angle]
              (let [c (Math/cos angle)
                    s (Math/sin angle)]
                [[c 0 s]
                 [0 1 0]
                 [(- s) 0 c]]))
            (rot-z
              [angle]
              (let [c (Math/cos angle)
                    s (Math/sin angle)]
                [[c (- s) 0]
                 [s c 0]
                 [0 0 1]]))]
      ;; actual test cases following here
      (are [matrix axis angle] (clojure.core.matrix/equals matrix (axis-angle->rotation axis angle))
           ;; case 1: no rotation
           (clojure.core.matrix/identity-matrix 3) [0 0 1] 0
           ;; case 2: rotation around x
           (rot-x (/ Math/PI 4)) [1 0 0] (/ Math/PI 4)
           ;; case 3: rotation around y
           (rot-y (/ Math/PI 4)) [0 1 0] (/ Math/PI 4)
           ;; case 4: rotation around z
           (rot-z (/ Math/PI 4)) [0 0 1] (/ Math/PI 4)))))

(deftest quaternion-conversions
  (testing "Single conversions for identity quaternions and rotation matrices."
    (is (= {:x 0.0 :y 0.0 :z 0.0 :w 1.0}
           (rotation->quaternion (clojure.core.matrix/identity-matrix 3))))
    (is (clojure.core.matrix/equals
         (clojure.core.matrix/identity-matrix 3)
         (quaternion->rotation {:x 0 :y 0 :z 0 :w 1}))))
  (testing "Back and forth conversions between quaternions and rotation matrices"
    (let [quaternion-roundtrip
          (comp rotation->quaternion quaternion->rotation)]
      (are [q] (= q (quaternion-roundtrip q))
           {:x 0.0 :y 0.0 :z 0.0 :w 1.0}
           {:x 0.0 :y 0.0 :z 1.0 :w 0.0}
           {:x 0.0 :y 1.0 :z 0.0 :w 0.0}
           {:x 1.0 :y 0.0 :z 0.0 :w 0.0}
           {:x (Math/sin (Math/toRadians 22.5)) :y 0.0
            :z 0.0 :w (Math/cos (Math/toRadians 22.5))}))))

(deftest homogeneous-transform-basics
  (testing "Creating and accessing homogeneous transforms."
    (let [hom-transform-roundtrip
          (comp (partial apply homogeneous-transform)
                (partial apply concat)
                dissect-homogeneous-transform)]
      (are [T] (clojure.core.matrix/equals T (hom-transform-roundtrip T))
           (clojure.core.matrix/identity-matrix 4)
           [[1 2 3 4] [5 6 7 8] [9 10 11 12] [0 0 0 1]]))))
