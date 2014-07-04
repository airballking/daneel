(ns daneel.transforms-test
  (:require [clojure.test :refer :all]
            [daneel.transforms :refer :all]))

(deftest quaternion-conversions
  (testing "Single conversions for identity quaternions and rotation matrices."
    (is (= {:x 0.0 :y 0.0 :z 0.0 :w 1.0}
           (quaternion-from-rotation (clojure.core.matrix/identity-matrix 3))))
    (is (clojure.core.matrix/equals
         (clojure.core.matrix/identity-matrix 3)
         (rotation-from-quaternion {:x 0 :y 0 :z 0 :w 1}))))
  (testing "Back and forth conversions between quaternions and rotation matrices"
    (let [quaternion-roundtrip
          (comp quaternion-from-rotation rotation-from-quaternion)]
      (are [q] (= q (quaternion-roundtrip q))
           {:x 0.0 :y 0.0 :z 0.0 :w 1.0}
           {:x 0.0 :y 0.0 :z 1.0 :w 0.0}
           {:x 0.0 :y 1.0 :z 0.0 :w 0.0}
           {:x 1.0 :y 0.0 :z 0.0 :w 0.0}
           {:x (Math/sin (Math/toRadians 22.5)) :y 0.0 :z 0.0 :w (Math/cos (Math/toRadians 22.5))}))))
