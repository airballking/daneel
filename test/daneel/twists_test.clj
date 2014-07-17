(ns daneel.twists-test
  (:refer-clojure :exclude [* - + == /])
  (:require [clojure.test :refer :all]
            [daneel.utils :refer :all]
            [clojure.core.matrix :refer :all]
            [clojure.core.matrix.operators :refer :all]
            [daneel.twists :refer :all]))

(deftest twist-basics
  (testing "Creating twists from translation velocities only."
    (are [translation t] (equals t (twist :translation translation))
         [0.0 0.0 0.0] [0.0 0.0 0.0 0.0 0.0 0.0]
         [1.0 1.0 1.0] [0.0 0.0 0.0 1.0 1.0 1.0]))
  (testing "Creating twists from angular velocities, only."
    (are [angular t] (equals t (twist :angular angular))
         [0.0 0.0 0.0] [0.0 0.0 0.0 0.0 0.0 0.0]
         [1.0 1.0 1.0] [1.0 1.0 1.0 0.0 0.0 0.0]))
  (testing "Creating twists from both angular and translation velocities."
    (are [angular translation t]
      (equals t (twist :angular angular :translation translation))
      [0.0 0.0 0.0] [0.0 0.0 0.0] [0.0 0.0 0.0 0.0 0.0 0.0]
      [1.0 1.0 1.0] [0.0 0.0 0.0] [1.0 1.0 1.0 0.0 0.0 0.0]
      [0.0 0.0 0.0] [1.0 1.0 1.0] [0.0 0.0 0.0 1.0 1.0 1.0]))
  (testing "Dissecting twists into both angular and translation velocities."
    (are [t ang-vel trans-vel]
      (let [{:keys [angular translation]} (dissect-twist t)]
        (and (equals angular ang-vel)
             (equals translation trans-vel)))
      [0.0 0.0 0.0 0.0 0.0 0.0] [0.0 0.0 0.0] [0.0 0.0 0.0] 
      [1.0 1.0 1.0 0.0 0.0 0.0] [1.0 1.0 1.0] [0.0 0.0 0.0] 
      [0.0 0.0 0.0 1.0 1.0 1.0] [0.0 0.0 0.0] [1.0 1.0 1.0])))

(deftest pluecker-transform-creation
  (testing "Creation of pluecker transforms from rotation and translation"
    (are [rotation translation pluecker]
      (equals (pluecker-transform rotation translation) pluecker)
      ;; case 1: all nothing
      (zero-matrix 3 3) [0.0 0.0 0.0] (zero-matrix 6 6)
      ;; case 2: only identity rotation
      (identity-matrix 3) [0.0 0.0 0.0] (identity-matrix 6)
      ;; case 3: only skew matrix
      (zero-matrix 3 3) [1 1 1] (zero-matrix 6 6)
      ;; case 4: identity rotation and trivial skew matrix
      (identity-matrix 3) [1 1 1]
      [[1 0 0 0 0 0]
       [0 1 0 0 0 0]
       [0 0 1 0 0 0]
       [0 -1 1 1 0 0]
       [1 0 -1 0 1 0]
       [-1 1 0 0 0 1]]))
  (testing "Creating pluecker transform from homogeneous transforms"
    ;; TODO(Georg): introduce global constant for epsilon
    (are [hom-transform pluecker] (equals pluecker (pluecker-transform hom-transform) 1E-16)
         ;; CASE 1: identity transforms
         (identity-matrix 4) (identity-matrix 6)
         ;; CASE 2: sanity check with all zeros
         (zero-matrix 4 4) (zero-matrix 6 6)
         ;; CASE 3: some translation
         (daneel.transforms/homogeneous-transform :translation [1 1 1])
         [[1 0 0 0 0 0]
          [0 1 0 0 0 0]
          [0 0 1 0 0 0]
          [0 -1 1 1 0 0]
          [1 0 -1 0 1 0]
          [-1 1 0 0 0 1]]
         ;; CASE 4: some rotation
         (daneel.transforms/homogeneous-transform :rotation (daneel.transforms/axis-angle->rotation [1 0 0] (/ Math/PI 2)))
         [[1 0 0 0 0 0]
          [0 0 -1 0 0 0]
          [0 1 0 0 0 0]
          [0 0 0 1 0 0]
          [0 0 0 0 0 -1]
          [0 0 0 0 1 0]]
         )))
