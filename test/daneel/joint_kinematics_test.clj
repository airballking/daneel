(ns daneel.joint-kinematics-test
  (:require [clojure.test :refer :all]
            [daneel.joint-kinematics :refer :all]
            [daneel.transforms :refer :all]
            [clojure.core.matrix :refer :all]
            [clojure.core.matrix.operators :refer :all]))

(deftest joint-predicates
  (testing "Fixed joint predicates."
    (are [joint result] (= (fixed-joint? joint) result)
         {:type :fixed} true
         {} false
         {:type :prismatic} false
         {:type :revolute} false))
  (testing "Prismatic joint predicates."
    (are [joint result] (= (prismatic-joint? joint) result)
         {:type :prismatic} true
         {} false
         {:type :fixed} false
         {:type :revolute} false))
  (testing "Revolute joint predicates."
    (are [joint result] (= (revolute-joint? joint) result)
         {:type :revolute} true
         {} false
         {:type :prismatic} false
         {:type :fixed} false)))

(deftest revolute-joint-forward-kinematics-solvers
  (testing "FK solver: revolute joint with identity origin."
    (are [axis origin joint-state transform]
      (equals transform ((revolute-joint-fk-solver axis origin) joint-state))
      ;; TEST 1
      ;; joint rotation axis..
      [1 0 0]
      ;; joint origin..
      (identity-matrix 4)
      ;; joint state..
      Math/PI
      ;; joint transform..
      (homogeneous-transform :rotation (axis-angle->rotation [1 0 0] Math/PI))
      
      ;; TEST 2
      ;; joint rotation axis..
      [0 0 1]
      ;; joint origin..
      (identity-matrix 4)
      ;; joint state..
      (- Math/PI)
      ;; joint transform..
      (homogeneous-transform :rotation (axis-angle->rotation [0 0 1] (- Math/PI)))))
  
  (testing "FK solver: revolute joint with non-identity origin."
    (are [axis origin joint-state transform]
      (equals transform ((revolute-joint-fk-solver axis origin) joint-state))
      ;; TEST 1
      ;; joint rotation axis..
      [0 1 0]
      ;; joint origin..
      (homogeneous-transform
       :rotation (axis-angle->rotation [0 0 1] Math/PI)
       :translation [0.1 0.2 0.3])
      ;; joint state..
      (/ Math/PI 4)
      ;; joint transform..
      (mmul
       (homogeneous-transform :rotation (axis-angle->rotation [0 0 1] Math/PI)
                              :translation [0.1 0.2 0.3])
       (homogeneous-transform :rotation (axis-angle->rotation [0 1 0] (/ Math/PI 4))))

      ;; TEST 2
      ;; joint rotation axis..
      [1 1 1]
      ;; joint origin..
      (homogeneous-transform
       :rotation (axis-angle->rotation [-1 0 1] (/ Math/PI 4))
       :translation [0.1 0.2 0.3])
      ;; joint state
      (/ Math/PI 16)
      ;; joint transform
      (mmul
       (homogeneous-transform :rotation (axis-angle->rotation [-1 0 1] (/ Math/PI 4))
                              :translation [0.1 0.2 0.3])
       (homogeneous-transform :rotation (axis-angle->rotation [1 1 1] (/ Math/PI 16)))))))

(deftest prismatic-joint-forward-kinematics-solver
  (testing "FK solver: prismatic joint with identity origin."
   (are [axis origin joint-state transform]
     (equals transform ((prismatic-joint-fk-solver axis origin) joint-state))
     ;; TEST CASE 1
     ;; joint axis..
     [0 0 1]
     ;; joint origin..
     (identity-matrix 4)
     ;; joint state..
     0.05
     ;; joint transform..
     (homogeneous-transform :translation [0 0 0.05])
     
     ;; TEST CASE 2
     ;; joint axis..
     [1 1 0]
     ;; joint origin..
     (identity-matrix 4)
     ;; joint state..
     (- 0.15)
     ;; joint transform..
     (homogeneous-transform :translation (* (- 0.15) (normalise [1 1 0])))))
  (testing "FK solver: prismatic joint with non-identity origin."
    (are [axis origin joint-state transform]
      (equals transform ((prismatic-joint-fk-solver axis origin) joint-state))
      ;; joint axis..
      [0 1 0]
      ;; joint origin
      (homogeneous-transform
       :rotation (axis-angle->rotation [0 0 1] Math/PI)
       :translation [0.1 0.2 0.3])
      ;; joint state..
      0.05
      ;; joint transform..
      (mmul
       (homogeneous-transform
                :rotation (axis-angle->rotation [0 0 1] Math/PI)
                :translation [0.1 0.2 0.3])
       (homogeneous-transform :translation [0 0.05 0])))))

(deftest fixed-joint-forward-kinematics-solver
  (testing "FK solver for fixed joints."
    (are [joint transform]
      (equals transform (fixed-joint-fk-solver joint))
      ;; TEST CASE 1
      ;; joint description..
      {:type :fixed :origin (identity-matrix 4)}
      ;; joint transform..
      (identity-matrix 4)
      
      ;; TEST CASE 2
      ;; joint description..
      {:type :fixed
       :origin (homogeneous-transform
                :rotation (axis-angle->rotation [0 0 1] Math/PI)
                :translation [0.1 0.2 0.3])}
      ;; joint transform..
      (homogeneous-transform
                :rotation (axis-angle->rotation [0 0 1] Math/PI)
                :translation [0.1 0.2 0.3]))))
