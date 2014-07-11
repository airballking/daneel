(ns daneel.joint-kinematics-test
  (:require [clojure.test :refer :all]
            [daneel.joint-kinematics :refer :all]
            [daneel.transforms :refer :all]
            [clojure.core.matrix :refer :all]))

(deftest single-joint-fk-solver-test
  (testing "FK solver for revolute joints"
    (are [joint joint-state transform]
      (equals transform ((forward-kinematics-solver joint) joint-state))
      ;; revolute joint without origin offset
      ;; joint description..
      {:type :revolute :axis [1 0 0] :origin (identity-matrix 4)}
      ;; joint state..
      Math/PI
      ;; joint transform..
      (homogeneous-transform :rotation (axis-angle->rotation [1 0 0] Math/PI))
      
      ;; revolute joint with origin offset
      ;; joint description
      {:type :revolute :axis [0 1 0]
       :origin (homogeneous-transform
                :rotation (axis-angle->rotation [0 0 1] Math/PI)
                :translation [0.1 0.2 0.3])}
      ;; joint state
      (/ Math/PI 4)
      ;; joint transform
      (mmul
       (homogeneous-transform :rotation (axis-angle->rotation [0 0 1] Math/PI)
                              :translation [0.1 0.2 0.3])
       (homogeneous-transform :rotation (axis-angle->rotation [0 1 0] (/ Math/PI 4))))))
  
  (testing "FK solver for prismatic joints."
    (are [joint joint-state transform]
      (equals transform ((forward-kinematics-solver joint) joint-state))
      ;; prismatic joint without offset
      ;; joint description..
      {:type :prismatic :axis [0 0 1] :origin (identity-matrix 4)}
      ;; joint state..
      0.05
      ;; joint transform..
      (homogeneous-transform :translation [0 0 0.05])

      ;; prismatic joint with offset
      ;; joint description..
      {:type :prismatic :axis [0 1 0]
       :origin (homogeneous-transform
                :rotation (axis-angle->rotation [0 0 1] Math/PI)
                :translation [0.1 0.2 0.3])}
      ;; joint state..
      0.05
      ;; joint transform..
      (mmul
       (homogeneous-transform
                :rotation (axis-angle->rotation [0 0 1] Math/PI)
                :translation [0.1 0.2 0.3])
       (homogeneous-transform :translation [0 0.05 0]))))
  
  (testing "FK solver for fixed joints."
    (are [joint joint-state transform]
      (equals transform ((forward-kinematics-solver joint) joint-state))
      ;; fixed joint without offset
      ;; joint description..
      {:type :fixed :origin (identity-matrix 4)}
      ;; joint state..
      0.05 ; dont'care value
      ;; joint transform..
      (identity-matrix 4)
      
      ;; fixed joint with offset
      ;; joint description..
      {:type :fixed
       :origin (homogeneous-transform
                :rotation (axis-angle->rotation [0 0 1] Math/PI)
                :translation [0.1 0.2 0.3])}
      ;; joint state..
      42 ; dont'care value
      ;; joint transform..
      (homogeneous-transform
                :rotation (axis-angle->rotation [0 0 1] Math/PI)
                :translation [0.1 0.2 0.3]))))
