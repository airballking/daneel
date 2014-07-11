(ns daneel.utils-test
  (:require [clojure.test :refer :all]
            [daneel.utils :refer :all]
            [clojure.core.matrix]))

(deftest vector-predicates
  (testing "Predicate to check whether a vector is a row vector."
    (are [v res] (= (row-vector? v) res)
         [1 0 0] true
         [1] true
         [] false
         0 false
         [[]] false
         [[1]] false
         [[1] [2 [3]]] false))
  (testing "Predicate to check whether a vector is a column vector."
    (are [v res] (= (column-vector? v) res)
         [1 0 0] false
         [1] false
         [] false
         0 false
         [[]] false
         [[1]] true
         [[1] [2 [3]]] true)))

(deftest vector-transposing
  (testing "Function to transpose row and column vectors."
    (are [v1 v2] (clojure.core.matrix/equals v1 (transpose-vector v2))
         [1 0 0] [[1] [0] [0]]
         [[1] [2]] [1 2]
         [0] [[0]])))
