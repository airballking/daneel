(defproject daneel "0.1.0-SNAPSHOT"
  :description "FIXME: write description"
  :url "http://example.com/FIXME"
  :license {:name "Eclipse Public License"
            :url "http://www.eclipse.org/legal/epl-v10.html"}
  :dependencies [[org.clojure/clojure "1.5.1"]
                 [net.mikera/core.matrix "0.26.0"]]
  :main ^:skip-aot daneel.core
  :target-path "target/%s"
  :profiles {:uberjar {:aot :all}})
