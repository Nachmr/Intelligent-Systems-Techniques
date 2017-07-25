(:durative-action board
 :parameters (?p - person ?a - aircraft ?c - city)
 :duration (= ?duration (boarding-time))
 :condition (and  (at ?p ?c)
                  (at ?a ?c))
 :effect (and  (not (at ?p ?c))
               (in ?p ?a)))

(:durative-action debark
 :parameters (?p - person ?a - aircraft ?c - city)
 :duration (= ?duration (debarking-time))
 :condition (and (in ?p ?a)
                 (at ?a ?c))
 :effect (and  (not (in ?p ?a))
               (at ?p ?c)))
(:task fly
 :parameters (?a - aircraft ?c1 - city ?c2 -city)
 (:method unico
  :precondition (bind ?cost (+ (total-fuel-used) (* (distance ?c1 ?c2) (slow-burn ?a))))
  :tasks  (
            (flyp ?a ?c1 ?c2 ?cost)
            )
  )
  )
 
 (:durative-action flyp
 
 :parameters (?a - aircraft ?c1 ?c2 - city ?cost - Number)
 :meta (
      (:tag prettyprint "FLY; ?a; ?c1; ?c2; ?cost")
      )
 :duration (= ?duration (/ (distance ?c1 ?c2) (slow-speed ?a)))
 :condition (and  (at ?a ?c1)
          
                 
                  (>= (fuel ?a) 
                         (* (distance ?c1 ?c2) (slow-burn ?a))))
 :effect (and  (not (at ?a ?c1))
               (at ?a ?c2)
              (increase (total-fuel-used)
                         (* (distance ?c1 ?c2) (slow-burn ?a)))
              (decrease (fuel ?a) 
                         (* (distance ?c1 ?c2) (slow-burn ?a)))))
                                  
(:task zoom
 :parameters (?a - aircraft ?c1 - city ?c2 -city)
 (:method unico
  :precondition (bind ?cost (+ (total-fuel-used) (* (distance ?c1 ?c2) (fast-burn ?a))))
  :tasks  (
            (zoomp ?a ?c1 ?c2 ?cost)
            )
  )
  )

                  
                  
(:durative-action zoomp
 :parameters (?a - aircraft ?c1 ?c2 - city ?cost - Number)
:meta (
      (:tag prettyprint "ZOOM; ?a; ?c1; ?c2; ?cost")
      )
 
 :duration (= ?duration (/ (distance ?c1 ?c2) (fast-speed ?a)))
 :condition (and  (at ?a ?c1)
                  (>= (fuel ?a) 
                         (* (distance ?c1 ?c2) (fast-burn ?a))))
 :effect (and (not (at ?a ?c1))
               (at ?a ?c2)
               (increase (total-fuel-used)
                         (* (distance ?c1 ?c2) (fast-burn ?a)))
              (decrease (fuel ?a) 
                         (* (distance ?c1 ?c2) (fast-burn ?a)))))

(:durative-action refuel
 :parameters (?a - aircraft ?c - city)
 :duration (= ?duration (/ (- (capacity ?a) (fuel ?a)) (refuel-rate ?a)))
 :condition (and  (> (capacity ?a) (fuel ?a))
                 (at ?a ?c))
 :effect (assign (fuel ?a) (capacity ?a)))
