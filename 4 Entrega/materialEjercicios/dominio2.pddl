(define (domain zeno-travel)


(:requirements
  :typing
  :fluents
  :derived-predicates
  :negative-preconditions
  :universal-preconditions
  :disjuntive-preconditions
  :conditional-effects
  :htn-expansion

  ; Requisitos adicionales para el manejo del tiempo
  :durative-actions
  :metatags
 )

(:types aircraft person city - object)
(:constants slow fast - object)
(:predicates (at ?x - (either person aircraft) ?c - city)
             (in ?p - person ?a - aircraft)
             (different ?x ?y) (igual ?x ?y)
             (hay-fuel-rapido ?a ?c1 ?c2)
             (hay-fuel-lento ?a ?c1 ?c2)
             (embarcar ?a ?p)
             (desembarcar ?a ?p)
             (destino ?p - person ?c1 - city)
             )
(:functions (fuel ?a - aircraft)
            (distance ?c1 - city ?c2 - city)
            (slow-speed ?a - aircraft)
            (fast-speed ?a - aircraft)
            (slow-burn ?a - aircraft)
            (fast-burn ?a - aircraft)
            (capacity ?a - aircraft)
            (refuel-rate ?a - aircraft)
            (fuel-limit)
            (total-fuel-used)
            (boarding-time)
            (debarking-time)
            )

;; el consecuente "vacío" se representa como "()" y significa "siempre verdad"
(:derived
  (igual ?x ?x) ())

(:derived 
  (different ?x ?y) (not (igual ?x ?y)))



;; este literal derivado se utiliza para deducir, a partir de la información en el estado actual, 
;; si hay fuel suficiente para que el avión ?a vuele de la ciudad ?c1 a la ?c2
;; el antecedente de este literal derivado comprueba si el fuel actual de ?a es mayor que 1. 
;; En este caso es una forma de describir que no hay restricciones de fuel. Pueden introducirse una
;; restricción más copleja  si en lugar de 1 se representa una expresión más elaborada (esto es objeto de
;; los siguientes ejercicios).
(:derived 
  
  (hay-fuel-rapido ?a - aircraft ?c1 - city ?c2 - city)
  (> (fuel ?a) (* (distance ?c1 ?c2) (fast-burn ?a)))
)

(:derived 
  
  (hay-fuel-lento ?a - aircraft ?c1 - city ?c2 - city)
  (> (fuel ?a) (* (distance ?c1 ?c2) (slow-burn ?a)))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;
(:derived 
  
   (distance ?c1 - city ?c2 - city)
   (= (distance ?c2 - city ?c1 - city) (distance ?c1 - city ?c2 - city) )
)

(:derived 
  
  (embarcar ?a - aircraft ?p - person)
  (< (capacity ?a) (+ (capacity ?a) 1))
)

(:derived 
  
  (desembarcar ?a - aircraft ?p - person)
  (< (capacity ?a) (- (capacity ?a) 1))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;

(:task transport-person
	:parameters (?p - person ?c - city)
	
  (:method Case1 ; si la persona est� en la ciudad no se hace nada
	 :precondition (at ?p ?c)
	 :tasks ()
   )
	 
   
   (:method Case2 ;si no est� en la ciudad destino, pero avion y persona est�n en la misma ciudad
	  :precondition (and (at ?p - person ?c1 - city)(at ?a - aircraft ?c1 - city))
				     
	  :tasks ( 
	  	      (board ?p ?a ?c1)
		        (mover-avion ?a ?c1 ?c)
		        (debark ?p ?a ?c )))

  (:method Case3 
    :precondition (and (at ?p - person ?c1 - city)
                       (at ?a - aircraft ?c2 - city))
             
    :tasks ( 
            (mover-avion ?a ?c2 ?c1)
            (board ?p ?a ?c1)
            (mover-avion ?a ?c1 ?c)
            (debark ?p ?a ?c )))
  


	)

(:task mover-avion
 :parameters (?a - aircraft ?c1 - city ?c2 -city)
 (:method fuel-suficiente-rapido 
  :precondition (and(hay-fuel-rapido ?a ?c1 ?c2)(>(fuel-limit)(total-fuel-used)))
  :tasks (
          (zoom ?a ?c1 ?c2)
         )
   )

  (:method no-hay-fuel-rapido ;; este método se escogerá cuando no haya fuel suficiente.
                           ;;primero repostará y después realizará el vuelo.
    :precondition (and (not(hay-fuel-rapido ?a ?c1 ?c2))(>(fuel-limit)(total-fuel-used)))
    :tasks (
          (refuel ?a ?c1)
          (zoom ?a ?c1 ?c2)
    )
  )



   (:method fuel-suficiente-lento 
  :precondition (and(hay-fuel-lento ?a ?c1 ?c2)(>(fuel-limit)(total-fuel-used)))
  :tasks (
          (fly ?a ?c1 ?c2)
         )
   )

  (:method no-hay-fuel-lento ;; este método se escogerá cuando no haya fuel suficiente.
                           ;;primero repostará y después realizará el vuelo.
    :precondition (and (not(hay-fuel-lento ?a ?c1 ?c2))(>(fuel-limit)(total-fuel-used)))
    :tasks (
          (refuel ?a ?c1)
          (fly ?a ?c1 ?c2)
    )
  )

)

(:task board
 :parameters (?p - person ?a - aircraft ?c1 - city)
 
 (:method embarque-pasajeros 
  :precondition  (and(and (at ?p - person ?c1 - city)(at ?a - aircraft ?c1 - city)(embarcar ?a - aircraft ?p - person)))
  :tasks (
          (embarcar ?p ?c)
          (board)
         )
  )

)


(:task debark
 :parameters (?p - person ?a - aircraft ?c1 - city)

  (:method desembarque-pasajeros
    :precondition (and (at ?p - person ?c1 - city)(at ?a - aircraft ?c1 - city)(desembarcar ?a - aircraft ?p - person))
    :tasks(
          (desembarcar ?p ?a)
          (debark)
    )
  )
)
  (:durative-action board
     :parameters (?p - person ?a - aircraft)
     :duration (= ?duration 2)
     :condition (and  (at ?p - person ?c1 - city)(at ?a - aircraft ?c1 - city) )
     :effect (embarcar ?a ?p)
  )

  (:durative-action debark
     :parameters (?p - person ?a - aircraft)
     :duration (= ?duration 2)
     :condition (and  (at ?p - person ?c1 - city)(at ?a - aircraft ?c1 - city) )
     :effect (desembarcar ?a ?p)
  )

(:import "Primitivas-Zenotravel.pddl") 


)
