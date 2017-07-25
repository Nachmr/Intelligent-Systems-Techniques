(define (domain RobotDistribuidor)

  (:requirements :strips :typing)
  (:types objetofisico - object
					cosas - objetofisico
					paquete robot - cosas
					habitacion)

  (:predicates
		;; Verdadero si ?r está en ?h
    (at ?r - objetofisico ?h - habitacion)

		;; Verdadero si ambas son habitaciones y están conectadas
		(conectada ?h1 - habitacion ?h2 - habitacion)

		;; El robot va vacío
    (free ?r - robot)

		;; El robot lleva al paquete
    (carry ?p - paquete ?r - robot)
  )

  ;; El robot se puede mover de una habitación a otra
  (:action move
    :parameters (?r - robot ?from ?to - habitacion)
    :precondition (and
										(at ?r ?from)
										(conectada ?from ?to))
    :effect (and
							(at ?r ?to)
              (not (at ?r ?from)))
  )

  ;; El robot puede coger ?obj en la habitación ?h
  (:action pick
    :parameters (?obj - paquete ?h - habitacion ?r - robot)
    :precondition (and
										(at ?obj ?h)
										(at ?r ?h)
										(free ?r))
    :effect (and
							(carry ?obj ?r )
							(not (at ?obj ?h))
            	(not (free ?r)))
	)

  ;; El robot puede soltar ?obj en la habitación ?h
  (:action drop
    :parameters (?obj - paquete ?h - habitacion ?r - robot)
    :precondition (and
										(carry ?obj ?r)
										(at ?r ?h))
    :effect (and
							(at ?obj ?h)
							(free ?r)
    					(not (carry ?obj ?r)))
  )
)
