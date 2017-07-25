
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Dominio ejercicio 1
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(define (domain RobotDistribuidor)
	(:requirements :strips :typing)
	(:types objeto - object
		algo - objeto
		paquete robot - algo
		habitacion
	)

	(:predicates
	
		;; TRUE si ?r está en ?h
		(at ?r - objeto ?h - habitacion)

		;; TRUE si son habitaciones conectadas
		(conectada ?h1 - habitacion ?h2 - habitacion)

		;; El robot esta vacio
		(vacio ?r - robot)

		;; El robot lleva el paquete
		(llevando ?p - paquete ?r - robot)
	)

	;; El robot se mueve de una habitación a otra
	(:action mover
	
		:parameters (?r - robot ?from ?to - habitacion)
		
		:precondition (and
			(at ?r ?from)
			(conectada ?from ?to)
		)
		
		:effect (and
			(at ?r ?to)
			(not (at ?r ?from))
		)
	)

	;;Acción de soltar obj en h, el robot puede soltar ?obj en la habitación ?h
	(:action soltar
	
		:parameters (?obj - paquete ?h - habitacion ?r - robot)
		
		:precondition (and
			(llevando ?obj ?r)
			(at ?r ?h)
		)
		
		:effect (and
			(at ?obj ?h)
			(vacio ?r)
			(not (llevando ?obj ?r))
		)
	)
	
	;;Acción de coger obj en h
	(:action coger
		:parameters (?obj - paquete ?h - habitacion ?r - robot)
		
		:precondition (and
			(at ?obj ?h)
			(at ?r ?h)
			(vacio ?r)
		)
		
		:effect (and
			(llevando ?obj ?r )
			(not (at ?obj ?h))
			(not (vacio ?r))
		)
	)

)

