
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Dominio ejercicio 4
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(define (domain RobotDistribuidor)
	(:requirements :strips :typing)
	(:types objeto - object
		algo - objeto
		paquete robot - algo
		habitacion fuente
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

	;;función para representar el agotamiento de la batería
	(:functions
		(descarga ?r - robot)
	)
	
	(:action cargar
		:parameters (?r - robot)
		:precondition (< (descarga ?r ) 5)
		:effect (assign (descarga ?r ) 50)
	)
	
	;; Acción de mover el robot a velocidad normal
	(:action mover
	
		:parameters (?r - robot ?from ?to - habitacion)
		
		:precondition (and
			(at ?r ?from)
			(conectada ?from ?to)
			(>= (descarga ?r) 5) ;;si la carga es mayor o igual a 5
		)
		
		
		:effect (and
			(at ?r ?to)
			(not (at ?r ?from))
			(decrease (descarga ?r) 5) ;;se decrementa en 5 el nivel de batería
		)
	)
	
	;; Acción de mover el robot a alta velocidad
	(:action mover_rapido
		:parameters (?r - robot ?from ?to - habitacion)
		:precondition (and
			(at ?r ?from)
			(conectada ?from ?to)
			(>= (descarga ?r) 10) ;;si la carga es mayor o igual a 10...
		)

		:effect (and
			(at ?r ?to)
			(not (at ?r ?from))
			(decrease (descarga ?r) 10) ;;se decrementa en 10 el nivel de batería
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





