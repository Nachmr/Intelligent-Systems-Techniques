
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Dominio ejercicio 3
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
		
		;; Carga de la batería y nivel
		(cambio ?n1 ?n2 - fuente)
		(nivelbateria ?r - robot ?f - fuente)
		
		;;Si la velocidad del robot es rapida
		(cambio-rapido ?n1 ?n2 - fuente)
	)

	;; Acción de mover el robot a velocidad normal
	(:action mover
	
		:parameters (?r - robot ?from ?to - habitacion ?bateriaantes ?bateriadespues - fuente)
		
		:precondition (and
			(at ?r ?from)
			(conectada ?from ?to)
			(nivelbateria ?r ?bateriaantes)
			(cambio ?bateriaantes ?bateriadespues)
		)
		
		
		:effect (and
			(at ?r ?to)
			(not (at ?r ?from))
			(not (nivelbateria ?r ?bateriadespues) )
			(nivelbateria ?r ?bateriaantes)
		)
	)
	
	
	;; Acción de mover el robot a alta velocidad
	(:action mover_rapido
		:parameters (?r - robot ?from ?to - habitacion ?bateriaantes ?bateriadespues - fuente)
		:precondition (and
			(at ?r ?from)
			(conectada ?from ?to)
			(nivelbateria ?r ?bateriaantes)
			(cambio-rapido ?bateriaantes ?bateriadespues) ;;el cambio de bateria es mas rapido que si se mueve normal
		)

		:effect (and
			(at ?r ?to)
			(not (at ?r ?from))
			(not (nivelbateria ?r ?bateriaantes) )
			(nivelbateria ?r ?bateriadespues)
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




