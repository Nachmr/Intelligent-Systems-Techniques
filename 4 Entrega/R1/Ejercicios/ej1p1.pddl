
(define (problem RDistribuye-1)
(:domain RobotDistribuidor)
(:objects
	r1 - robot
	p1 - paquete
	p2 - paquete
	hab0 - habitacion
	hab1 - habitacion
	hab2 - habitacion

	)
(:init
	(at r1 hab0)
	(at p1 hab0)
	(at p2 hab2)
	(vacio r1)
	(conectada hab0 hab1)
	(conectada hab1 hab0)
	(conectada hab2 hab1)
	(conectada hab1 hab2)
)
(:goal (and
	(at r1 hab1)
	(at p2 hab0)
	(at p1 hab2)
	))
)