(define (problem RDistribuye-1)
(:domain RobotDistribuidor)
(:objects
	r1 r2 - robot
	p1 p2 p3 p4 p5 p6 p7 - paquete
	hab0 hab1 hab2 hab3 hab4 - habitacion
	)
(:init
	(at r1 hab0)
	(at r2 hab0)
	(at p1 hab0)
	(at p2 hab0)
	(at p3 hab0)

	(vacio r1)
	(vacio r2)

	
	(= (descarga r1 ) 50)
	(= (descarga r2 ) 50)


	
	(conectada hab0 hab1)
	(conectada hab1 hab0)
	(conectada hab2 hab1)
	(conectada hab1 hab2)
	(conectada hab2 hab3)
	(conectada hab3 hab2)
	(conectada hab3 hab4)
	(conectada hab4 hab3)
)
(:goal (and
	(at p1 hab2)
	(at p2 hab3)
	(at p3 hab4)
	
	(at r1 hab1)
	(at r2 hab2)
	))

)
