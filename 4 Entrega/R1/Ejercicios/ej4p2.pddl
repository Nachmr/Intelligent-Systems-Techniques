(define (problem RDistribuye-1)
(:domain RobotDistribuidor)
(:objects
	r1 r2 r3 - robot
	p1 p2 p3 p4 p5 p6 p7 p8 p9 p10 - paquete
	hab0 hab1 hab2 hab3 hab4 - habitacion
	)
(:init
	(at r1 hab0)
	(at r2 hab0)
	(at r3 hab4)
	(at p1 hab0)
	(at p2 hab0)
	(at p3 hab0)
	(at p4 hab0)
	(at p5 hab0)
	(at p6 hab2)
	(at p7 hab2)
	(at p8 hab2)
	(at p9 hab4)
	(at p10 hab2)

	(vacio r1)
	(vacio r2)
	(vacio r3)

	(= (descarga r1 ) 50)
	(= (descarga r2 ) 50)
	(= (descarga r3 ) 50)

	(conectada hab0 hab1)
	(conectada hab1 hab0)
	(conectada hab2 hab1)
	(conectada hab1 hab2)
	(conectada hab1 hab3)
	(conectada hab3 hab1)
	(conectada hab3 hab4)
	(conectada hab4 hab3)
)
(:goal (and
	(at p6 hab0)
	(at p7 hab0)
	(at p8 hab0)
	(at p9 hab0)
	(at p10 hab0)
	(at p1 hab2)
	(at p2 hab2)
	(at p3 hab2)
	(at p4 hab2)
	(at p5 hab2)

	(at r1 hab2)
	(at r2 hab2)
	(at r3 hab4)
	))

)
