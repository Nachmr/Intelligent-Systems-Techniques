(define (problem zeno-0) (:domain zeno-travel)
(:customization
(= :time-format "%d/%m/%Y %H:%M:%S")
(= :time-horizon-relative 2500)
(= :time-start "05/06/2007 08:00:00")
(= :time-unit :hours))

(:objects 
    p1 p2 p3 - person
    Almeria Barcelona Cadiz Cordoba Gibraltar Granada Huelva Jaen Madrid Malaga Sevilla - city
    a1 a2 - aircraft
)
(:init
    (at p1 Almería) 
    (at p2 Barcelona)
    (at p3 Malaga)
    (at a1 Granada)
    (at a2 Madrid)


    (destino p1 Bilbao)
    (destino p2 Cordoba)
    (destino p3 Gibraltar)

    (= (fuel-limit) 1500)

    (= (distance Almeria Barcelona) 809)
    (= (distance Almeria Bilbao) 958)
    (= (distance Almeria Cadiz)  463 )
    (= (distance Almeria Cordoba) 316 )
    (= (distance Almeria Gibraltar) 339)
    (= (distance Almeria Granada) 162)
    (= (distance Almeria Huelva) 505)
    (= (distance Almeria Jaen) 220 )
    (= (distance Almeria Madrid) 547 )
    (= (distance Almeria Malaga) 207 )
    (= (distance Almeria Sevilla) 410 )

    (= (distance Bilbao Barcelona) 620)
    (= (distance Barcelona Cadiz)  1284 )
    (= (distance Barcelona Cordoba) 908 )
    (= (distance Barcelona Gibraltar) 1124)
    (= (distance Barcelona Granada) 868)
    (= (distance Barcelona Huelva) 1140)
    (= (distance Barcelona Jaen) 804 )
    (= (distance Barcelona Madrid) 621 )
    (= (distance Barcelona Malaga) 997 )
    (= (distance Barcelona Sevilla) 410 )

    (= (distance Bilbao Cadiz)  1058 )
    (= (distance Bilbao Cordoba) 796 )
    (= (distance Bilbao Gibraltar) 1110)
    (= (distance Bilbao Granada) 829)
    (= (distance Bilbao Huelva) 939)
    (= (distance Bilbao Jaen) 730 )
    (= (distance Bilbao Madrid) 395 )
    (= (distance Bilbao Malaga) 939 )
    (= (distance Bilbao Sevilla) 933 )

    (= (distance Cadiz Cordoba) 261 )
    (= (distance Cadiz Gibraltar) 124)
    (= (distance Cadiz Granada) 269)
    (= (distance Cadiz Huelva) 214)
    (= (distance Cadiz Jaen) 330 )
    (= (distance Cadiz Madrid) 654 )
    (= (distance Cadiz Malaga) 240 )
    (= (distance Cadiz Sevilla) 126 )

    (= (distance Cordoba Gibraltar) 294)
    (= (distance Cordoba Granada) 160)
    (= (distance Cordoba Huelva) 241)
    (= (distance Cordoba Jaen) 108 )
    (= (distance Cordoba Madrid) 396 )
    (= (distance Cordoba Malaga) 165 )
    (= (distance Cordoba Sevilla) 143 )

    (= (distance Gibraltar Granada) 255)
    (= (distance Gibraltar Huelva) 289)
    (= (distance Gibraltar Jaen) 335 )
    (= (distance Gibraltar Madrid) 662 )
    (= (distance Gibraltar Malaga) 134 )
    (= (distance Gibraltar Sevilla) 201 )

    (= (distance Granada Huelva) 346)
    (= (distance Granada Jaen) 93 )
    (= (distance Granada Madrid) 421 )
    (= (distance Granada Malaga) 125 )
    (= (distance Granada Sevilla) 252 )

    (= (distance Huelva Jaen) 347 )
    (= (distance Huelva Madrid) 591 )
    (= (distance Huelva Malaga) 301 )
    (= (distance Huelva Sevilla) 95 )

    (= (distance Jaen Madrid) 335 )
    (= (distance Jaen Malaga) 203 )
    (= (distance Jaen Sevilla) 246 )


    (= (distance Madrid Malaga) 532 )
    (= (distance Madrid Sevilla) 534 )

    (= (distance Malaga Sevilla) 209 )



    (= (fuel a1) 200)
    (= (slow-speed a1) 10)
    (= (fast-speed a1) 20)
    (= (slow-burn a1) 1)
    (= (fast-burn a1) 2)
    (= (capacity a1) 300)
    (= (refuel-rate a1) 1)

    (= (fuel a2) 200)
    (= (slow-speed a2) 10)
    (= (fast-speed a2) 20)
    (= (slow-burn a2) 1)
    (= (fast-burn a2) 2)
    (= (capacity a2) 300)
    (= (refuel-rate a2) 1)

    (= (total-fuel-used) 0)
    (= (boarding-time) 10)
    (= (debarking-time) 5)
 )


(:tasks-goal
   :tasks(
   (transport-person p1 c5)
   (transport-person p2 c5)
   (transport-person p3 c5)

    (destino p1 c5)
    (destino p2 c5)
    (destino p3 c5)

   
   )
  )
)