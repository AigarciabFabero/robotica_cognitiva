(define (problem transport_scenario)
  (:domain robot_problem)
  
  (:objects
    laboratory office corridor - location
    package - object
  )
  
  (:init
    (robot_at laboratory)
    (object_at package laboratory)
    (connected laboratory corridor)
    (connected corridor laboratory)
    (connected corridor office)
    (connected office corridor)
    (hand_empty)
    (charging_station corridor)
    (battery_full)
  )
  
  (:goal
    (and 
      (object_at package office)
    )
  )
)