;; Domain for robot navigation between waypoints
(define (domain waypoint-navigation)
  (:requirements :strips :typing)
  
  (:types 
    robot waypoint - object
  )
  
  (:predicates 
    (robot-at ?r - robot ?w - waypoint)
    (connected ?w1 - waypoint ?w2 - waypoint)
    (visited ?w - waypoint)
  )
  
  (:action move
    :parameters (?r - robot ?from - waypoint ?to - waypoint)
    :precondition (and 
      (robot-at ?r ?from)
      (connected ?from ?to)
    )
    :effect (and 
      (not (robot-at ?r ?from))
      (robot-at ?r ?to)
      (visited ?to)
    )
  )
)