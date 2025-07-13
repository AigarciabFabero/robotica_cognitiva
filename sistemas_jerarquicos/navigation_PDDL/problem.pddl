;; Problem: Robot navigation through multiple waypoints
(define (problem waypoint-navigation-problem)
  (:domain waypoint-navigation)
  
  (:objects 
    robot1 - robot
    wp1 wp2 wp3 wp4 wp5 - waypoint
  )
  
  (:init 
    ;; Robot initial position
    (robot-at robot1 wp1)
    
    ;; Initial visited waypoint
    (visited wp1)
    
    ;; Waypoint connections (bidirectional)
    (connected wp1 wp2)
    (connected wp2 wp1)
    (connected wp2 wp3)
    (connected wp3 wp2)
    (connected wp3 wp4)
    (connected wp4 wp3)
    (connected wp4 wp5)
    (connected wp5 wp4)
    (connected wp1 wp3)
    (connected wp3 wp1)
    (connected wp2 wp5)
    (connected wp5 wp2)
  )
  
  (:goal 
    (and 
      (robot-at robot1 wp5)
      (visited wp2)
      (visited wp3)
      (visited wp4)
    )
  )
)