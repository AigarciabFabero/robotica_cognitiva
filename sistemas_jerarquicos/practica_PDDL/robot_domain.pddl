(define (domain robot_problem)
  (:requirements :strips :typing)
  (:types location object)

  (:predicates
    (robot_at ?l - location)
    (connected ?from ?to - location)
    (object_at ?o - object ?l - location)
    (holding ?o - object)
    (hand_empty)
    (charging_station ?l - location)
    (battery_full)
  )

  (:action move
    :parameters (?from ?to - location)
    :precondition (and (robot_at ?from) (connected ?from ?to) (battery_full))
    :effect (and (not (robot_at ?from)) (robot_at ?to) (not (battery_full)))
  )

  (:action pick
    :parameters (?o - object ?l - location)
    :precondition (and (robot_at ?l) (object_at ?o ?l) (hand_empty))
    :effect (and (not (object_at ?o ?l)) (not (hand_empty)) (holding ?o))
  )

  (:action place
    :parameters (?o - object ?l - location)
    :precondition (and (robot_at ?l) (holding ?o))
    :effect (and (object_at ?o ?l) (hand_empty) (not (holding ?o)))
  )

  (:action recharge
    :parameters (?l - location)
    :precondition (and (robot_at ?l) (charging_station ?l))
    :effect (battery_full)
  )
)