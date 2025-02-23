(define (domain blocks-world)
    (:requirements :strips :typing)

    (:types block)

    (:predicates
        (on ?b1 - block ?b2 - block)  ;; Block ?b1 is on ?b2
        (on-table ?b - block)  ;; Block ?b is on the table
        (clear ?b - block)  ;; Nothing is on top of ?b
        (holding ?b - block)  ;; The robot is holding ?b
        (hand-empty)  ;; The robot's hand is empty
    )

    ;; Action: Pick up a block from the table or another block
    (:action pick-up
        :parameters (?b - block)
        :precondition (and (clear ?b) (hand-empty))
        :effect (and (holding ?b) 
                     (not (clear ?b))
                     (not (hand-empty)) 
                     (not (on-table ?b)))
    )

    ;; Action: Place a block on another block
    (:action place-on
        :parameters (?b1 - block ?b2 - block)
        :precondition (and (holding ?b1) (clear ?b2))
        :effect (and (not (holding ?b1))
                     (hand-empty)
                     (not (clear ?b2))
                     (clear ?b1)
                     (on ?b1 ?b2))
    )

    ;; Action: Place a block on the table
    (:action put-down
        :parameters (?b - block)
        :precondition (holding ?b)
        :effect (and (not (holding ?b))
                     (clear ?b)
                     (hand-empty)
                     (on-table ?b))
    )
)