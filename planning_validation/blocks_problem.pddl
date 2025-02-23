(define (problem simple-blocks)
    (:domain blocks-world)

    (:objects
        A B - block
    )

    (:init
        (on-table A) (on-table B)
        (clear A) (clear B)
        (hand-empty)
    )

    (:goal
        (and (on A B))
    )
)
