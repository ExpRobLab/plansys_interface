(define (domain navigation)
    (:requirements :strips :typing :adl :fluents :durative-actions)

    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:types
        robot marker base
    );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

    ;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
    (:predicates

        (robot_at ?r - robot ?m - marker)
        (robot_not_at ?r - robot ?m - marker)
        (detected ?m - marker)
        (undetected ?m - marker)
        (photo_mode)
        (detect_mode)
        (is_after ?m_prev - marker ?m_post - marker)
        (photographed ?m - marker)
        (at_base ?r - robot ?b - base)
        (is_base ?b - base ?m - marker)
        (unphotographed ?m - marker)
        (free ?r - robot)
    );; end Predicates ;;;;;;;;;;;;;;;;;;;;

    ;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:durative-action move_to_detect
        :parameters (?r - robot ?m - marker ?m0 - marker ?b - base )
        :duration ( = ?duration 5)
        :condition (and
                (at start (undetected ?m))
                (at start (is_base ?b ?m0))
                (at start (not(= ?m ?m0)))
                (at start (robot_not_at ?r ?m))
                (at start (detect_mode))
                 (at start (free ?r))
                )
        
        :effect (and
             (at start (not(free ?r)))

            (at end(not(robot_not_at ?r ?m)))
            (at end(robot_at ?r ?m))

            (at end(not(robot_at ?r ?m0)))
            (at end(robot_not_at ?r ?m0))
           
            (at end(detected ?m))
            (at end(not(undetected ?m)))
            
            (at end (free ?r))
        )
    )

    (:durative-action change_state
        :parameters (?r - robot ?m1 - marker ?m2 - marker ?m3 - marker ?m4 - marker ?m0 - marker ?b - base)
        :duration ( = ?duration 5)
        :condition (and
            (at start (not(= ?m1 ?m2)))
            (at start (not(= ?m1 ?m3)))
            (at start (not(= ?m1 ?m4)))
            (at start (not(= ?m1 ?m0)))

            (at start (not(= ?m2 ?m1)))
            (at start (not(= ?m2 ?m3)))
            (at start (not(= ?m2 ?m4)))
            (at start (not(= ?m2 ?m0)))

            (at start (not(= ?m3 ?m1)))
            (at start (not(= ?m3 ?m2)))
            (at start (not(= ?m3 ?m4)))
            (at start (not(= ?m3 ?m0)))

            (at start (not(= ?m4 ?m1)))
            (at start (not(= ?m4 ?m3)))
            (at start (not(= ?m4 ?m2)))
            (at start (not(= ?m4 ?m0)))

            (at start (detected ?m1))
            (at start (detected ?m2))
            (at start (detected ?m3))
            (at start (detected ?m4))
            (at start (detected ?m0))

            (at start (is_base ?b ?m0))
            (at start (detect_mode))
        )
        :effect (and
            (at end (photo_mode))
            (at end (not(detect_mode)))

            (at end (at_base ?r ?b))
            (at end (robot_at ?r ?m0))
            (at end (not(robot_not_at ?r ?m0)))

            (at end (not(robot_at ?r ?m1)))
            (at end (not(robot_at ?r ?m2)))
            (at end (not(robot_at ?r ?m3)))
            (at end (not(robot_at ?r ?m4)))
            (at end (robot_not_at ?r ?m1))
            (at end (robot_not_at ?r ?m2))
            (at end (robot_not_at ?r ?m3))
            (at end (robot_not_at ?r ?m4))

        )
    )



; ;; probelma da qui in poi
    (:durative-action move_to_photograph_first
        :parameters (?r - robot ?m1 - marker  ?m0 - marker ?b - base)
        :duration ( = ?duration 5)
        :condition (and
            (at start (photo_mode))

            (at start (photographed ?m0))
            (at start (unphotographed ?m1))

            (at start (detected ?m1))
            (at start (detected ?m0))
            (at start (not(= ?m1 ?m0)))

            (at start (is_after ?m0 ?m1)) 
            (at start (is_base ?b ?m0))
            
            (at start (robot_not_at ?r ?m1))
            (at start (robot_at ?r ?m0))


        )
        :effect (and

            
            (at end (robot_at ?r ?m1))
            (at end (not(robot_not_at ?r ?m1)))

            (at end (not (robot_at ?r ?m0)))
            (at end (robot_not_at ?r ?m0))

            (at end (photographed ?m1))
            (at end (not(unphotographed ?m1)))

            (at end (not(at_base ?r ?b)))
           
        )
    )


    (:durative-action move_to_photograph
        :parameters (?r - robot ?m1 - marker ?m2 - marker ?m0 - marker ?b - base)
        :duration ( = ?duration 5)
        :condition (and
            (at start (photo_mode))
            
            (at start (not(= ?m1 ?m2)))
            (at start (not(= ?m1 ?m0)))

            (at start (photographed ?m1))
            (at start (unphotographed ?m2))

            (at start (is_after ?m1 ?m2)) 

            (at start (robot_not_at ?r ?m2))
            (at start (robot_at ?r ?m1))

            (at start (detected ?m1))
            (at start (detected ?m2)) 
        )
        :effect (and

            
            (at end (robot_at ?r ?m2))
            (at end (not(robot_not_at ?r ?m2)))

            (at end (not (robot_at ?r ?m1)))
            (at end (robot_not_at ?r ?m1))

            (at end (photographed ?m2))
            (at end (not(unphotographed ?m2)))

            


        )
    )

)