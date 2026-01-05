(define (problem navigation_problem) (:domain navigation)
(:objects
    robot1 - robot
    marker1 marker0 marker3 marker2 marker4 - marker
    base - base
)
(:init

    (undetected marker1)
    (undetected marker2)
    (undetected marker3)
    (undetected marker4)

    (detected marker0)
    (photographed marker0)

    (unphotographed marker1)
    (unphotographed marker2)
    (unphotographed marker3)
    (unphotographed marker4)



    (is_base base marker0)
    (robot_at robot1 marker0)

    (robot_not_at robot1 marker1)
    (robot_not_at robot1 marker2)
    (robot_not_at robot1 marker3)
    (robot_not_at robot1 marker4)

    (is_after marker0 marker1)
    (is_after marker1 marker2)
    (is_after marker2 marker3)
    (is_after marker3 marker4)
    

    (detect_mode)
    (free robot1)
    
 
)
(:goal
    (and
; (photo_mode)
    (photographed marker1)
    (photographed marker2)
    (photographed marker3)
    (photographed marker4)
    )
)
)
