
;************************************************
;*						*
;*	Proyecto Final 				*
;*                      1/5/20                  *
;*                                              *
;************************************************




(deffacts Initial-state-objects-rooms-zones-actors


; Objects definitions
	( item (type Objects) (name fridge)(room kitchen)(image table)( attributes no-pick brown)(pose 1.5 1.4 0.0))
	( item (type Objects) (name mesa)(room service)(image table)( attributes no-pick brown)(pose 1.6 0.5 0.0))
	( item (type Objects) (name reception)(room corridor)(image table)( attributes no-pick brown)(pose 0.2 1.05 0.0))


	( item (type Objects) (name Apple)(room corridor)(zone corridor)(image Apple)(attributes pick)(pose 0.2 1.05 1.0))
	( item (type Objects) (name Sushi)(room corridor)(zone corridor)(image Sushi)(attributes pick)(pose 0.2 1.05 0.5))
	( item (type Objects) (name Milk)(room corridor)(zone corridor)(image Milk)(attributes pick)(pose 0.2 1.05 0.0))
	( item (type Objects) (name Soap)(room corridor)(zone corridor)(image Soap)(attributes pick)(pose 0.3 1.05 1.0))
	( item (type Objects) (name Perfume)(room corridor)(zone corridor)(image Perfume)(attributes pick)(pose 0.3 1.05 0.5))
	( item (type Objects) (name Shampoo)(room corridor)(zone corridor)(image Shampoo)(attributes pick)(pose 0.3 1.05 0.0))

	( item (type Objects) (name freespace)(room any)(zone any)(image none)(attributes none)(pose 0.0 0.0 0.0))

; Rooms definitions
	( Room (name deposit)(zone deposit)(zones frontentrance)(center 0.75 1.6))
	( Room (name kitchen)(zone kitchen)(zones frontentrance fridge)(center 1.65 1.6)) 
	( Room (name corridor)(zone corridor)(zones dummy1 depositentrance kitchenentrance studioentrance bedroomentrance serviceentrance reception)(center 0.6 1.05))
	( Room (name studio)(zone studio)(zones frontentrance)(center 0.4 0.45))
	( Room (name bedroom)(zone bedroom)(zones frontentrance)(center 1.05 0.45))
	( Room (name service)(zone service)(zones frontentrance)(center 1.65 0.45))
	

; Robots definitions
	( item (type Robot) (name robot)(zone studio)(pose 0.4 0.45 0.0))

; Furniture definitions
;	( item (type Furniture) (name cubestable)(zone bedroom)(image table)( attributes no-pick brown)(pose 6.183334 7.000000 0.0))

; Doors definitions
;	( item (type Door) (name outsidedoor) (status closed) )

	( Arm (name left))

;stacks definitions
        (stack corridor corridor Apple Sushi Milk)
        (stack corridor corridor Soap Perfume Shampoo)

        (real-stack corridor corridor Apple Sushi Milk)
        (real-stack corridor corridor Soap Perfume Shampoo)


	(goal-stack 2 service service Soap Perfume Shampoo)
	(goal-stack 1 kitchen kitchen Apple Sushi Milk)
	
        (plan (name cubes) (number 0)(duration 0))

)



