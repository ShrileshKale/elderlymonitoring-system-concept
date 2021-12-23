(define (domain actuation)
    (:requirements 
        :strips 
        :typing
        :negative-preconditions
    )
    
    (:predicates
      (led ?l)
      (ON ?l)
      (Appliance ?a)
      (Input ?a)
      
	  (user ?k)
    )
    
    (:action Led-ON
      :parameters (?l ?a)
      :precondition (and (led ?l) (not(ON ?l)) (Input ?a))
      :effect (ON ?l)
    )
    
    
    (:action Led-OFF
      :parameters (?l ?a)
      :precondition (and (led ?l) (ON ?l) (not(Input ?a)))
      :effect (not (ON ?l))
    )
    
    
    (:action Appliance-ON-Input
      :parameters (?a ?k ?l)
      :precondition (and (user ?k) (Appliance ?a) (not(Input ?a)))
      :effect (Input ?a)
    )
    
    
    (:action Appliance-OFF-Input
      :parameters (?a ?k ?l)
      :precondition (and (user ?k) (Appliance ?a) (Input ?a))
      :effect (not(Input ?a))
    )
    
   
)             

 
