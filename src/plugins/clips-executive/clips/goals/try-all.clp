;---------------------------------------------------------------------------
;  try-all.clp - CLIPS executive - goal to try some sub-goals
;
;  Created: Mon Jun 04 15:47:52 2018
;  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; Sub-type: TRY-ALL-OF-SUBGOALS
; Perform: one goal at a time, ordered by goal priority
; Succeed: if exactly one sub-goal succeeds
; Fail:    if all sub-goals fail or are rejected and at least one failed
; Reject:  if all sub-goals are rejected and none failed
;
; A TRY-ALL parent goal will order the goals by priority and then
; start performing them in order. For goals which are rejected or
; fail, the goal continues with the next goal. On the first
; successfully executed goal, stops execution and succeeds. If all
; sub-goals are rejected, rejects the parent goal. If all sub-goals
; fail, fails the parent goal.
;
; Interactions:
; - User FORMULATES goal
; - User SELECTS goal
; - User EXPANDS goal, consisting of:
;   * create goals with parent ID equal the TRY-ALL goal ID
;   * set TRY-ALL goal mode to EXPANDED
; - Automatic: if no sub-goal formulated -> FAIL
; - Automatic: if all sub-goals rejected -> REJECT
; - Automatic: if all sub-goals failed -> FAIL
; - Automatic: take highest FORMULATED sub-goal and COMMIT to
; - Automatic: DISPATCH committed sub-goal by SELECTING it
; - User: handle sub-goal expansion, commiting, dispatching
; - Automatic: one of the following outcomes for the sub-goal:
;   * REJECTED: mode EXPANDED (re-try with other sub-goal)
;   * FAILED: mode EXPANDED (retry with other sub-goal)
;   * COMPLETED: mode FINISHED, outcome COMPLETED
; User: EVALUATE goal
; User: cleanup goal

(defrule try-all-goal-expand-failed
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type TRY-ALL-OF-SUBGOALS)
							 (mode EXPANDED))
	(not (goal (type ACHIEVE) (parent ?id)))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED)
					(message (str-cat "No sub-goal for TRY-ALL goal '" ?id "'")))
)

(defrule try-all-goal-commit
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type TRY-ALL-OF-SUBGOALS)
							 (mode EXPANDED))
	(goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED)
				(priority ?priority))
	(not (goal (id ~?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED)
						 (priority ?priority2&:(> ?priority2 ?priority))))
	=>
	(modify ?gf (mode COMMITTED) (committed-to ?sub-goal))
)

(defrule try-all-goal-reject
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type TRY-ALL-OF-SUBGOALS)
							 (mode EXPANDED))
	(forall (goal (id ?sub-goal) (parent ?id) (type ACHIEVE))
					(goal (id ?sub-goal) (mode FINISHED) (outcome REJECTED)))
	=>
	(modify ?gf (mode FINISHED) (outcome REJECTED) (committed-to nil))
)

(defrule try-all-goal-failed
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type TRY-ALL-OF-SUBGOALS)
							 (mode EXPANDED))
	(forall (goal (id ?sub-goal) (parent ?id) (type ACHIEVE))
					(goal (id ?sub-goal) (mode FINISHED) (outcome FAILED|REJECTED)))
	(goal (id ?some-subgoal) (mode FINISHED) (outcome FAILED))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED) (committed-to nil))
)

(defrule try-all-goal-dispatch
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type TRY-ALL-OF-SUBGOALS)
							 (mode COMMITTED) (committed-to ?sub-goal))
	?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED))
	=>
	(modify ?gf (mode DISPATCHED))
	(modify ?sg (mode SELECTED))
)

(defrule try-all-goal-subgoal-rejected
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type TRY-ALL-OF-SUBGOALS)
							 (mode DISPATCHED) (committed-to ?sub-goal))
	?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FINISHED) (outcome REJECTED))
	=>
	; cleanup all plan info associated with the rejected sub-goal
	(delayed-do-for-all-facts ((?plan plan)) (eq ?plan:goal-id ?sub-goal)
		(delayed-do-for-all-facts ((?pa plan-action))
			(and (eq ?pa:goal-id ?sub-goal) (eq ?pa:plan-id ?plan:id))
			(retract ?pa)
		)
		(retract ?plan)
	)
	(modify ?gf (mode EXPANDED))
)

(defrule try-all-goal-subgoal-failed
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type TRY-ALL-OF-SUBGOALS)
							 (mode DISPATCHED) (committed-to ?sub-goal))
	?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FINISHED) (outcome FAILED))
	=>
	; cleanup all plan info associated with the rejected sub-goal
	(delayed-do-for-all-facts ((?plan plan)) (eq ?plan:goal-id ?sub-goal)
		(delayed-do-for-all-facts ((?pa plan-action))
			(and (eq ?pa:goal-id ?sub-goal) (eq ?pa:plan-id ?plan:id))
			(retract ?pa)
		)
		(retract ?plan)
	)
	(modify ?gf (mode EXPANDED))
)

(defrule try-all-goal-subgoal-completed
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type TRY-ALL-OF-SUBGOALS)
							 (mode DISPATCHED) (committed-to ?sub-goal))
	?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FINISHED) (outcome COMPLETED))
	=>
	(modify ?gf (mode FINISHED) (outcome COMPLETED))
)