# Requirements and State Transitions

## State transition list format

State transition lists are in the format:

### State_name

#### Event: Event here

**Service:** Service here

**Transition:** Transition here

If a state has more than one event associated with it, it will follow this pattern

### State

#### Event: Event here

**Service:** Service here

**Transition:** Transition here

#### Event: Event here

**Service:** Service here

**Transition:** Transition here

---

## Manager requirements list

Manager will:

- Receive user commands.
- If entered commands are the user requesting to lower the winch, send command to robot to lower winch.
- If entered commands are the user sending run commands, get randomizations and package the information to send to the robots.
- Send the information.
- Wait for ready response.
- Send go signal (`<G>`).
- Receive and print finished statements.
- Receive, parse, and print a battery warning from the robot
- Prompt to do another run.
- Send emergency stop signal.

---

## Manager state transitions

### Wait_for_user_input

#### Event: Received all run commands from user
This means the user command started with `S`, `T`, or `I` and has the appropriate number of entries.

Commands include:

- Run Type (single, tied-in, or independent, demarcated as `S`, `T`, and `I` respectively)
- `Speed1`
- `Speed1` variance
- `winchHeight1`
- `winchHeight1` variance
- `startDelay1`
- `startDelay1` variance
- `lowerDelay1` (when stopped in the middle to lower the load)
- `lowerDelay1` variance
- `hoistDelay1` (when stopped in the middle and waiting to raise load)
- `hoistDelay1` variance
- `hoistStopPosition1` (where to the robot stops to raise/lower)
- `hoistStopPosition1` Variance
- `lowerLoadDepth1` (how far down the load goes when stopped and raise/lowering)
- `lowerLoadDepth1` Variance
- Chance to raise/lower `1` (`0–100`, at `100` robot will always stop to raise/lower)
- `startSideWeight1` (`0–100`, at `0` robot will always start on one side and at `100` robot will always start on the other side; at `50` it is a 50% for either side)
- For independent runs each of these variables will be repeated for the other robot.
- Last input is number of runs in the batch.

**Service:** Parse user commands, generate random values within given range and generate `nextStartSides`.

```
If singleRobot = true,
    Send randomized values and information to node1 robot.
Elseif singleRobot = false && tiedIn = false
    Send randomized values and information to both robots.
Elseif singleRobot = false && tiedIn = true
    Send the same randomized value and information to both robots.
```

**Transition:** `Wait_for_acknowledgement`

#### Event: received a command to lower the winch

**Service:** broadcast a lower winch command to the robots. This command will be structured:

- `<W, depth1, depth2>`

**Transition:** `Wait_for_user_input`

---

### Wait_for_acknowledgement

#### Event: If received an acknowledgement broadcast from the appropriate number of robots for the run type (These acknowledgments are in the form `<A#>` where `#` is `1` or `2` respectively depending on if it is from node 1 or node 2 respectively).

**Service:** Print out acknowledgement received

**Transition:** `Wait_for_ready`

#### Event: If `timeThreshold` is elapsed and acknowledgement broadcast hasn’t been received from the appropriate number of robots for the run type.

**Service:** Print out error message

**Transition:** `Wait_for_user_input`

---

### Wait_for_ready

#### Event: Received ready from appropriate number of robots for case
`R1` and `R2` for tied-in and independent runs, just `R1` for single runs.

**Service:** Broadcast start message (`<G>`)

**Transition:** `Wait_for_finished`

---

### Wait_for_finished

#### Event: If received finished broadcast from appropriate number of robots for the run type (two for `I` and `T`, one for `S`)

**Service:** Print out measured variables from the robot’s finish handshake. Iterate up the run count.

```
if (# of runs < runs in batch){
    prompt user if they want to do the next run in the batch
    nextRunQuestion == True.
    Transition to Wait_for_batch_decision
}
If (# of runs >= runs in batch){
    Clear stored values from previous user command.
    Transition to Wait_for_user_input
}
```

**Transition:** either `Wait_for_batch_decision` or `Wait_for_user_input`

---

### Wait_for_batch_decision

#### Event: If (received a “y” && `nextRunQuestion == True`)

**Service:** Generate the new random values from previous user input and broadcast to robots based on the type of runs. `nextRunQuestion == False`.

**Transition:** `Wait_for_ready`

#### Event: If (received a “n” && `nextRunQuestion == True`)

**Service:** Clear stored values from previous user command. `nextRunQuestion == False`.

**Transition:** `Wait_for_user_input`

---

### Issue Emergency Stop (global function check during each loop)

#### Event: Received emergency stop command from user

**Service:** Broadcast emergency stop signal (`<X>`)

**Transition:** `Wait_for_user_input`

---

### Check for emergency stop (global function check during each loop)

#### Event: Read in a broadcast from the robot that is flagged to say there is an emergency stop
`<B#>`, where `#` is the node’s number (so `B1` or `B2` for node 1 and node 2 respectively).

**Service:** Print out that an emergency stop was issued. And that the battery is low and what nodes sent the warning.

**Transition:** `Wait_for_user_input`

---

## Node requirements list

Node will:

- Receive lower winch command wirelessly from manager (including a rough distance that the winch is unwound).
- Robot lowers winch down by the requested amount (in feet), then goes to wait for command state.
- Lowering the winch requires:
  - Running the winch up ever so slightly (barely get any movement from the encoder)
  - Unlocking pawl via a servo
  - Turning the motor off and then running it the other way
  - When the encoder is at the right spot, turn motor off and relock the pawl
- User attaches the load manually to the winch line.
- Receive run command packet wirelessly from manager.
- Robot parses the commands and sets the command variables.
- If the run type is `S` (single) and the robot is the pseudomanager (node 1) then only it will load in the run commands.
  - If run type is `T` (tied in) then both robots will read in the same run commands (so ideally they will execute the exact same run).
  - If the run type is `I` (independent) than the pseudomanager (node 1) will load in the first set of commands and the full-node (node 2) will load in the second set of commands.
- Robot rehomes the winch.
  - Running the winch upwards doesn’t require doing anything other than running the winch motor.
  - Run winch upwards until a contact switch is hit. Set winch encoder = 0.
- Robot lowers winch to winch height (that was set from the managers command packet).
  - Unlock, lower, lock.
- Robot resets `driveWheel` and `truePosition` encoders = 0.
- Robot sends a ready response to the manager.
- Robot receives a go command from the manager (`<G>`)).
- Robot waits the start delay (sent by the initial run command packet).
- Robot drives forward.
- If (`lowerAndHoist == TRUE`)
  - Robot stops at the `lower_load` position
  - Robot waits `lowerDelay`
  - Robot unlocks, lowers load, then locks winch
  - Robot waits `hoistDelay`
  - Robot unlocks, raises, and locks the winch
- If (`destinationIsStartSide() == true`)
  - Reverse direction and drive back to the same side you started from
- Else
  - Continue drive forward to the end of the rail
- If (reached slowdown position)
  - `Slowdown = TRUE`
- If (contact switch = true)
  - Turn off motors
  - `Slowdown = false`
  - Reset encoders  
    (this requires logic for which side you are on, one side will be 0, the other will be the full length of the rod (30ish feet))
- If (`destinationIsStartSide() == True && lowerAndHoist == FALSE`)
  - Unlock, hoist winch close to top, lock winch
  - Drive to opposite side of rail
  - If (reached slowdown position)
    - `Slowdown = TRUE`
  - If (contact switch = true)
    - Turn off motors
    - `Slowdown = FALSE`
    - Reset encoders  
      Require logic for which side you are on.
- Robot broadcasts finish handshake (which includes measured variables).

Measured variables are:

- Average robot speed in center
- Measured winch height before translation
- Measured winch maximum depth
- Time elapsed from go to robot starting to move
- Time elapsed from go to robot stopping to lower hoist
- Time elapsed from go to finished
- Start delay command
- Lower_delay command
- Hoist_delay command

At any time the robot should be listening for an emergency stop command from the manager.

- If emergency stop command received:
  - Stop all motors
  - Lock winch

Also checking in the loop while waiting for a command (before the start of a run) for if battery voltage gets too low.

- If (too low = true)
  - Broadcast emergency stop to other robot (this should be flagged so that only the other robot reads it in, not the manager)
  - Broadcast battery too low message (to manager)
  - Set state to emergency stop

For variables labeled `1` or `2` at the end, only node 1 should read in the `1`s and node 2 in the `2`s.

The node number should be stored in a definition, so that all it takes to change node 1’s code to node 2’s is to change the definition at one point in the code, and all logic throughout the entire code should be able to adjust to this without needing to be changed.

---

## Node state transitions

### Wait_for_command

**Event:** received a command to lower the winch packet (`<W, depth1, depth2>`)

**Service:** store requested lower distance. Set winch variables (`winchAction = "lower"`, `depth = some_depth`). Set `winchNextState = Wait_for_command`.

**Transition:** `Wait_for_winch_action`

---

### Wait_for_winch_action (in loop be calling the winch() function)

**Event:** `winchComplete == true`

**Service:** reset winch variables

**Transition:** to `[winchNextState]` (this variable is set before entering this state)

---

### Wait_for_command

**Event:** `run_command_packet_received`

**Service:** parse packet, set run variables (respect `S/T/I` logic), send parse acknowledgement to manager (`<A#>` where `#` is the node’s number, so `A1` or `A2`). Set winch variables (`winchAction = "rehome"`). Set `winchNextState = Set_winch_to_command_height`.

**Transition:** `Wait_for_winch_action`

**Event:** `battery_too_low() == TRUE` 

**Service:** `stop motors, lock winch, broadcast emergency stop, (broadcast emergency stop to other node is as follows `!X!`, the `!` flags prevent this message from being read by the manager), broadcast a battery warning message to the manager (`<B#>`, where `#` is the node’s number, so `B1` or `B2`).

**Transition:** `Emergency_stop`

---

### Set_winch_to_command_height

**Event:** state entered (which means the rehome was completed)

**Service:** Set winch variables (`winchAction = "lower"`, `depth = heightCommand`). Set `winchNextState = Send_ready_to_manager`.

**Transition:** `Wait_for_winch_action`

---

### Send_ready_to_manager

**Event:** entered state (which means winch has been lowered to the right height)

**Service:** reset `driveWheel` and `truePosition` encoders to 0, transmit READY packet to manager. (ready package is `<R#>`, where number is the nodes number, so either `R1` or `R2`).

**Transition:** `Wait_for_go`

---

### Wait_for_go

**Event:** go command received (`<G>`)

**Service:** record go command reference timestamp, start `startDelay` timer

**Transition:** `Wait_for_start_delay`

---

### Wait_for_start_delay

**Event:** `startDelay` elapsed

**Service:** record start timestamp, start drive motion (nonblocking)

**Transition:** `Driving`

---

### Driving

#### Event: (lower_load_position reached && `lowerAndHoist == TRUE`)

**Service:** record hoist_stop timestamp, stop drive motion; start `lowerDelay` timer

**Transition:** `Wait_lower_delay`

#### Event: slowdown position reached

**Service:** set `slowdown = TRUE`

**Transition:** `Driving`

#### Event: contact_switch_hit

**Service:** record finished timestamp (only if this is the first time it has traversed the rail this run. If this contact switch being hit was the result of returning to the driving state from Post_traversal_decision than it should not record another time stamp), stop motors; reset encoders based on side logic

**Transition:** `Post_traversal_decision`

---

### Wait_lower_delay

**Event:** `lowerDelay` elapsed

**Service:** set winch variables (`winchAction = "lower"`, `depth = lowerLoadDepth`). Set `winchNextState = Wait_hoist_delay`.

**Transition:** `Wait_for_winch_action`

---

### Wait_hoist_delay

**Event:** enters state

**Service:** 
```
if (hoistDelay timer is not started){start `hoistDelay` timer}
```

**Transition:** `Wait_hoist_delay`

#### Event: `hoistDelay` elapsed

**Service:** set winch variables (`winchAction = "hoist"`, `depth = height`). Set `winchNextState = Resume_driving_after_hoist`.

**Transition:** `Wait_for_winch_action`

---

### Resume_driving_after_hoist

**Event:** enters state (which means winch has been raised again)

**Service:** determine `destinationIsStartSide()`; set drive direction accordingly; resume driving

**Transition:** `Driving`

---

### Post_traversal_decision

**Event:** enters state

**Service:**
```
if (destinationIsStartSide() == TRUE AND currentSide == startSide){transition to Send_finish}
else if (destinationIsStartSide() == TRUE){set winch variables (winchAction = prepareForTraverse) Set winchNextState = Drive_to_opposite. Transition to  Wait_for_winch_action} 
else { transition to Send_finish}
```

**Transition:** either `Wait_for_winch_action` or `Send_finish` (the actual transition is controlled by the logic in the service section above)

---

### Drive_to_opposite

**Event:** state entered

**Service:** set drive variables such to drive to the opposite side.

**Transition:** `Driving`

---

### Send_finish

**Event:** entered state (run is complete)

**Service:** compute measured values, send FINISH handshake. Finished handshake should start with `<F#, …>` where `#` is the nodes number (so `F1` or `F2`).

**Transition:** `Wait_for_command`

---

### Emergency_stop

#### Event: entered state

**Service:** Stop motors, lock winch

**Transition:** `Emergency_stop`

---

### Global Background events (always active, not states)

**Event:** `emergency stop command is received` (`<X>`, or `!X!`)

**Service:** immediate transition to Emergency_stop

