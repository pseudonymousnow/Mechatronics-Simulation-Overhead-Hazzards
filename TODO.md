# Project TODO

## Core Tasks

- [ ] Node occasionally gets hung without sending ready (hard to replicate), debug
- [ ] pawl currently doesn't lift for initial homing, adjust. (not a library issue, this is directly implemented in node_main.cpp)
- [ ] Check that the after run reported measured values make sense, adjust calculations in node_main if necessary
- [ ] Double check winch radius/winch encoder reading
- [ ] during rehoming really wild swinging can cause issues (may need to change when the code is allowed to read the contact switch. Robot thinks it teleports across the rail when contact switch pressed prematurely)
- [ ] fix manager not recognizing battery warning sent by node
- [ ] make and implement a control system lookup table
- [ ] check the code and values flagged with !CHECK!

## Second Robot Tasks
- [ ] implement tied in and indepentant run cases
- [ ] make sure battery warning broadcast tells the other robot to make an emergency stop. 

