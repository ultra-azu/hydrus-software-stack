
# Mission Planning Package

## About

This is the mission planning package. In this package the goal is to create the planning of the missions that Hydrus will execute. This is consider the last phase of the robot software.  It will connect  with all the publish data and it will send actions to execute to the thrusters, and torpedo.


## Preplanning

This module is the last and more abstract package that is going to be develop. This is because its very hard to predict what things and going to be ready for deployment and information that is going to be ready to use on the submarine. So in order to mitigate that I will encourage to not build interfaces with abstractions with features we think are going to be ready, and only implements things that are ready to use.

## State Machines

Behind this module  is going to be all the features related to autonomy. The core idea behind the autonomy is going to be handle with state machines.


## Planning.
One thing it can be done is that instead of changing from state to state for each instant based on a series of rules we can try to plan the actions ahead. 


## Examples of implementation:

smach - http://wiki.ros.org/smach

moveit - https://moveit.picknik.ai/main/index.html