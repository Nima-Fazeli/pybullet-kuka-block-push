# pybullet-kuka-block-push

## Script explanation
````
kukaBlock.py 
````
Runs the simulations of a Kuka robot pushing a square block on a horizontal surface. ```withVis``` flag turns the visualization of the environment on and off, recommended off for large-scale data collection. ```simRuns``` is the number of simulations to be run. ```withRand``` specifies whether the initial conditions are random or not, leave ```True``` for data collection.

*Note:* Every call to the ```.simulate``` method resets the robot and block (block to different initial conditions). 

```
KukaBlockClass.py
```
Does all the under-the-hood simulation setup and running. 
