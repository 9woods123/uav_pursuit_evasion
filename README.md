# uav_pursuit_evasion
uav_pursuit_evasion


Pursuit_evasion using LOS（line of sight) , simulator rotorS is used to verify.  


Welcome to the uav_pursuit_evasion wiki!



```
cd  yourcatkinws

git clone https://github.com/9woods123/uav_pursuit_evasion.git

git clone https://github.com/catkin/catkin_simple.git

git clone  https://github.com/ethz-asl/rotors_simulator.git

catkin_make
```


For single  pursuit  singe evasion:

` roslaunch pursuit_evasion SPSE_constant_evasion_theta.launch `

The singe evasion UAV should fly at a constant evasion theta.

For multi pursuit  singe evasion:

` roslaunch pursuit_evasion MPSE_random_evasion_theta.launch `

The singe evasion UAV should fly at a random evasion theta.
