# TROPIC Double Support on Compliant Ground

This optimization package is developed based on TROPIC [1] that optimize the walking gaits with the Hybrid Zero Dynamics types of controller.

## The major modifications are followings:
1. Optimaze the walking gaits on compliant ground.
2. Optimize The walking gaits that is composed of the double support and sigle support phases as seen the below.

Currently, the compliant ground are modeled by the spring damper model:

<img src="https://latex.codecogs.com/svg.latex?\Large&space;F_{n\in\{x,y\}}=K_pd_n+K_d\dot{d}_n" title="\Large F_{n\in\{x,y\}}=K_pd_n+K_d\dot{d}_n" />
Nonlinear and rigid contact models will be supported soon.


![alt text](https://github.com/DI-X/HelloWorld/blob/master/phase.png | width=100)
<img src="https://github.com/DI-X/HelloWorld/blob/master/phase.png" width="100" 
<img src="spatial_12_dof.gif" width="500">

[1] [[Link]](https://github.com/fevrem/TROPIC/blob/master/MF_PMW_JPS_IROS2020_TROPIC.pdf) M. Fevre, P. M. Wensing, and J. P. Schmiedeler, "Rapid Bipedal Gait Optimization in CasADi", in Proc. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2020, pp. 3672-3678.



