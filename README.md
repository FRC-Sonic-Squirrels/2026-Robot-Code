# 2026-Robot-Code
## Introduction
Competition code for team 2930, The Sonic Squirrels, 2026 FRC Rebuilt robot "Doug".

Won Creativity award at Clackamas for our dual intake system.

---------------------------------------------------------------------------

## Key Features

* Periodically controlled intake, shooter, and mechanism subsystems.

* Dual intake system with hard coded extention limits

--------------------------------------------------------------------------

## Autonomous
* State machine system that goes through the actions of intaking and scoring fuel as well as pathfinding.

--------------------------------------------------------------------------

## Periodic Subsystems
* The intake and shooter subsystems run through periodic checks (checking the state of the subsystem based on enum states) to detemine what action they should be taking.

* Using constants throughout the robot that are changed when certain commands are called, the state of these subsystems is able to be altered.

--------------------------------------------------------------------------

## Extras
* Fully autonomous aiming of the robot through set positions using data from our vision system.
--------------------------------------------------------------------------

## Credits & References

- 3061-lib for the swerve library and base advantage kit structuring [3061 Lib](https://github.com/HuskieRobotics/3061-lib)
* MK4/MK4i code initially from Team 364's [BaseFalconSwerve](https://github.com/Team364/BaseFalconSwerve)
* general AdvantageKit logging code, AdvantageKit-enabled Gyro classes, swerve module simulation and drive characterization from Mechanical Advantage's [SwerveDevelopment](https://github.com/Mechanical-Advantage/SwerveDevelopment)
* Setting up Spotless code linting [WPILib Spotless setup](https://docs.wpilib.org/en/latest/docs/software/advanced-gradlerio/code-formatting.html#spotless)
* Sleipnir Group Choreo [Choreo](https://github.com/SleipnirGroup/Choreo)
* Photonvision for gathering information from the coprocessors [Photonvision](https://photonvision.org/)
