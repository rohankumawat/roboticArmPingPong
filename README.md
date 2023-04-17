<p align="center"><img src="https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExMzA2MzhkMTYyZWEwODg5MTAzM2I2NjkwMjc2MjZmNTkzNzk1M2YxYSZjdD1n/LIZGe35ppCezRxJR31/giphy.gif">
<p>Rago is a robotic arm which is designed to play table tennis in real-time by tracking the ball's trajectory and hitting the ball if its within its proximity. Our journey begins by solving the kinematics equation, finding the trajectory for each joint, 3D printing of robotic arm, assembling of parts, and tracking of the ball in realtime. It has been equiped with camera sensor and machine learning algorithm to track the ball. It runs on Raspberry Pi 4B with Raspbian OS</p>
<br>

## CONTRIBUTORS
<ul><li>Athul Prasad Prasad Geetha - @thelastsayian</li>
<li>Aishwarya Palit - @aish0512</li>
<li>Gaurav Madihalli Sreekantegowda - @gauravam</li>
</ul>
<br>

## DEPENDENCIES

To build and run 'Rago', you need the following dependencies:

- CMake (2.8 or greater)
- OpenCV (4.5 or greater)
- Wiring Pi
- Eigen 

- To do anything further, clone this repository to your Raspberry Pi using the `git clone` command or by downloading the repository as a .zip file.
```
>$ https://github.com/rohankumawat/roboticArmPingPong.git
```

- Navigate to the 'roboticArmPingPong' directory.
```
>$ cd roboticArmPingPong
```
- Make the ``` dependency.sh ``` executable.
```
>$ chmod +x dependency.sh
```
- Now run the executable.
```
>$ ./dependency.sh
```

<br>

## Installation

- Navigate to the 'roboticArmPingPong' directory. 
 ``` 
 >$ cd roboticArmPingPong
  ```

```
- Run `cmake .` to generate the Makefile
- Run `make` to compile the source code
- Run `./roboticArmPingPong` to start the program
```
```
>$ cmake ..
>$ make
>$ ./roboticArmPingPong
```
<br>
## Document
[document/RealTime Embedded Programming.docx](https://github.com/rohankumawat/roboticArmPingPong/blob/main/document/RealTime%20Embedded%20Programming.docx)
<br>

## Social Media
Twitter: https://twitter.com/TeamRago<br>
Instagram: https://instagram.com/rago_the_robo

