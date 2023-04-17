<p align="center"><img src="https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExMzA2MzhkMTYyZWEwODg5MTAzM2I2NjkwMjc2MjZmNTkzNzk1M2YxYSZjdD1n/LIZGe35ppCezRxJR31/giphy.gif">
<p>Rago is a robotic arm which is designed to play table tennis in real-time by tracking the ball's trajectory and hitting the ball if its within its proximity. Our journey begins by solving the kinematics equation, finding the trajectory for each joint, 3D printing of robotic arm, assembling of parts, and tracking of the ball in realtime. It has been equiped with camera sensor and machine learning algorithm to track the ball. It runs on Raspberry Pi 4B with Raspbian OS</p>

## CONTRIBUTORS
<ul><li>Athul Prasad Prasad Geetha</li>
<li>Aishwarya Palit</li>
<li>Gaurav Madihalli Sreekantegowda</li>
</ul>

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
- Run `./waveguide` to start the program
```
```
>$ cmake ..
>$ make
>$ ./roboticArmPingPong
```
## Social Media
 <img src="https://www.google.com/url?sa=i&url=https%3A%2F%2Fwww.freepik.com%2Ffree-vector%2Finstagram-icon_954290.htm&psig=AOvVaw0KqaFsNB_UFaVrka_hzAYV&ust=1681854427993000&source=images&cd=vfe&ved=0CBAQjRxqFwoTCNjxtJLysf4CFQAAAAAdAAAAABAE"(<https://twitter.com/TeamRago>) <br>
 <img src="https://www.google.com/url?sa=i&url=https%3A%2F%2Fwww.shutterstock.com%2Fsearch%2Ftwitter-logo&psig=AOvVaw1jBQJifQgUQkzi6L8W6b7y&ust=1681854647292000&source=images&cd=vfe&ved=0CBAQjRxqFwoTCKD5wvnysf4CFQAAAAAdAAAAABAJ"(<https://instagram.com/TeamRago>)

