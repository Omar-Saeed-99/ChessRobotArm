# ChessRobotArm
pick and place robot arm with chess playing feature 

This is the first iteration of my project, which I've named CheBo.It involves a robotic arm designed to hold a real-life chess board and use image processing techniques to identify the chess pieces on the board.
An AI-powered chess engine is then utilized to predict the computer's next move, which is subsequently sent to an Arduino microcontroller to control the robot arm and make the corresponding move on the physical board.

Using ROS I have made a package using the URDF file from solid work and pass
it into the gazebo simulation environment.

![Screenshot from 2023-05-21 17-30-53](https://github.com/Omar-Saeed-99/ChessRobotArm/assets/87039861/936a5353-6f1d-47e3-8ed6-2878578d6c71)

I use a yolo model to recognize our pieces types so I made a data set of 250
photo and trained it for 25 Epocs.

![Screenshot from 2023-03-26 14-34-09](https://github.com/Omar-Saeed-99/ChessRobotArm/assets/87039861/cdedebcf-f925-4985-a127-fda56657f39f)

![Screenshot from 2023-05-27 13-14-27](https://github.com/Omar-Saeed-99/ChessRobotArm/assets/87039861/87e0a7c4-454d-41a7-a3cb-46af15f79d7e)

I used the opencv Chess corners detections do know the 64 square and map the real chess board

![Screenshot from 2023-04-21 22-17-34](https://github.com/Omar-Saeed-99/ChessRobotArm/assets/87039861/9a06243c-7931-469b-b598-a8e845148cd0)

A GUI has been made using PYQT to make the things clear to the user
![Screenshot from 2023-05-27 11-49-51](https://github.com/Omar-Saeed-99/ChessRobotArm/assets/87039861/fd14573b-823d-4326-a15a-265203bf76ff)

SolidWorks design 

![1685983508650](https://github.com/Omar-Saeed-99/ChessRobotArm/assets/87039861/f7f7061f-ad91-4af6-be61-8d3313be98d4)

Real Robot
![1685983496842](https://github.com/Omar-Saeed-99/ChessRobotArm/assets/87039861/56564355-c8af-49b9-bd97-c60191396fc0)
