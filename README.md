# 4D-RADAR 

### Class diagram
![ROS_WORK-페이지-4 drawio](https://github.com/user-attachments/assets/d1a5ac63-1679-4d08-98d5-cb6214e7761b)



### Execution Environment
* * *
|    |Environment   |
|----|-------:|
|OS|Ubuntu 20.04/22.04|
|Python|Python 3.10|
|ROS|ROS2 foxy, humbel|

### Installation
------------------
1. Ubuntu (20.04/22.04)
2. ROS2 (foxy, humbel)

### Excution
------------------
1. Clone the repository
```cmd
$ git clone https://github.com/hrin-yoon/4D-RADAR.git
cd 4D-RADAR
```

2. Setting
- Befor multi pc excute,we need to set each PC using DDS(Data Distribution Service)

```cmd
# PC1 & PC2 
$ export ROS_DOMAIN_ID=5
$ echo "export ROS_DOMAIN_ID=5" >> ~/.bashrc #환경변수설정 - 자동 실행 
$ source ~/.bashrc

$ export ROS_LOCALHOST_ONLY=0
$ echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc 
$ source ~/.bashrc
```


3. Excute
-  Single PC communocattion
-  You don'need to 2.Setting
-  If you want to communicate in only one pc, You need to open 4 terminal.

```cmd
$ python3 psudo_radar.py # terminal 1
$ python3 perception.py # terminal 2

$ rviz2 #terminal 3
$ rqt_graph # terminal 4

```
- Multi PC communication
- If you want to communicate in multi pc, You need to open 4 terminal. ( PC1 - 1, PC2 - 3)
```cmd
$ python3 psudo_radar.py # terminal 1 in PC1
$ python3 perception.py # terminal 2 in PC2

$ rviz2 #terminal 3 in PC2
$ rqt_graph # terminal 4 in PC2

```

### Expected result
------------------
[Screencast from 08-28-2024 05:15:29 PM.webm](https://github.com/user-attachments/assets/9b76decd-96f8-4641-a394-12305c022ce0)
