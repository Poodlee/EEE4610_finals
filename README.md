# :pushpin: 사막개미 기반 배달 RC카 만들기 

<p align="center">
  <img src="https://github.com/user-attachments/assets/a9556ae4-aa6c-4526-9497-780cc274e060" alt="image" width="300"/>
</p>

</br>

## 1. 프로젝트 개요
- 목적: 사막개미를 모티브로 한 저렴하고 빠르게 할 수 있는 배달 RC카를 만들기
- 제작 기간: 2023년 9월 ~ 12월
- 참여 인원: 팀 프로젝트(3인)

</br>

## 2. 사용 언어 및 환경 
  - C++
  - Ubuntu 20.04 + ROS1 Noetic 

</br>

## 3. 핵심 기능
해당 코드의 주된 기능은 사막 개미의 이동 방식을 모티브로 해서 저렴한 배달 RC카를 만드는 것입니다.
그래서 desert_ant_navigation_node를 실행하고 목표 좌표를 x,y(m) 입력하면 RC카가 이동하게 제작했습니다. 


<summary><b>핵심 기능 설명 펼치기</b></summary>
<div markdown="1">
해당 프로젝트의 경우 사막 개미의 이동 방식을 모티브로 한 내용으로 아래 표와 같이 각각의 사막 개미의 이동 방식을 RC카에 맞게 수정했습니다.

<p align="center">
  <img src="https://github.com/user-attachments/assets/278697bb-5272-4273-9954-a66483683fab" alt="image" width="600"/>
</p>

자세한 내용은 아래 보고서에 작성했습니다.
[[종합설계보고서].pdf](https://github.com/user-attachments/files/17011959/7.pdf)

### 4.1. 전체 흐름
전체적으로는 아래와 같이 node에 희망하는 좌표를 넣으면 장애물을 피해 해당 위치에 도달한 이후 다시 원래 위치로 돌아오게 하는 것이었습니다.
영상에서 왼쪽은 gazebo로 simulation한 내용이고 오른쪽은 모터의 통신 값을 기준으로 path integration한 결과입니다. 

<p align="center">
  <img src="https://github.com/user-attachments/assets/6b62866a-584d-447f-b2d4-bf38c66750fe" alt="image" width="600"/>
</p>



### 4.2. Path Integration 📍[코드 확인](https://github.com/Poodlee/EEE4610_finals/blob/main/catkin_ws/src/desert_ant_navigation_node.cpp#L324)
모터의 PWM 신호로 이동한 정도를 파악했고 이때 직선 주행의 경우에는 단순 적분 곡선 주행의 경우에는 ackerman steering equation을 이용해 업데이트를 진행했습니다.  
```
Algorithm 2 Path Integration
	Input: cmd_stamped - an AckermannDriveStamped message containing vehicle speed, steering angle, and timestamp
	Output: Updated vehicle pose incorporating the new position and orientation

Data:
  WHEEL_BASE: Distance between the front and rear wheels of the vehicle (meter)
  WHEEL_AXIS: Distance between the left and right wheels of the vehicle (meter)
  CAR_SPEED: Nominal speed of the car (m/s)
  pose_: Current pose of the vehicle, including translation and rotation
  dr_cmd_: Command structure for dead reckoning containing speed, steering angle, and timestamp


1	Extract Vehicle Commands:
speed ← cmd_stamped.drive.speed
steer ← cmd_stamped.drive.steering_angle
timestamp ← cmd_stamped.header.stamp

2	Compute Time Difference:
dt ← timestamp - pose_.timestamp

3	Update Pose:
if (speed ≠ 0) then
   if (steer ≠ 0) then
       R ← Calculate turn radius using WHEEL_BASE, WHEEL_AXIS, and steer
      omega ← speed / R
      dtheta ← omega * dt
      Calculate rotation center based on pose_ and R
      Update pose_.translation and pose_.rotation for turning motion
   else
      Update pose_.translation for straight-line motion
   end if
end if
pose_.timestamp ← timestamp

4	Visualize Updated Pose:
Call visualizePose() function
5	Compute Dead Reckoning Command:
steer ← Calculate steering angle towards the goal
speed ← Set speed based on dt
Update dr_cmd_ with new speed, steer, and current time

```

### 4.3. Obstacle Avoidance 📍[코드 확인](https://github.com/Poodlee/EEE4610_finals/blob/main/catkin_ws/src/desert_ant_navigation_node.cpp#L362)
전방에 장애물이 등장할 시에 차량의 폭과 비교해 지나갈 수 있는 Open 플레이스 중 목적지와 방향이 최대한 유사한 곳으로 회피하게 설정했습니다. 

<p align="center">
  <img src="https://github.com/user-attachments/assets/2aabe56d-2e78-46d3-8b8c-8ed8eded335c" alt="image" width="400"/>
</p>

### 4.4. Position Update 
position update의 경우에는 lidar와 주어진 map을 기준으로 하는 Localization에 대해서 진행했습니다. 아래 결과 사진을 보게 되면 실제로 더 정확하나 연산이 오래 걸려 저속에서는 사용했으나 고속에서는 Path integration 만으로 진행했습니다. 

영상

<p align="center">
  <img src="https://github.com/user-attachments/assets/5cdcc248-510a-4051-98a3-b4b48075e3fe" alt="image" width="400"/>
</p>

결과 사진

<p align="center">
  <img src="https://github.com/user-attachments/assets/3e68fe49-5883-490d-83ee-d04b9fbeba66" alt="image" width="300"/>
</p>

</div>


</br>
