## 上海交通大学交龙战队2019 Robomaster 步兵机器人
## SJTU JDragon Robomaster 2019 Infantry

本代码继承于上海交通大学交龙战队RoboMaster机甲大师赛<a href="https://github.com/1105042987/RM_frame">机器人统一框架</a>.

采用状态机来转换机器人工作状态，状态机切换逻辑如下：
遥控器右上拨杆控制inputmode

|      Position     | inputmode|
|--------- 			   | -----------|
|upper | REMOTE_MODE|
|middle | KEYBOARD_MODE|
|lower | STOP |

遥控器左上拨杆控制functionmode

| Position|functionmode|
|----|----|
|upper|UPPER_POS|
|middle|MIDDLE_POS|
|lower|LOWER_POS|

二者组合控制WorkState
WorkState开机初始化PREPARE_STATE, 开机初始化后自动入NORMAL_STATE

|	inputemode|	functionmode| WorkState |
|:----:|:----:|:----:|
|STOP| * |STOP_STATE|
|REMOTE_INPUT|UPPER_POS|NORMAL_STATE|
|REMOTE_INPUT|MIDDLE_POS|ADDITIONAL_STATE_ONE|
|REMOTE_INPUT|LOWER_POS|ADDITIONAL_STATE_TWO|

