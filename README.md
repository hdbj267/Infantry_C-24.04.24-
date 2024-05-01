tip:在工程里面看才是正常格式

步兵后续规划（2025开始）：

1.更换新的限功率的方案

2.自瞄调pid

3.完成功率控制板与a板的can通信

4.解决ui界面不推送的问题

5.之后麦轮代码更换为全向轮代码

重点关注下面的task和app(重要度排序)：
gimbal_task/

shoot_task/

rule.c/

connect_task/

can2_app/

can1_app/

初此之外，还要关注

usart.c

stm32f4xx_it.c/

其他的基本上用不上/不重点关注
