
## How to build
```
	cmake --preset Release
	cmake --build build
```




## morb란

morb란
  onboard 통신 middleware입니다.
  application 간의 통신은 모두 morb를 통합니다.
  topics.h에 hardware 또는 module마다 자신의 topic이 정의되어 있습니다.
  자신의 topic에 주기적으로 publish하여 다른 application이 자신의 데이터를 
  가져갈 수 있게 합니다.
  원하는 topic을 subscribe하여, callback함수를 통해 데이터를 받을 수 있습니다.
  
## Project 계층
drivers : 
  하드웨어에 대한 드라이버들이 있습니다.
  
lib : 
  하드웨어 독립적인 라이브러리들이 있습니다.
  
platform : 
  stm32f411ceu6 HAL driver가 들어있습니다.
  보드의 페리페럴을 초기화합니다.

middleware : 
  subscribe/publish 방식의 통신 미들웨어 morb가 있습니다.
  
hardware (application) : 
  drivers를 사용하여 하드웨어와 직접 통신하는 코드들이 있습니다.

modules (application) :
  자세추정, 자세제어 코드등이 있습니다.
  하드웨어와 직접 통신하지 않습니다.
  센서 데이터들은 hardware계층에서 publish한 topic을 subscribe하여 사용합니다.

main.c (application) : 
  platform, hardware, modules 를 초기화하고, 루프를 호출합니다.


## Photos
![photo_2026-03-03_15-06-45](https://github.com/user-attachments/assets/3c72dd2d-4eca-452d-9cf0-edc773dd3324)

![photo_2026-03-03_15-06-55](https://github.com/user-attachments/assets/cabb93f9-d984-43c2-a3f2-7e23969e758a)

![photo_2026-03-03_15-06-59](https://github.com/user-attachments/assets/15f91311-2197-41ae-aaf9-42365f0c49a0)

![photo_2026-03-03_15-07-02](https://github.com/user-attachments/assets/0551886b-c4cb-4b8f-a106-19aeb3fe3e01)


