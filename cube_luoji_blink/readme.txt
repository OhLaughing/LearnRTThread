灯闪裸机程序
## 目录结构
Drivers
	STM32L4xx_HAL_Driver   -- 从STM32Cube_FW_L4_V1.17.0\Drivers\STM32L4xx_HAL_Driver 拷贝
	MDK-ARM
		startup_stm32l475xx.s	-- STM32Cube_FW_L4_V1.17.0\Drivers\CMSIS\Device\ST\STM32L4xx\Source\Templates\arm\startup_stm32l475xx.s

├─Core  
│  ├─Inc  -- STM32Cube_FW_L4_V1.17.0\Projects\B-L475E-IOT01A\Templates\Inc #有不同
│  └─Src  -- STM32Cube_FW_L4_V1.17.0\Projects\B-L475E-IOT01A\Templates\Src
├─Drivers
│  ├─CMSIS
│  │  ├─Device
│  │  │  └─ST
│  │  │      └─STM32L4xx
│  │  │          ├─Include
│  │  │          └─Source
│  │  │              └─Templates
│  │  │                  ├─arm
│  │  │                  ├─gcc
│  │  │                  └─iar
│  │  │                      └─linker
│  │  └─Include
│  └─STM32L4xx_HAL_Driver   ---- 从STM32Cube_FW_L4_V1.17.0\Drivers\STM32L4xx_HAL_Driver 拷贝
│      ├─Inc
│      │  └─Legacy
│      └─Src
└─MDK-ARM
    ├─cube_luoji_blink
    ├─DebugConfig
    └─RTE
        └─_cube_luoji_blink