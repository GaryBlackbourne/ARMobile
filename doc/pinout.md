#stm32f303re:

 - PA3 - RX USART2
 - PA2 - TX USART2

 - PC10 - bluetooth tx
 - PC11 - bluetooth rx

 - PB4 - PWM1 right
 - PB10 - PWM2 left

 - PA4 - LEFT 1
 - PB0 - LEFT 2

 - PC1 - RIGHT 1
 - PC0 - RIGHT 2

 
# DIRECTIONS:

### right forward 
 - PC1 = 1
 - PC0 = 0

### right backward
 - PC1 = 0
 - PC0 = 1

### left forward
 - PA4 = 1
 - PB0 = 0

### left backward
 - PA4 = 0
 - PB0 = 1

### stop
 - PC1 = PC0
 - PA4 = PB0
