Проект вездехода на гусеничной платформе.
Управление: 
STM32F407VGT6 - управление движением, периферия, сенсоры, сервоприводы камеры.
Orange Pi Zero2 - веб-сервер, камера, связь, навигация.

Интерфейсы
I2C1 - компас и прочие датчики
I2C2 - дисплей 
I2C3 - дальномеры через мультиплексор
SPI1 - радиомодуль LoRa
SPI3 - флеш-память распаяна на плате
USART1 - связь с апельсином
USART2 - связь еще с чем резерв
USART3 - связь еще с чем резерв
UART5 - связь с gps-модулем


Таймеры
TIM1 - резерв выходов нет
TIM2 - высокой разрядности резерв
TIM3 - энкодер левого колеса
TIM4 - энкодер правого колеса
TIM5 - высокой разрядности резерв
TIM6 - внутренний таймер разрядность 1 мксек выходов нет
TIM7 - внутренний таймер разрядность 1 мксек выходов нет
TIM8 - ШИМ светодиодов фары 2кГц
TIM9 - ШИМ для двигателей два канала 2кГц
TIM10 - ШИМ серво поворот камеры влево-вправо 50Гц
TIM11 - ШИМ серво поворот камеры вверх-вниз 50Гц
TIM12 - ультразвук разрядность 1 мксек подчинен TIM13
TIM13 - выходов нет, используется как мастер для запуска TIM12, период 100мсек
TIM14 - выходов нет, резерв