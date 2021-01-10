This is an embedded application running on top of FreeRTOS. Nucleo
STM32F446RE is the board of choice here. This application will run
a main menu which will prompt the user over a serial monitor to select
to:
- Display and change time and date; set a daily alarm if needed
- Play guess-a-number game
- Run an integers calculator
- Toggle an LED on the Nucleo board
- Run a temperature monitor in the background to track current,
  highest, and lowest ambient temperatures
- Put the application to sleep and wait for a user interrupt