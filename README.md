# Hoverwheel
![IMG_0981](https://github.com/bwees/hoverwheel/assets/12686250/e0d7de32-7833-40df-8884-ec7927dcdfcf)

This project set out to make an Onewheel alternative for under $400. Hoverwheel uses an old hoverboard for the motors and almost all of the electronics. 

An ESP32 provides high-speed balance control and control parameter adjustments over BLE.
This project does not use VESC and instead, the control algorithm has been written from scratch.

I am still working on a full write-up about the development of this project and will post a link here when done.

## Firmware
The firmware was written in C++ with the Arduino framework inside of PlatformIO. Arduino was used to maximize compatibility with other microcontrollers.

## Web Configurator
I wrote a small web app before the iOS app to control parameters. I am no longer updating it with new settings and changes as it is not very useful outside of the initial setup. Once iOS devices support WebBluetooth I will continue development and likely rewrite it.

## App
I wrote a SwiftUI iOS/Watch app to control the BLE parameters as well as provide sensor data such as battery voltage, speed, and motor duty cycle. 


Settings Pane | Live Sensor Pane
--- | ---
![IMG_1319](https://github.com/bwees/hoverwheel/assets/12686250/5f0a3534-6f06-4725-a782-1c1a3dc1eaa3) | ![IMG_1318](https://github.com/bwees/hoverwheel/assets/12686250/4130ecc4-cfb5-45d8-93e7-4beb22646d46) 

### Watch App
![incoming-1F9A280C-2333-4C53-8C56-6325C7F6C59C](https://github.com/bwees/hoverwheel/assets/12686250/de321d09-e273-413c-8cad-ff9c8973d079)

