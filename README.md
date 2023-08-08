# IFNET2023
IFNET Camp 2023 Electronics Code


Dear counselors:

We will be demonstrating microelectronics using a microcontroller coupled with a GPS receiver, LoRa radio, and OLED screen:
MakerFocus LoRa GPS Module LoRaWAN 863 923mHz Development Board LoRa Kit Ultra Low Power Design CP2102 SX1262 ASR6502 Chip with 0.96 inch OLED Display and Antenna

This unit is preassembled and there is no wiring/soldering required on our part. This information is for your information, and is not required for participation in the activity.

The above unit consists of an MCU (the "brains" of the electronics), a GPS chip (to receive and decode GPS information), a LoRa radio (LOng RAnge communication protocol radio), and a screen. All of these components come preconnected, we just need to provide instructions on what to do.

For this activity I have programmed all the units using the Arduino IDE (Arduino is an open source electronics platform, specifying hardware organization/communications and a programming language based on C++ with many, many libraries available to operate a variety of hardware). It is possible to program in other languages, with C, C++, and Python being the most common. The nice thing about electronics approved for arduino is that they have a simplified hardware interface (often you just plug in a USB cable) and relatively easy programming interface. Another common hardware/software platform is Raspberry Pi - which historically has entailed much more sophisticated/powerful hardware than arduino. Raspberry pi hardware is more computer-like - allowing, for example, video processing, deep learning, etc. Historically, Arduino hardware is more simplistic, but allows many more inputs/outputs and is MUCH cheaper. So building a system to turn on and off sprinklers in your garden when the soil is dry would be well suited to Arduino, but a system to monitor video and identify faces would be Raspberry pi (or beyond). That said, the hardware processing gap is narrowing, and the latest arduino capable hardware is impressively sophisticated (see the ESP32 chip, for example).  

Here, there will be two types of units:
HIDER - a unit that determines its GPS location, and then broadcasts it blindly
SEEKER - a unit that listens for the HIDER GPS location, and then obtains its own GPS location to determine how far away the HIDER is.

There will be 1 HIDER unit, and any number of SEEKERs. We will use this setup to play a massively unfair game of hide and seek (the hider will have to carry a HIDER transmitter the whole time).

For your reference, the code for the HIDER and SEEKER units is provided here. YOU DO NOT HAVE TO UNDERSTAND THIS CODE OR THE ELECTRONICS INVOLVED - but feel free to ask me about it if you want to learn!!


-Jay
