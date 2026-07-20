# BEEP Final Project - Miniature Keyboard

Developed by Aaryan Sagar & Akilan Dayasankar

Welcome to our project! As part of [Embedded Systems @ Purdue](https://www.esap.dev/)'s Boilermaker
Entry Embedded Program (BEEP), we developed a miniature electric keyboard. This keyboard the
following features:

* 12 keys in the C chromatic scale
* a knob to control note volumes
* buttons to shift the keyboard up and down octaves in C

![Photo of piano](media\photo.jpg)

## Demo

A demonstration of the video can be found [here](https://youtu.be/ytdognYXWMQ).

## Electronics

The piano was originally constructed with materials from the
[SunFounder ESP32 Ultimate Starter Kit](https://www.sunfounder.com/collections/esp32-1/products/sunfounder-esp32-ultimate-starter-kit-with-esp32-camera-extension-board-battery)
. The code was written specifically for an ESP32-WROOM-32E microcontroller. In its initial design,
it uses multiple push buttons, a potentiometer, an HXJ8002 audio amplifier module, and a speaker.
The schematic below details how all the components are put together.

![Schematic of piano](media\schematic.svg)

## Code

The code for the piano was written mainly in C, one of the foremost language for embedded systems.
Our code uses these features:

* lookup tables to quickly access note frequencies & loudnesses
* a direct digital synthesizer to create sound waves which blend different notes (frequencies)
  together
* a log-linear interpolation to approximate equal-loudness level contours standardized under ISO
  226:2023
