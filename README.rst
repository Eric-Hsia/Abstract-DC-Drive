#################
Abstract DC Drive
#################

Preview
*******

`Abstract DC Drive Preview <https://www.youtube.com/watch?v=tyCCANgLpCM&feature=youtu.be>`_

Description
***********

This repository is project of control system for DC motor using PID regulator.
It is based on `Abstract-STM32Fx <https://github.com/SlavaLikhohub/Abstract-STM32Fx>`_ library.

Tested using the **STM32F103 Bluepill**. 
If you are using something else consider reviewing pinouts settings.

Example of master program `Abstract-DC-Drive-Master  <https://github.com/SlavaLikhohub/Abstract-DC-Drive-Master>`_.

Example of the circuit `Abstract DC Drive <https://easyeda.com/editor#id=|a6b68a62451c4518ad637d9006d4cecf>`_

Requirements
************

List of requirment provided here `Abstract-STM32Fx <https://github.com/SlavaLikhohub/Abstract-STM32Fx>`_.

How to start
************

#. Make sure you have completed the installation steps described in Requirements_ first.
#. Recursively clone the repository:

   .. code-block:: shell-session
      
      git clone --recursive https://github.com/SlavaLikhohub/Abstract-DC-Drive
      
   or clone first and then initialize all submodules
   
   .. code-block:: shell-session
      
      git clone https://github.com/SlavaLikhohub/Abstract-DC-Drive
      git submodule update --init --recursive
      
#. Check pinouts at beginings of **src/*.c** files.
#. Connect your STM32 to computer.
#. Build and flash the program:
   
.. code-block:: shell-session
      
   make PROFILE=release LOG=0 tidy all

