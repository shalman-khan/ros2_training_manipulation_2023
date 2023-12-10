
Assessment Section 3
====================


Scan Tray 1 and Spawn Tray 1 as Collision Object [MESH]
-------------------------------------------------------


.. image:: _static/scan_tray1.png
   :alt: Alternative text
   :align: center
   
   

Scan Tray 2 and Spawn Tray 2 as Collision Object [MESH]
-------------------------------------------------------

.. image:: _static/scan_tray2.png
   :alt: Alternative text
   :align: center
   
   

Scan ARUCO box and Spawn ARUCO box as Collision Object [BOX]
------------------------------------------------------------

.. image:: _static/scan_box.png
   :alt: Alternative text
   :align: center
   
   
   
.. image:: _static/scantray12_code.png
   :alt: Alternative text
   :align: center


.. image:: _static/boxcode.png
   :alt: Alternative text
   :align: center


Verification of Spawned Collision Objects
------------------------------------------------------------

**In Terminal 1**

.. code-block:: bash

   cd ~assessment_ws/
   colcon build & source install/setup.bash
   ros2 launch pick_and_place ur_bringup.launch.py
   
**In Terminal 2** 

.. code-block:: bash

   cd ~assessment_ws/simulation_student_copy/rosbags
   ros2 bag play <tray1_rosbag> or <tray2_rosbag> or <tray1_and_aruco_box_rosbag> - l
   
   
**In Terminal 3**

.. code-block:: bash

   cd ~assessment_ws/
   source install/setup.bash
   ros2 launch pick_and_place pick_and_place.launch.py
