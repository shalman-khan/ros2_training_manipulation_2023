
Assessment Section 4
====================
|

Pick Box from Tray 1 [Height: 0.42m]
------------------------------------


.. image:: _static/pick_tray1.png
   :alt: Alternative text
   :align: center
   
|

Place Box in Tray 2 [Height: 0.35m]
-----------------------------------

.. image:: _static/place_tray2.png
   :alt: Alternative text
   :align: center
   
   

.. image:: _static/pickandplacecode.png
   :alt: Alternative text
   :align: center

|


   
Offset Pick and Place
---------------------

.. image:: _static/offsetpickplace.png
   :alt: Alternative text
   :align: center

**Verification of Pick and Place**


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
