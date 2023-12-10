Assessment Section 5
====================
|

.. image:: _static/additionaLex.png
   :alt: Alternative text
   :align: center

|

.. raw:: html

   <div style="font-size: larger; font-weight: bold;">
      Repeat Pick and Place Sequence
   </div>

|

.. image:: _static/repeatpickplace.png
   :alt: Alternative text
   :align: center

|   
   

.. raw:: html

   <div style="font-size: larger; font-weight: bold;">
      To run real hardware 
   </div>

|

**In Terminal 1**

.. code-block:: bash

   cd ~assessment_ws/
   colcon build & source install/setup.bash
   ros2 launch pick_and_place ur_bringup.launch.py
   
**In Terminal 2** 

.. code-block:: bash

   cd ~assessment_ws/simulation_student_copy/rosbags
   ros2 launch gripper_driver_interface gripper_bringup.launch.py
   ros2 launch robotiq_ros_service robotiq_ros_service.launch.py
   
   
**In Terminal 3**

.. code-block:: bash

   cd ~assessment_ws/
   source install/setup.bash
   <Activate Joint Trajectory Controller>>
   ros2 launch pick_and_place pick_and_place.launch.py
