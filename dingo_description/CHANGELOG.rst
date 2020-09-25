^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dingo_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.1.1 (2020-09-14)
------------------
* Disable gravity on the L515 links; they were causing the odom frame to drift in gazebo
* Nav improvements (`#5 <https://github.com/dingo-cpr/dingo/issues/5>`_)
  * Expose the scan_topic argument in the gmapping_demo and amcl_demo launch files
  * Add placeholder support for the RS L515 and D455 so that the gazebo plugins work; the meshes for these sensors don't exist yet, but we can at least get the plugin configured & add the appropriate links for now
  * Refactor the RealSense macro to put the mesh + gazebo plugin in one place. Create a more-accurate L515 mesh out of cylinders until Intel releases an official mesh for the sensor
* Fix missing quotes around the lidar models
* Model Update (`#3 <https://github.com/dingo-cpr/dingo/issues/3>`_)
  * Make the yellow of the chassis more yellow and less orange
  * Fix the spawn position of the Hokuyo lidar & its focal point. Add the STLs for the LMS-1xx & Hokuyo lidars.  Add lms1xx as a dependency so that the lidar macro is available.
  * Add additional mounting points for accessories, fix the position of the existing front_mount
  * Start adding support for the Intel RealSense cameras as out-of-the-box accessories
  * Update the dependencies
  * Remove the hector gazebo plugin dependency; it's included in dingo_gazebo, and isn't strictly needed in the description package
  * Refactor the lidar accessory variables, add clarifying comments to make it easier to find supported values for the sensor models
  * Fix the measurements.  Keep the front/rear mounts 5cm back from the bumpers, move the bumper mounts to the center of the bumper
  * Remove the rear_bumper_mount since that's where the HMI is, which would prevent anything from being reasonably mounted there
* [dingo_description] Switched to find over dirname since it was causing errors in tests.
* Fix the spawn position of the Hokuyo lidar & its focal point. Add the STLs for the LMS-1xx & Hokuyo lidars.  Add lms1xx as a dependency so that the lidar macro is available.
* Contributors: Chris I-B, Chris Iverach-Brereton, Tony Baltovski

0.1.0 (2020-08-10)
------------------
* Unified Dingo folders/files as much as possible, updated meshes, bumped CMake version and made roslaunch a test depend.
* Added dingo_control; split dingo_description into separate Dingo-D and Dingo-O flavours; still much work to be done here, but preparing hand-off for Tony
* Moved IMU
* Updated joint names for simplicity
* Fixed collision
* Added description
* Contributors: Dave Niewinski, Jason Higgins, Tony Baltovski
