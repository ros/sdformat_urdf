^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sdformat_urdf
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2026-02-17)
------------------
* chore: remove tinyxml2 vendor (`#40 <https://github.com/ros/sdformat_urdf/issues/40>`_)
* Contributors: Daisuke Nishimatsu

2.0.2 (2025-07-04)
------------------
* Relax the version of urdfdom_headers (`#39 <https://github.com/ros/sdformat_urdf/issues/39>`_)
* Explain how Gazebo and RViz can resolve mesh URIs (`#26 <https://github.com/ros/sdformat_urdf/issues/26>`_) (`#29 <https://github.com/ros/sdformat_urdf/issues/29>`_)
* Contributors: Alejandro Hernández Cordero, Andrew Symington

2.0.1 (2024-04-23)
------------------

2.0.0 (2024-04-23)
------------------
* Use gz vendor packages and update to Harmonic (`#28 <https://github.com/ros/sdformat_urdf/issues/28>`_)
* Support Gazebo Harmonic (`#23 <https://github.com/ros/sdformat_urdf/issues/23>`_)
* ign to gz migration (`#19 <https://github.com/ros/sdformat_urdf/issues/19>`_)
  * ign->gz
  * revertingsdf13 changes
  ---------
* Create sdformat_urdf_plugin as SHARED instead of MODULE for macOS compatibility (`#22 <https://github.com/ros/sdformat_urdf/issues/22>`_)
  * Create sdformat_urdf_plugin as SHARED instead of MODULE for macOS compatibility
  * Remove trailing whitespace
* Support Gazebo Garden (`#17 <https://github.com/ros/sdformat_urdf/issues/17>`_)
* updating gtest conditionals (`#16 <https://github.com/ros/sdformat_urdf/issues/16>`_)
* Contributors: Addisu Z. Taddese, Dharini Dutia, Louise Poubel, Rhys Mainwaring, Silvio Traversaro

1.0.1 (2022-06-21)
------------------
* Remove version requirement on sdformat_test_files (`#15 <https://github.com/ros/sdformat_urdf/issues/15>`_)
* Contributors: Shane Loretz

1.0.0 (2022-06-21)
------------------
* Support for universal and ball joint as floating joints `#13 <https://github.com/ros/sdformat_urdf/issues/13>`_
* Support SDFormat 12 (`#12 <https://github.com/ros/sdformat_urdf/issues/12>`_)
* Use urdfdom_headers::urdfdom_headers (`#5 <https://github.com/ros/sdformat_urdf/issues/5>`_, `#6 <https://github.com/ros/sdformat_urdf/issues/6>`_))
* Contributors: Dharini Dutia, Louise Poubel, Shane Loretz

0.1.0 (2020-11-02)
------------------
* Initial urdf parser plugin that parses sdformat (`#1 <https://github.com/ros/sdformat_urdf/issues/1>`_)
* Contributors: Shane Loretz, Addisu Z. Taddese, Steve Peters, Chris Lalancette, Eric Cousineau
