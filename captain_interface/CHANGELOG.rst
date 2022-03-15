^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package captain_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#16 <https://github.com/smarc-project/lolo_common/issues/16>`_ from NiklasRolleberg/noetic-devel
  Noetic devel
* started adding feedback from VBS
* Publishing battery information and temperature variance
* Publishing water temperature
* Changed message type for rudder, elevator and thrusters
* added publishers for controller status & dvl status
* Added the service node to the interface launchfile and added captain ip as a variable in lolo_bringup
* Added services to enable/disable control inputs on the captain
* changed rosparam for captain ip so it works
* added rosparam with the ip address of the captain
* added rosparam for captain ip and port
* first working version of udp inteface
* Merge pull request `#14 <https://github.com/smarc-project/lolo_common/issues/14>`_ from nilsbore/test_release
  Release building
* Added install targets for captain interface
* Merge pull request `#12 <https://github.com/smarc-project/lolo_common/issues/12>`_ from NiklasRolleberg/noetic-devel
  bug fix in captain interface and more parameters in launch script
* fixed error in captain interface and added a few more parameters to lolo_bringup
* Merge pull request `#11 <https://github.com/smarc-project/lolo_common/issues/11>`_ from smarc-project/lab_test
  Adding stuff for dry test in workshop
* Switched to geopoint instead of stamped in captain
* Fixed namesapces in waypoint action server
* Added waypoint action server to captain launch file
* Fixed the captain interface thruster topic names
* Merge remote-tracking branch 'origin/noetic-devel' into lab_test
* Merge pull request `#10 <https://github.com/smarc-project/lolo_common/issues/10>`_ from NiklasRolleberg/noetic-devel
  Noetic devel
* updated actionserver
* chaged frames for sensors and actuators
* Merge pull request `#9 <https://github.com/smarc-project/lolo_common/issues/9>`_ from NiklasRolleberg/noetic-devel
  updated actionserver
* updated actionserver
* Merge pull request `#8 <https://github.com/smarc-project/lolo_common/issues/8>`_ from Sthune/noetic-devel
  GPS status and linear acceleration added
* Merge pull request `#7 <https://github.com/smarc-project/lolo_common/issues/7>`_ from NiklasRolleberg/noetic-devel
  Noetic devel
* GPS status and linear acceleration added
* changed some printouts and fixed odom publisher
* simple actionserver for going to a waypoint
* Added service calls to convert between lat lon and utm and odometry
* Merge pull request `#5 <https://github.com/smarc-project/lolo_common/issues/5>`_ from NiklasRolleberg/noetic-devel
  Noetic devel
* Added geographic_msgs as a dependency
* Added message_generation to package.xml
* Changed float32 to float64 and changed a few topics
* changed cola2msgs/dvl to smarc_msgs/dvl
* Add 'captain_interface/' from commit '062d3bef310146b2550b8495c565488c35bbcc35'
  git-subtree-dir: captain_interface
  git-subtree-mainline: 328934d3d40f21c3b9ee21062295ba986b2044bc
  git-subtree-split: 062d3bef310146b2550b8495c565488c35bbcc35
* Contributors: Jollerprutt, Niklas, Nils Bore, Sebastian Thuné, niklasrolleberg
