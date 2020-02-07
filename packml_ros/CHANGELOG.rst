^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package packml_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* [Fix] Missing install rule for packml_ros_node `#9 <https://github.com/plusone-robotics/packml/issues/9>`_ 
* Contributors: Charles Costello

1000.3.0 (2020-02-06)
---------------------
* Feature core `#43 <https://github.com/plusone-robotics/packml/issues/43>`_ state observer
* Updated unit test definition to use SendCommand (replaces old Transition)
* Added documentation, changed MapStat items to float (was double), and moved metric constants to enumeration
* Merge branch 'por_development' into feature-core-`#43 <https://github.com/plusone-robotics/packml/issues/43>`_-state-observer
* Moved packml_msgs::utils to packml_ros (where it is used)
* Added service server to increment stats
* Removed StateTransition service, changed SendEvent service param name, extracted command and event guards to separate functions.
* Added invokeStateChange handler, and tied packml_ros more specifically to packml_sm::packml_state_machine_continuous
* Changed signature for incrementing stats and added a SendEvent stub.
* Changed "Transition.srv" to "SendCommand.srv". Added stubs for InvokeStateChange.srv and SendEvent.srv. Changed command enumeration to match omac guidelines. Added public function to allow event processing for state machine. Removed auto signalling of state_complete_event from runStateMethod.
* Contributors: Aaron Wood

1000.2.0 (2019-10-23)
---------------------
* Added stats services `#3 <https://github.com/plusone-robotics/packml/issues/3>`_, `#5 <https://github.com/plusone-robotics/packml/issues/5>`_, `#7 <https://github.com/plusone-robotics/packml/issues/7>`_, `#6 <https://github.com/plusone-robotics/packml/issues/6>`_
* Fix "can't locate node" issue  `#2 <https://github.com/plusone-robotics/packml/issues/2>`_
* Contributors: Austin Deric, Charles Costello, Isaac I.Y. Saito

1000.1.0 (2019-07-05)
---------------------
* Initial release
* Contributors: AustinDeric, Geoffrey Chiou, Joshua Curtis, Joshua Hatzenbuehler, Shaun Edwards
