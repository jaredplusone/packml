^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package packml_sm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1000.3.0 (2020-02-06)
---------------------
* Added documentation, changed MapStat items to float (was double), and moved metric constants to enumeration
* Merge branch 'por_development' into feature-core-`#43 <https://github.com/plusone-robotics/packml/issues/43>`_-state-observer
* Moved packml_msgs to separate repository. Updated StatesEnum to match OMAC Implementation v1.00
* Fixed test to call state complete between states. Added service definitions for GetState, IncrementStat, and StateTransition
* Added invokeStateChange handler, and tied packml_ros more specifically to packml_sm::packml_state_machine_continuous
* Changed signature for incrementing stats and added a SendEvent stub.
* Changed "Transition.srv" to "SendCommand.srv". Added stubs for InvokeStateChange.srv and SendEvent.srv. Changed command enumeration to match omac guidelines. Added public function to allow event processing for state machine. Removed auto signalling of state_complete_event from runStateMethod.
*  `#8 <https://github.com/plusone-robotics/packml/issues/8>`_ from `plusone-robotics/feature-core-#43 <https://github.com/plusone-robotics/feature-core-/issues/43>`_-state-observer
* Contributors: Aaron Wood

1000.2.0 (2019-10-23)
---------------------
* Interval stats `#7 <https://github.com/plusone-robotics/packml/issues/7>`_
* Added load stats service call `#3 <https://github.com/plusone-robotics/packml/issues/3>`_, `#5 <https://github.com/plusone-robotics/packml/issues/5>`_
* Contributors: Charles Costello

1000.1.0 (2019-07-05)
---------------------
* Initial release
* Contributors: AustinDeric, Geoffrey Chiou, Isaac I.Y. Saito, Joshua Curtis, Joshua Hatzenbuehler, Shaun Edwards
