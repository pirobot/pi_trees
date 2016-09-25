^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pi_trees_lib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2016-09-25)
------------------
* Added error message when number of random weights do not match number of children
* Added weighted_random_example.py
* Added weighted versions of random selector, sequence and iterator
* Added composite_task.py example
* Added try-except to print_tree function
* Replaced print_tree function to fix some pathological cases
* Fixed syntax errors in WaitTask
* Merge branch 'indigo-devel' of https://github.com/pirobot/pi_trees into indigo-devel
  Fixed typo in WaitTask
* Fixed reset_after behavior for Sequence and Selector tasks
* typo for waittask
* Added root level status (BEHAVE.status) to the main loop so that root node status would be colored appropriately
* Removed old parallel tasks (commented out earlier)
* Fixed ParallelOne and ParallelAll tasks to really execute each child task on every iteration
* Fixed reset_after status for Sequence
* Added status colors to root node for DOT graph
* Added parallelogram shape for ParallelOne and ParallelAll tasks
* Added task_setup.py for use with patrol_tree.py example
* Added root node status color to display
* Updated the patrol_tree.py sample script to check battery level only when not recharging
* Changed rate from 10 to 5 for patrol_tree.launch example
* Added patrol example to pi_trees_lib examples
* Modified print_dot_tree() function to write dotfile only if tree has changed since last write
* Fixed initial value of last_dot_tree from None to the empty string
* Added pi_trees_viewer suppot to pi_trees_lib
* Added reset_after option to other composite tasks
* Added a few more task decorators and the reset_after option (defaults to False) to reset any task after execution if set True
* Added ASCII symbols option to graph representation and improved the print_dot_tree() output
* Added UntilFail and until_fail() decorators
* Fixed bug in task_not() decorator and added invert alias
* Fixed bug in TaskNot decorator and add Invert alias
* Added limit() decorator
* Added 1 to the displayed loop count for the loop() decorator to be consistent with the Limit decorator
* Added Limit decorator
* Added random versions of Sequence, Selector and Iterator
* Added new CallbackTask to pi_trees_lib
* Added new CallbackTask to pi_trees_lib
* Added catkin_install_python line to CMakeLists.txt
* Renamed scripts directory back to examples and parallel_tasks.py to parallel_example.py
* Small reformatting of comment
* Renamed examples directory to scripts and renamed counting.py example to parallel_tasks.py
* Removed dependency on pygraph and pygraphviz for now
* Updated dependencies in package.xml files
* Contributors: Jack000, Patrick Goebel

0.1.1 (2014-08-01)
------------------
* Added initial CHANGELOG.rst files
* Removed print_dot_tree statement from counting.py example
* Updates some comments
* Updated counting.py example
* Did some house cleaning
* Cleanup
* Initial commit
* Contributors: Patrick Goebel
