^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pi_trees_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2016-09-25)
------------------
* Tweaked behavior of SimpleActionTask when task is preempted so that the task will continue when control returns to that branch of the tree
* Tweaked comments in SimpleActionTask
* Fixed SimpleActionTask to better handle goal actions that do not succeed (e.g. aborted, preempted, etc.)
* Tweaked done_cb behavior for SimpleActionTask so that a user-defined done_cb is appended to the default_done_cb instead of replacing it
* adding sys import to resolve error in ServiceTask
* Updated dependencies in package.xml files
* Contributors: Patrick Goebel, maxfolley

0.1.1 (2014-08-01)
------------------
* Added initial CHANGELOG.rst files
* Deleted obsolete examples directory from pi_trees_ros
* Updates some comments
* Uncommented feedback_cb for SimpleActionTask
* Did some house cleaning
* Cleanup
* Initial commit
* Contributors: Patrick Goebel
