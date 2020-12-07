This branch is made as an attempt at
creating a QUICK solutin for orienting the
white side of the block facing upwards.

This must be created as a separate function

Incorporation of this function depends on the
total time left after executing both static and
dynamic strategies..


-Need to add another line to PickedPose	that
-sets Flag == 1 if the block is too close to another
 block and return it. This Flag can be used to
 NOT call WhiteSideUp as it would be too risky
 to change pick orientation in that case..