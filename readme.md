## RoboTeam Twente software for use in the RoboCup 2019 SSL-Vision Blackout Technical Challenge.

Currently uses OpenCV 3.4 to detect orange ball using HSV matching, and draws the smallest enclosing circle.
Radius of this circle is used in calculations with focal distance of camera (determined experimentally) to determine distance of ball.
X-position of this circle is used in calculations with the horizontal FOV of the camera (determined experimentally) to determine angle of ball.

Angle and distance (polar coordinates) are converted to x,y (cartesian coordinates).
Circular vector is used for smoothing (moving average filter) and to determine velocity vector of ball. This last feature still needs some work. 