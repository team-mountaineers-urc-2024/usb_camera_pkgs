# USB Camera Packages

# Image Mod Package

## Description
This Package was designed to contain ROS nodes to do a variety of image modifications. The two existing nodes are an image_convert node that takes in an image of format yuy2_yuv and converts it to bgra, and another node image_flip that will rotate an image 180 degrees.

## Known Limitations
The main limitations are of these packages are in their hard-coded nature. They are not very flexible and are designed with one purpose in mind. They are short though, and easily modifiable to whatever is needed.

# Theora Mux Package

## Description
The Theora Mux Package was a quick fix set in place to allow for use of the Theora image compression algorithm on the robot. One problem with Theora compression is that it requires 3 header packets at the beginning of each stream. When using the ROS2 image compression package with Theora, it only sends these header messages once at the beginning when the topic is first subscribed to. This means that it is impossible to un and resubscribe to the message, and if any of the first 3 messages are dropped on the way (like when using best effort) then you are out of luck and have to restart the node. This node changes that though, by subscribing to the theora topic before anything else and saving the first three messages that it comes across. A service can be called at any later time that will replay these first three messages allowing for the header info to be passed through and the image in subsequent messages to be rendered.

## Known Limitations
The service may need to be called multiple times because of it's best effort nature. This entire node may be able to be moved to the base station side if the ability to determine header message composition (or when an image has changed) is allowed.

The node to decompress the Theora Images is not present in this repo, and is instead in the secondary attached one. The ROS2 Image Transport for Theora decompression is broken in most versions except for (at least at the time) Rolling. The secondary repo is a snapshot of this working copy with the non-humble code ripped out allowing for the basic decompression to function.

Fun fact, Theora Compression is named after the Edison Carter's controller Theora Jones in the Movie Max Headroom: 20 Minutes Into the Future

# Usb Cam

## Description
This is the normal ROS2 usb cam package with the support of simlinks added.

## Known Limitations
This simlink support may be conditional, and could cause issues if used in certain cases. There may be a better way to implement it too.