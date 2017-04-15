# Operation and configuration of Victoria to win the RoboMagellan contest.

There are some setup procedures that need to be done and some tools that are available to help.

**INSERT MORE SETUP INFO HERE**

----
# **Configuring the cone detector**
When the **cone_detector.launch** is invoked, there are a couple of tools you can use to control and setup Victoria.

* Running **rqt** and using the dynamic reconfiguration plugin, if you click on the "cone_detector" item in the list on the left, you should see something like

![Dynamic reconfiguration for the cone detector](/doc/rqt_dynamic_configure_cone_detector.png)

There are two cone detector filters with their own set of parameters, and a few parameters that are common to the whole cone detector process.

The two filter parameter names are prefixed with "a" and "b". E.g., alow_hue_, ahigh_hue_. The two filter results are "OR"-ed together. 
You could use the a-filter to pick up the low hue range of the cone and the b-filter to pick up the high hue range, since the orange cone can have a color that spans both ends of the hue scale.
Or, you could use the a-filter to pick up most of the cone and the b-filter to pick up the specular highlights which are out of range of the a-filter, being nearly colorless.

Besides the "a" and "b" filters, after those two filter results are "OR"-ed together, additional parameters apply.

* **min_cone_area_** and **max_cone_area_**

This is a filter to prevent colored balls, leaves, sides of barns and such from being acceptable candidates. 
The cone detector computes a poloygon to encompass each connected region of pixels from the joined filters and considers each of
the resulting polygons a potential cone candidate. One test made is whether the area of that polygon seems reasonable. 
The image was downsampled to 320 by 240, so it's independent of the original picture resolution. 
If the polygon doesn't have an enclosed area in square pixels within the closed interval defined by these two values, the polygon
is rejected as not a cone.

* **max_aspect_ratio_**

The cone detector finds the horizontal centerline of the polynomial enclosing the candidate cone in the image and then finds the leftmost
x coordinate both above and below the centerline, and the rightmost x points above and below. It then compares the length of a line drawn
between the leftmost and rightmost x coordinates above the centerline (width of top of the polynomial) with the length of a line
drawn between the leftmost and rightmost x coordinates below the centerline (width of bottom of polynomial). If the upper line
length is more that **max_aspect_ratio_** times the bottom line length, the polygon is rejected as not pointy enough.

* **poly_epsilon_**

When the cone detector attempts to surround a region of connected pixels with a polygon, it wants to form a simple polygon, ignoring minor
jaggies along the edge. This parameter indicates how many pixels away from an ideal line a point can be and still be considered
on a candidate polygon line. Larger values will result in blobs being represented by polygons of fewer sides, but possibly over
simplifying the polygon. Smaller values will result in minor aberrations in the blob causing a too-complex polygon to be formed.
The resulting polygon is examined for how many sides it used to surround the blob. If the polygon has less than three sides or more than ten,
it is rejected as being too simple or too complex a shape.

* **erode_kernel_size_**

After the two filters are "OR"-ed together, a two step process is taken to try to get rid of noisey pixels. The two steps attempt
to expand black areas (things not considered possible pixels in the cone) to cover over small bits of white areas (things considered
as possible pixels in the cone), and then shrink the white areas (or I've got white and black backwards—doesn't matter). This is done
by forming an "n by n" ellipse, an image processing "kernel" that is moved over every pixel in the image and performing an erode 
and dilate process over the pixels encompassed by the kernel. This value determines the size of that kernel. Large values will
remove useful distinguishing details from the potential pixels in the cone, and small values will leave speckles in the picture.
