

## 7942IEEE ROBOTICS AND AUTOMATION LETTERS, VOL. 7, NO. 3, JULY 2022
P-AgBot: In-Row & Under-Canopy Agricultural
Robot for Monitoring and Physical Sampling
Kitae Kim, Graduate Student Member, IEEE, Aarya Deb, and David J. Cappelleri
Abstract—In this work, we present a novel agricultural robot
called the Purdue AgBot or P-AgBot that has been designed for
in-row and under canopy crop monitoring and physical sampling.
We suggest approaches to autonomous navigation, crop monitor-
ing, and crop sampling that can be applied in crop rows and under
canopies for different agricultural environments. Each monitoring
approach was designed to extract key morphological characteris-
tics of the crops. The proposed approaches of P-AgBot have been
experimentally verified not only in simulation but also with real
corn and sorghum crops. Crop heights and stalk diameters are able
to be estimated in real-time with less than 10% error. Vision-based
detection of leaf samples was implemented and physical sampling
is accomplished with a more than 80% success rate.
Index Terms—Field robots, agricultural automation.
## I. INTRODUCTION
## P
RECISION agriculture uses technology to acquire and
analyze data from farms in order to monitor the state of
agricultural crops. Traditionally, crop monitoring and assess-
ment has been accomplished through costly, labor-intensive, and
time-consuming processes of crop scouting, manual sampling,
and documenting the state of the farm. Recently, Internet of
Things (IoT) technology and agricultural robotics have emerged
as a viable approach to implement and create new precision
agriculture practices [1]–[5]. The data obtained from agricul-
tural IoT sensors and robots (drones) can be used to predict
and control the state of the farm efficiently. In addition, these
automated measurement systems assist farmers in managing
crops and increasing crop production.
For crop monitoring, a variety of Unmanned Aerial Vehicles
(UAVs) [6]–[9] and Unmanned Ground Vehicles (UGVs) [9]–
[11] are currently utilized with autonomous navigation. UAVs
and UGVs showed great performances in environments where
Global Navigation Satellite System (GNSS) signals are avail-
able. Potenaet al.proposed a novel map registration algo-
rithm by using both UAVs and UGVs and presented align-
ments with heterogeneous 3D maps in [9]. Although the results
Manuscript received 24 February 2022; accepted 14 June 2022. Date of
publication 29 June 2022; date of current version 6 July 2022. This letter
was recommended for publication by Associate Editor Z. Wang and Editor H.
Moon upon evaluation of the reviewers’ comments. This work was supported by
IoT4Ag Engineering Research Center, the National Science Foundation (NSF)
under the NSF Cooperative Agreement EEC-1941529.(Corresponding author:
## David J. Cappelleri.)
The authors are with the School of Mechanical Engineering, Purdue
University, West Lafayette, IN 47907 USA (e-mail: kim3686@purdue.edu;
deb8@purdue.edu; dcappell@purdue.edu).
This  letter  has  supplementary  downloadable  material  available  at
https://doi.org/10.1109/LRA.2022.3187275, provided by the authors.
Digital Object Identifier 10.1109/LRA.2022.3187275
Fig. 1.  P-AgBot and its components. 1. Tracking camera, 2. 3D LiDAR sensor,
- Two-finger style gripper, 4. RGB-D camera, 5. Six degree-of-freedom robotic
arm, 6. Servo motor, 7. 3D printed linkage with nichrome wire end-effector, and
- 2D LiDAR sensor.
in [9] showed high performances, the relative displacement and
rotation provided by GNSS were used to obtain the results
in GNSS-friendly environments. Zhanget al.proposed high
precision control and corn stand counting algorithms for an
autonomous ground robot and the algorithms performed with
high accuracy in [10]. However, the results were shown to only
work in an environment with high GNSS capability. However,
when operating inside rows of crops and/or under the canopy of
crops, GNSS signals are not reliable or non-existent. Therefore,
for agricultural robots to operate in-row and under the canopy
of crops, alternative platforms [12]–[18] and approaches to
estimate the vehicle pose and precisely navigate are required.
Additionally, in instances that require physical samples of crops
to be collected in challenging hard-to-reach areas, new technol-
ogy must be developed.
In this paper, we present a novel agricultural robot platform
called the Purdue AgBot or P-AgBot (Fig. 1) that has been
designed for in-row and under-canopy crop monitoring, assess-
ment, and physical sampling. This integrated robotic system is
designed for operation in row crops, such as corn and sorghum.
2377-3766 © 2022 IEEE. Personal use is permitted, but republication/redistribution requires IEEE permission.
See https://www.ieee.org/publications/rights/index.html for more information.
Authorized licensed use limited to: Purdue University. Downloaded on April 24,2025 at 17:02:41 UTC from IEEE Xplore.  Restrictions apply.

KIMet al.: P-AgBOT: IN-ROW & UNDER-CANOPY AGRICULTURAL ROBOT FOR MONITORING AND PHYSICAL SAMPLING7943
P-AgBot can automate multiple repetitive tasks accurately at
the same time and overcome the challenges present in harsh
agricultural environments. We demonstrate novel approaches
to autonomous navigation, monitoring, and crop sampling in
both simulation and with real corn and sorghum crops. First,
we provide an overview of the P-AgBot design in Section II.
Then the localization & autonomous navigation approach and
crop monitoring algorithms are presented in Sections III and IV,
respectively. Our approach to physical sampling is discussed in
Section V. Section VI details our experimental results with the
P-AgBot. Finally, in Section VII, we conclude the paper.
## II. D
## ESIGNOVERVIEW
The agricultural environment produces some unique chal-
lenges for autonomous robots. In the case of row-crops, farmers
utilize narrow spacing between the rows (typically, from 18
## 
to
## 30
## 
) in order to control weed spread and minimize the compe-
tition between plants for essential elements such as sunlight,
water, and nutrients. However, this narrow spacing provides
distinct geometric constraints for autonomous robots navigating
between the crop rows. Once mature, crops can become very
dense. For example, corn and sorghum can grow up to 8 feet tall
with their leaves creating a canopy that covers the most of, if not
all the space between the rows. Consequently, GNSS receivers
on autonomous agricultural robots navigating under the canopy
in the crop rows are not able to collect any/or reliable signals.
Overhanging leaves, weeds, or downed crops provide obstacles
that must be traversed or avoided. If physical samples of crops
are required, the deployed sampling system on the robot must
be versatile enough to perceive and sample the crops at various
stages (heights) during the growing cycle.
To meet the previously described challenges, we created the
P-AgBot, as shown in Fig. 1. It is built off of a commer-
cial Jackal unmanned ground vehicle platform from Clearpath
Robotics [19]. This is a weatherproof, all-terrain platform with
a high torque 4×4 drivetrain for outdoor operations in rugged
environments. It has an onboard CPU (Intel Core i5 4570 T) for
motor control, data processing, and navigation. The P-AgBot
consists of a unique combination and configuration of integrated
sensors. At the front of P-AgBot, a two-dimensional (2D) light
detection and ranging (LiDAR) sensor (ROBOTIS LDS-01) is
mounted for localization and autonomous navigation. The 2D
LiDARhasa360-degreeangularrangeandthelaserscanreaches
a maximum of 3,500mmat 5 Hz. At the back of P-AgBot, a
tracking camera (Intel RealSense T265) and a 3D LiDAR sensor
(Ouster OS1-64) are mounted. The tracking camera publishes
visual-inertial odometry (VIO) at 200 Hz and the VIO tracks
its own orientation and position in six degrees of freedom.
The 3D LiDAR is mounted vertically in order to capture the
entirety of crops at various heights for mapping and capturing
morphological measurements. For robot control and processing
sensor data, the Robot Operating System (ROS) is utilized.
The P-AgBot also has an integrated six degree-of-freedom
robotic arm, the Kinova Gen3 Lite [20]. This is a lightweight
manipulator and that is capable of handling payloads up to
0.5 kg. It is powered directly through the 24 V power supply of
Jackal with an average power consumption of 20 W. We present
a novel end-effector design consisting of a servo-operated 3D-
printed linkage integrated with a nichrome wire, which works in
conjunction with the 2-finger style gripper of the Kinova arm to
perform physical sampling of leaves. With a maximum reach
of 1munder full extension, we use the arm to sample and
manipulate crop leaves. An RGB-D camera (Intel RealSense
D435) is mounted to the end-effector link of the arm to provide
vision-based guidance to the arm by detecting the position and
distance of the desired leaf during the physical sampling process.
## III. L
## OCALIZATION&AUTONOMOUSNAVIGATIONAPPROACH
A number of components are needed for successful au-
tonomous operation of the P-AgBot in between rows and under
the canopy of crops. In terms of autonomous navigation, the
robot needs to not only estimate its position precisely but also
traverse to goal points without collision. GNSS-based localiza-
tion methods are not applicable here due to large multipath errors
resulting from unreliable (if any) signals when navigating under
the crop canopy. Instead of the GNSS-based localization, we
use adaptive Monte Carlo localization (AMCL) [21] with an
extended Kalman filter (EKF) [22]. The AMCL algorithm is
a probabilistic localization routine that utilizes a particle filter
algorithm in a pre-known occupancy grid map while comparing
results with 2D laser scan data. The P-AgBot is teleoperated
initially through the crop rows to establish this occupancy grid
map. The occupancy grid map is made by a ROS opensource
package GMapping  [23]. After the initial pose estimation,
AMCL recursively generates new samples to predict the robot’s
position during autonomous traversing. One of the approaches to
improve the performance of localization is to predict the motion
of the robot with a small drift. Thus, the EKF fuses two kinds of
odometry information, wheel odometry, and VIO, as a means to
improve motion prediction.
In terms of autonomous navigation in the presence of clut-
tered crop canopies, conventional path planning algorithms are
inappropriate to be applied. When the robot detects weeds and
hanging leaves that are not in the pre-defined occupancy map,
traditional methods consider these elements as rigid obstacles
thatcannotbepassed.However,thisisnotthecaseforin-rowand
under-canopy navigation. Therefore, we present a 2D LiDAR-
based method that performs reliable autonomous navigation for
this situation.
The autonomous navigation task may seem similar to imple-
menting a simple line following routing. However, it is more
challenging than that due to the cluttered nature of agricultural
environments and the lack of lane markings. Thus, we utilize
the 360-degree range 2D LiDAR to understand the robot’s
surrounding area. The concept of the autonomous navigation
system is shown in Fig. 2. The approach uses 2D laser scans
which satisfy particular conditions at each timestamp around
P-AgBot to sense crops while it is traversing between goal
points. Laser scans from crops near the robot are reflected with
high intensities. The P-AgBot only extracts high-intensity laser
scans at set angles on either side of a row that it drives. With
this scan information, the robot identifies the plants at risk of
Authorized licensed use limited to: Purdue University. Downloaded on April 24,2025 at 17:02:41 UTC from IEEE Xplore.  Restrictions apply.

## 7944IEEE ROBOTICS AND AUTOMATION LETTERS, VOL. 7, NO. 3, JULY 2022
Fig. 2.  Autonomous under-canopy navigation: 2D LiDAR scans are used to
identify and sense distances to the left (d
l
) and right (d
r
) of P-AgBot to crops
for collision avoidance when moving both forward (red) and backward (green).
collision. Taking into account the scan information at specific
angular ranges, the robot computes minimum distances between
the robot and crops on the left and right side of the robot,d
l
and
d
r
, respectively. The robot then determines linear and angular
velocities that minimize the difference betweend
l
andd
r
in
order to keep the robot navigating in the middle of row. Different
angular ranges of the scan data is utilized when the robot travels
forward and reverse. This allows for autonomous navigation to
way points in both directions. This avoids the robot needing to
turn around inside the crop row, where there is little room to do
so, to navigate to a way point that is behind it.
The proposed scheme shows multiple advantages. First,
this approach prevents malfunctions from weeds, overhanging
leaves, and downed corn trees in the pathway. Since crops are
planted in the rows, we assume that obstacles encountered on the
pathway between the rows are traversable obstacles including
downed corn plants which have passable diameters, listed as
ground truth values in Table IV. This is why we primarily focus
on angular ranges of the 2D LiDAR data on the robot’s sides
for navigation rather than angular ranges corresponding to the
front or back of the robot. Even if traversable obstacles are close
to crop plants, this method gains merits. In this case, these
traversable obstacles are in the range of the 2D laser scans.
Therefore, the navigation system deals with the hanging leaves
and weeds and generates trajectories that avoid collisions from
them. Another advantage of this approach is robustness in di-
verse climate conditions. Not only is the navigation performance
not affected by the lighting conditions, but it also can perform
well in windy conditions. In moderate wind, the crop leaves
shake but the crop stems, which the approach primarily uses,
remainstableordonotshakeconsiderablytoaffectperformance.
## IV. C
## ROPMONITORING
Manually obtained morphological measurements are rou-
tinely used to assess the status and health of crops throughout the
growing season as well as in plant phenotyping studies. Thus, it
is desirable for the P-AgBot to be able to autonomously capture
these types of measurements as it traverses the field. Here, we
propose new schemes to monitor two kinds of indicators, the
crop height, and stalk radius, to assist in these crop monitoring
studies. Since our proposed crop monitoring methods are based
Fig. 3.  Stalk radius estimation. (a) Cluster laser scans (p
i
) at periodic times-
tampst
Tn:T(n+1)−1
. (b) The two farthest points fit to major axis of an ellipse
to estimate the radius and center point of thejth crop stalk. (c) The final radius
## (R
k
) and the center point (X
k
) estimated based on the average of consecutive
measurements.
onLiDARsensors,itcanperformconsistentlyinvariouslighting
conditions.
## A. Crop Height
The vertically-mounted 3D LiDAR on P-AgBot is used to
estimate crop height. When the robot is traversing under clut-
tered crops, on-board cameras are not effective in estimating
crop height. However, the high-resolution and high-accuracy
3D LiDAR enables the effective capture of the entire crop shape.
Our method clusters the obtained data in to rows based on the
position of the clusters with respect to the robot. Once clustered
into distinct rows, the data is analyzed to determine the points
that are located at the highest level from the ground for each crop
in every row in real-time. To compensate for windy conditions
when crops may be moving, our method utilizes mean values of
the height estimations from adjacent times in order to determine
the final crop heights value. Additionally, with the large scanning
range of the 3D LiDAR, it is possible to estimate the heights of
several rows to the left and right of the row being traversed at
the same time.
## B. Stalk Radius
We need to consider several characteristics of sensors and
crops in order to accurately estimate the stalk radius. The 3D
LiDAR has a minimum scanning range, which limits collect-
ing reliable data close to the sensor. Therefore, it will not be
effective to use it to estimate the stalk radius. However, the
2D LiDAR with a shorter minimum scan range, that is also
used for navigation (Section III) can be utilized to collect the
stalk data needed to estimate its radius. The 2D LiDAR is also
mounted lower on the Jackal than the 3D LiDAR. This is an
ideal vantage point to obtain stalk radius data, as the stalk is
typically free from clutter at the bottom of the crops and sturdier
than at higher locations, which are more susceptible to wind
disturbances. The other important crop characteristic to be dealt
with is the thin-nature of the crop stalk (typically on the order of
10-20mm). The thinner the stalk radius is, the smaller number
of scans are reflected by the stalks. This limited scan data makes
it difficult to estimate the radius accurately. To overcome this
issue, we designed the stalk radius estimation scheme, shown
schematically in Fig. 3 and described in Algorithm 1.
Authorized licensed use limited to: Purdue University. Downloaded on April 24,2025 at 17:02:41 UTC from IEEE Xplore.  Restrictions apply.

KIMet al.: P-AgBOT: IN-ROW & UNDER-CANOPY AGRICULTURAL ROBOT FOR MONITORING AND PHYSICAL SAMPLING7945
Algorithm 1 is used to estimate the centerX
k
and radiusR
k
of the crop stalk. It uses a density-based spatial clustering of
applications with noise (DBSCAN) [24] technique, which is a
popular clustering method. Laser scans at several timestamps
t
Tn:T(n+1)−1
with a time periodTare clustered by DBSCAN.
With each cluster (C
t
Tn:T(n+1)−1
## ,j
), the two farthest points based
on their euclidean distance from each other (c
## 1,j
andc
## 2,j
), are
identified. These are used as the end points of the major axis
of an ellipse to fit the data to. From this fit, the center point
## (x
t
Tn:T(n+1)−1
## ,j
), and radius (r
t
Tn:T(n+1)−1
## ,j
) of the cluster are
estimated. As the final step, all the center points (x
t
Tn:T(n+1)−1
## ,j
## )
and corresponding radius estimates (r
t
Tn:T(n+1)−1
## ,j
) that are
computed in every periodTare clustered and the final center
position (X
k
) and (ellipse major axis) radius estimate (R
k
## )
ofkth crop stalk are computed as the output of Algorithm 1.
Additionally, the P-AgBot traverses rows on both sides of crops
to observe the stalks on either side to increase the amount of data
points used to estimate the radius. This strategy greatly improves
the performance of estimating the stalk radius when compared
to when data is only obtained from one side of the stalk.
## V. P
## HYSICALSAMPLING
A. End-Effector Operation
The gripper on the P-AgBot arm is used to grasp the desired
leaf for physical sampling and the nichrome wire end-effector
is capable of cleanly cutting the leaf from the stalk of the
corn plant. This key components for this design are shown in
Fig. 1. The nichrome wire is housed in a 3D-printed linkage,
and mounted to the servo motor shaft with proper insulation.
A micro-controller (Arduino Uno) is used to send a signal to a
relay module for energizing the circuit intermittently, and also
control the angle of the servo motor. The serial communication
between the Ardunio and Jackal is established through ROS. The
nichrome wire is connected directly to the 12 V power supply
rail of Jackal with a 0.5Ωpower resistor in series to keep the
current draw under the recommended 10 A rating. The high
resistivity of the nichrome wire makes it very suitable for this
application as it heats up rapidly when current is passed through
Fig. 4.  Leaf detection algorithm stages.
the circuit, and it cools down equally rapidly upon removal of
the power source. To execute a physical sampling operation, the
arm follows the trajectory required to maintain a correct pose
for enabling the fingers of the gripper to grasp the target leaf
close to its petiole (where it connects to stalk). The relay is then
triggered to energize the circuit and heat up the nichrome wire.
Finally, the servo motor is used to swing the wire and cut the
leaf through localized heating. After completion of the sampling
operation, the sliced leaf is manipulated again using the arm and
placed in a storage box.
B. Vision-Guided Leaf Detection
A leaf detection algorithm has been developed for vision-
guided leaf sampling. The end-effector is guided by this al-
gorithm using image processing techniques and positioned ac-
cordingly for the gripper to grasp the detected leaf. It uses the
wrist-mounted RGB-D camera to detect the crop leaves using
OpenCV in real time to extract the positional information. The
camera is oriented such that the end-effector components are not
in its field of vision. Some image preprocessing is performed in
the RGB frame to improve the performance of this algorithm
and the data from the depth frame is utilized simultaneously to
increase robustness. The steps for the leaf detection routine are
shown in Fig. 4 and described below.
1) Image Filtering:The RGB stream from the camera is used
as the input frame (Fig. 4(a)). A Gaussian filter is added for
de-noising. The frame is transformed to HSV (hue, saturation,
and value) color space so that only the details of the object of
Authorized licensed use limited to: Purdue University. Downloaded on April 24,2025 at 17:02:41 UTC from IEEE Xplore.  Restrictions apply.

## 7946IEEE ROBOTICS AND AUTOMATION LETTERS, VOL. 7, NO. 3, JULY 2022
interest are preserved (Fig. 4(b)). The filtering parameters are
selected while accounting for the color properties of the leaves
and lighting conditions of the environment (Fig. 4(c)). This
HSV thresholding method ensures the segmentation of leaves
and removal of the background clutter. This segmented image
(Fig. 4(d)) is then used to perform Canny edge detection. The
removal of background by the HSV thresholding mask in the
previous step aids in the process of edge detection (Fig. 4(e)).
2) Contour Detection:The contours of each detected leaf are
extracted in this step. They replicate the outlines of the leaves
generated by the canny edge detector, but this step provides
more flexibility by storing the shape, area, and position of the
individual contours. The de-noising step in the preprocessing
stage is unable to filter out all of the image noise or imperfections
caused by lens flare. Therefore, the unwanted contours detected
due to the presence of residual noise are removed by applying a
thresholding filter based on minimum contour area. After com-
pletion of this step, the generated contours represent each of the
detected leaves respectively. The contour information is stored
in the form of an indexed array, thus allowing the extraction
of information correlating to a particular leaf. Additionally, the
centers of the contours and their associated distance from the
camera are calculated by combining the detection results with
the depth frame. Their coordinates are stored as the output of
this routine (Fig. 4(f)).
3) End-Effector Localization:The coordinates of the centers
of the contours (X,Y,Z) are stored with respect to the image
frame which is then aligned to the global frame. The X axis
is coming out of plane in front of the camera, Y axis is to
the left of the camera, and the Z axis is above the camera.
The X distance is calculated directly from the depth frame of
the RGB-D camera, while the Y and Z distances are approxi-
mated from the center point of the image frame with the help
of camera calibration at each known depth value. Since the
position of the camera is known relative to the end-effector,
the transformed coordinates (X
## 
## ,Y
## 
## ,Z
## 
) are calculated in the
final step. These coordinates are relayed to the arm using the
Kinova Kortex API in the Cartesian frame which localizes
the end-effector to a position which enables grasping of the leaf.
## VI. E
## XPERIMENTALRESULTS
Experiments were designed to evaluate the P-AgBot’s per-
formance. They were conducted in several agricultural envi-
ronments and the autonomous navigation, crop monitoring, and
crop sampling capabilities of the P-AgBot were evaluated. Two
simulated testing environments were created with rows of corn
and rows of sorghum in a ROS Gazebo simulation environment.
The other two testing environments are with physical corn
and sorghum plants from or inside a greenhouse. Images and
characteristics of each environment are provided in Fig. 5 and
Table I, respectively.
## A. Localization & Navigation Results
The performance of the autonomous navigation module was
validated in three different environments (Fig. 5(a), (b), and (d)).
Fig. 5.  Images of the simulated ((a)–(b)) and experimental ((c)–(d)) testing
environments.
## TABLE I
## S
## IMULATION ANDEXPERIMENTALENVIRONMENTS
## TABLE II
## A
## UTONOMOUSNAVIGATIONRESULTS:DEVIATIONSFROMNOMINALTARGET
## TRAJECTORY
Initially, the P-AgBot was teleoperated in the respective test-
ing environments to build an occupancy map, as described in
Section III. Waypoints in the map were then set up for the
P-AgBot to drive forward to a goal location where a plant of
interest exists and then for it to drive backwards back to its
starting location.
The navigation results were qualitatively and quantitatively
evaluated, as shown in Fig. 6 and Table II. We compared the
AMCL poses with the nominal target trajectory. The AMCL
poses of the P-AgBot in Fig. 6 demonstrate that the the poses of
P-AgBot were not lost by AMCL localizer in the pre-defined
occupancy maps. They also represent the forward (red) and
backwards (green) trajectory of the P-AgBot in comparison
to the nominal target trajectory (blue) that is in the middle of
Authorized licensed use limited to: Purdue University. Downloaded on April 24,2025 at 17:02:41 UTC from IEEE Xplore.  Restrictions apply.

KIMet al.: P-AgBOT: IN-ROW & UNDER-CANOPY AGRICULTURAL ROBOT FOR MONITORING AND PHYSICAL SAMPLING7947
Fig. 6.  Autonomous navigation results for (a) simulated and (b) greenhouse
sorghum. Black dots denote crop stalk placement; nominal target trajectories are
in blue. The AMCL poses are plotted with red markers when driving forward
and with green markers when driving backward.
the crop row. In Table II, deviations from the nominal target
trajectory along with acceptable deviations for driving without
collision are listed. The acceptable deviation of each crop field
can be computed by considering the physical size of P-AgBot
and the row spacing of the crops. When the trajectory deviations
are smaller than the acceptable deviations, the robot is driving
safely.
## B. Crop Monitoring Results
During autonomous driving, both the height and stalk radius
were computed simultaneously in real-time. While crop heights
were estimated in all four experimental environments, only the
sim-corn and greenhouse-corn environments were used to esti-
mate the stalk radius. This is because the stalks of the sorghum in
the greenhouse were obstructed by their pots and/or overhanging
leaves, which would not be the case in the actual field when
measures would be required.
To estimate stalk radius, P-AgBot traversed three different
rows in order to get an acceptable amount of data from either
side of the stalk, as described in Section IV. Quantitative errors
between manually-measured ground truths and the estimated
results for heights and stalk radii are summarized in Tables III
and IV, respectively. The results in Table III show smaller or
comparable errors when compared to crop height measurement
estimates from a laser-scanner mounted to a UAV in [25].
Ground truths in Table IV represent the length of the major
axis of each crop stalk. The average errors in both tables are all
10% or less for both crops considered in all the agricultural test
environments, with most in the 5% average error range. The es-
timates in Table IV achieve smaller errors than the measurement
results of maize stalk diameters calculated with a phenotyping
robot and RGB-D camera images in [26].
## TABLE III
## C
## ROPHEIGHTESTIMATES
## TABLE IV
## S
## TALKRADIUSESTIMATES
## C. Physical Sampling Results
The vision-guided leaf detection algorithm along with the
physicalleafsamplingsystemwereevaluatedusingtwodifferent
experiments:1)leafpositionestimationonanartificialcornplant
and grasping; and 2) physical sampling of sorghum leaves inside
the greenhouse. The same leaf detection algorithm was used for
both the corn and sorghum leaves.
Thirty total trials were conducted where the leaf detection
algorithm was usedtodeterminethecenter of tenselectedleaves.
## Thegeneratedcontoursarethenusedasgraspinglocationsonthe
leaf for the manipulator; twenty seven of them were successful.
For the successful trials, a sample real-time detection image
from the on-board RGB-D camera is shown in Fig. 7(a). The
absolute errors for estimation of the leaf position are plotted for
each leaf in Fig. 7(b) with colored bars. The lowest errors were
reported along the X axis since it is directly calculated from the
depth frame. The errors along the Y and Z axis were slightly
higher, especially for the leaves in the peripheral vision of the
camera. A likely source of this error is from camera calibration
error or lens distortion. For each trial, images of the leaf were
subsequently examined manually to determine the allowable
absolute error tolerances along each axis. These are plotted with
dotted lines for each trial in Fig. 7(b). The error tolerance range
for the Z axis can be explained due to the 0.10mwide gripper
opening combined with the vertically angled orientation of some
leaves. The error tolerance for the Y axis was higher than that
of Z axis due to the side-to-side orientation of the leaves on the
Authorized licensed use limited to: Purdue University. Downloaded on April 24,2025 at 17:02:41 UTC from IEEE Xplore.  Restrictions apply.

## 7948IEEE ROBOTICS AND AUTOMATION LETTERS, VOL. 7, NO. 3, JULY 2022
Fig. 7.  Leaf detection results. (a) Leaf contours and center points detected on
an artificial corn plant; (b) Colored bar: Absolute errors of leaf position relative
to ground truth (measured from the base of the arm). Error bar: Deviation of
absolute errors for all of the trails. Dotted line: Tolerances for absolute errors
along each axis. Average tolerance errors for X, Y, Z axes are 0.05m,0.11m,
and 0.89m, respectively.
periphery, providing the gripper a wider range of points that lie
on the leaf to grasp. The three unsuccessful trials failed due to
instances of overlapping leaves being detected as a single leaf,
yielding an unrealistic contours and center coordinate positions.
It is likely that these trials would not have failed on a real corn
plant with uncluttered leaves and better lighting conditions than
the cluttered nature of the artificial corn plant that was used.
However, for the twenty seven successful trails, the errors for
leaf position estimation were within the tolerable ranges and
the gripper was able to successfully grasp the leaf. Additionally,
this algorithm can also be used to generate a decision matrix for
picking the best candidate leaf of a plant to sample, based on
error values and success rates from this study. For example, since
we have achieved a higher success rate of physical sampling on
leaves near the center and oriented horizontally, the coordinates
of the centers of the contours generated by this algorithm can
be utilized to isolate a peripheral leaf that is not cluttered
by neighboring leaves to maximize the sampling success rate.
Therefore, the optimal leaf for physical sampling can be picked
by taking this heuristic approach into consideration.
Fig. 8.  P-AgBot physical sampling: (a) Driving autonomously inside sorghum
rows and stopped at the desired goal point for leaf grasping; (b) Gripper
successfully grasping a leaf; (c) End-effector successfully cutting the leaf.
Once a leaf is successfully grasped, the nichrome-wire end-
effector is able to cut it. Video screenshots of a successful
physical leaf sampling operation are shown in Fig. 8. During
the course of autonomous driving, the P-AgBot was able to stop
adesiredgoal point infront of thesorghum plant plant of interest.
Once stopped, the end-effector was used to successfully cut the
leaf from the stalk.
## VII. C
## ONCLUSION
In this paper, the P-AgBot, an agricultural robot platform
for crop sampling and monitoring is presented. The proposed
robotic system is designed to operate in rows and under crop
canopies. With our novel autonomous navigation system, the
P-AgBot can traverse in narrow rows where GNSS signals
cannot be utilized. The autonomous navigation results showed
slight differences between the nominal target trajectories but
the small errors did not significantly affect crop safety. Rather,
the damage to hanging leaves is minimized with our proposed
scheme. The height estimation scheme performed effectively to
estimate the crop heights in multiple rows simultaneously. Fur-
thermore,despitethelackofstalkscandataduetothethin-nature
of corn stalks, our method was able to accurately estimate the
stalk diameters. The P-AgBot has also been demonstrated to be
able to autonomously physically sample a crop of interest using
its vision-guided control framework.
The usability of P-AgBot is impacted by the existence and
quality of the pre-defined map. It means that our proposed
schemes are available in the sites where the map exists. To
broaden the applications, our future work will be on developing
a robust simultaneous localization and mapping solution ac-
cording to the characteristics of agricultural environments, and
combining the solution with the existing methods. Additionally,
a neural network trained on the images of leaves and stalks of
Authorized licensed use limited to: Purdue University. Downloaded on April 24,2025 at 17:02:41 UTC from IEEE Xplore.  Restrictions apply.

KIMet al.: P-AgBOT: IN-ROW & UNDER-CANOPY AGRICULTURAL ROBOT FOR MONITORING AND PHYSICAL SAMPLING7949
agricultural crops will be implemented to make the vision-based
leaf detection algorithm more robust and decrease the estimation
error for leaf position.
## A
## CKNOWLEDGMENT
We would like to thank Prof. Mitch Tuinstra at Purdue Uni-
versity for providing greenhouse access for our experiments and
Brian Huang for P-AgBot sensor mounting.
## R
## EFERENCES
[1] M. Ayaz, M. Ammad-Uddin, Z. Sharif, A. Mansour, and E.-H. M.
Aggoune, “Internet-of-things (IoT)-based smart agriculture: Toward mak-
ing the fields talk,”IEEE Access, vol. 7, pp. 129551–129583, 2019.
[2] S. Wolfert, L. Ge, C. Verdouw, and M.-J. Bogaardt, “Big data in smart
farming–a review,”Agricultural Syst., vol. 153, pp. 69–80, 2017.
[3] M. S. Farooq, S. Riaz, A. Abid, T. Umer, and Y. B. Zikria, “Role of IoT
technology in agriculture: A systematic literature review,”Electronics,
vol. 9, no. 2, 2020, Art. no. 319.
[4] A. Khanna and S. Kaur, “Evolution of Internet of Things (IoT) and its
significant impact in the field of precision agriculture,”Comput. Electron.
Agriculture, vol. 157, pp. 218–231, 2019.
[5] C. R. Kagan, D. P. Arnold, D. J. Cappelleri, C. M. Keske, and K. T.
Turner, “Special report: The Internet of Things for precision agriculture
(IoT4Ag),”Comput.Electron.Agriculture,vol.196,2022,Art.no.106742.
[6] J. ten Harkel, H. Bartholomeus, and L. Kooistra, “Biomass and crop height
estimation of different crops using UAV-based LiDAR,”Remote Sens.,
vol. 12, no. 1, 2020, Art. no. 17.
[7] P. K. R. Maddikuntaet al., “Unmanned aerial vehicles in smart agriculture:
Applications, requirements, and challenges,”IEEE Sensors J., vol. 21,
no. 16, pp. 17608–17619, Aug. 2021.
[8] N. Delavarpour, C. Koparan, J. Nowatzki, S. Bajwa, and X. Sun, “A tech-
nical study on UAV characteristics for precision agriculture applications
and associated practical challenges,”Remote Sens., vol. 13, no. 6, 2021,
Art. no. 1204.
[9] C. Potena, R. Khanna, J. Nieto, R. Siegwart, D. Nardi, and A. Pretto, “Agri-
ColMap: Aerial-ground collaborative 3D mapping for precision farming,”
IEEE Robot. Automat. Lett., vol. 4, no. 2, pp. 1085–1092, Apr. 2019.
[10] Z. Zhang, E. Kayacan, B. Thompson, and G. Chowdhary, “High preci-
sion control and deep learning-based corn stand counting algorithms for
agricultural robot,”Auton. Robots, vol. 44, no. 7, pp. 1289–1302, 2020.
[11] F. Rovira-Más, V. Saiz-Rubio, and A. Cuenca-Cuenca, “Augmented per-
ception for agricultural robots navigation,”IEEE Sensors J., vol. 21, no. 10,
pp. 11712–11727, May. 2021.
[12] R. Manish, Z. An, A. Habib, M. R. Tuinstra, and D. J. Cappelleri, “AgBug:
Agricultural robotic platform for in-row and under canopy crop monitoring
and assessment,” inProc. Int. Des. Eng. Tech. Conf. Comput. Inf. Eng.
Conf., 2021, Art. no. V08BT08A017.
[13] W. McAllister, J. Whitman, J. Varghese, A. Davis, and G. Chowd-
hary, “Agbots 3.0: Adaptive weed growth prediction for mechanical
weeding agbots,”IEEE Trans. Robot., vol. 38, no. 1, pp. 556–568,
## Feb. 2022.
[14] A. N. Sivakumaret al., “Learned visual navigation for under-canopy
agricultural robots,”Robot. Sci. Syst., 2021.
[15] E. Kayacan, S. N. Young, J. M. Peschel, and G. Chowdhary, “High-
precisioncontroloftrackedfieldrobotsinthepresenceofunknowntraction
coefficients,”J. Field Robot., vol. 35, no. 7, pp. 1050–1062, 2018.
[16] TerraSentia, “Terrasentia robot - earthsense, inc.,”TerraSentia, 2017.
[Online]. Available: https://www.earthsense.co
[17] ecorobotix, “Technology for the environment,”ecorobotix, 2018. [Online].
Available: https://ecorobotix.com
[18] Naoi Technologies, “Autonomous weeding for agricultural robots,”Naio
Technol., 2017. [Online]. Available: http://www.naio-technologies.com
[19] Clearpath Robotics,Clearpath Robot., 2009. [Online]. Available: https:
## //clearpathrobotics.com/
[20] Kinova Robotics,Kinova Inc., 2006. [Online]. Available: https://www.
kinovarobotics.com/
[21] D. Fox, “Adapting the sample size in particle filters through KLD-
sampling,”Int. J. Robot. Res., vol. 22, no. 12, pp. 985–1003, 2003.
[22] S. J. Julier and J. K. Uhlmann, “Unscented filtering and nonlinear estima-
tion,”
Proc. IEEE, vol. 92, no. 3, pp. 401–422, Mar. 2004.
[23] G. Grisetti, C. Stachniss, and W. Burgard, “Improved techniques for grid
mapping with Rao-Blackwellized particle filters,”IEEE Trans. Robot.,
vol. 23, no. 1, pp. 34–46, Feb. 2007.
[24] M. Esteret al., “A density-based algorithm for discovering clusters in large
spatial databases with noise,” inProc. 2nd Int. Conf. Knowl. Discov. Data
Mining, 1996, pp. 226–231.
[25] D. Anthony, S. Elbaum, A. Lorenz, and C. Detweiler, “On crop height
estimation with UAVs,” inProc. IEEE/RSJ Int. Conf. Intell. Robots Syst.,
2014, pp. 4805–4812.
[26] Z.Fan,N.Sun,Q.Qiu,T.Li,andC.Zhao,“Ahigh-throughputphenotyping
robot for measuring stalk diameters of maize crops,” inProc. IEEE 11th
## Annu. Int. Conf. Cyber Technol. Autom., Control, Intell. Syst., 2021,
pp. 128–133.
Authorized licensed use limited to: Purdue University. Downloaded on April 24,2025 at 17:02:41 UTC from IEEE Xplore.  Restrictions apply.