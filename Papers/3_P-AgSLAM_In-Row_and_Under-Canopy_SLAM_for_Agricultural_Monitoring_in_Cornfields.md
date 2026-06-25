

## 4982IEEE ROBOTICS AND AUTOMATION LETTERS, VOL. 9, NO. 6, JUNE 2024
P-AgSLAM: In-Row and Under-Canopy SLAM for
Agricultural Monitoring in Cornfields
Kitae Kim, Graduate Student Member, IEEE,AaryaDeb, Student Member, IEEE,
and David J. Cappelleri
, Member, IEEE
Abstract—In this letter, we present an in-row and under-canopy
Simultaneous  Localization  and  Mapping  (SLAM)  framework
called the Purdue AgSLAM or P-AgSLAM which is designed for
robot pose estimation and agricultural monitoring in cornfields.
Our SLAM approach is primarily based on a 3D light detection
and ranging (LiDAR) sensor and it is designed for the extraction
of   unique   morphological   features   of   cornfields   which   have
significantly different characteristics from structured indoor and
outdoor urban environments. The performance of the proposed
approach  has  been  validated  with  experiments  in  simulation
and  in  real  cornfield  environments.  P-AgSLAM  outperforms
existing  state-of-the-art  LiDAR-based  state  estimators  in  robot
pose estimations and mapping.
Index    Terms—Agriculturalautomation,roboticsand
automation in agriculture and forestry, SLAM.
## I. INTRODUCTION
## I
NTERNET of Things (IoT) technologies have been em-
ployed in the agricultural sector to realize precision agricul-
ture, leading to enhanced prediction and production controls[1],
[2],[3],[4],[5],[6],[7]. Traditional large-scale agriculture has
relied on labor-intensive and costly manual processes for crop
monitoring. To improve efficiency and accuracy in crop man-
agement, agricultural robotics combined with IoT technologies
have emerged as a viable alternative to manual labor.
Various autonomous platforms and approaches have been
utilized for crop monitoring, including Unmanned Ground Ve-
hicles (UGVs)[8],[9],[10]and Unmanned Aerial Vehicles
(UAVs)[11],[12],[13],[14]. However, cornfields, which are the
focus of this letter, present unique challenges for UAVs, partic-
ularly when monitoring crops in rows and beneath canopies. As
corn plants grow, the canopy becomes dense and closed, limiting
the flying space between crop rows. In[11],Liu,Nardarietal.
proposed an integrated system capable of autonomous flights
Manuscript received 30 October 2023; accepted 23 March 2024. Date of
publication 8 April 2024; date of current version 18 April 2024. This letter
was recommended for publication by Associate Editor Z. Wang and Editor H.
Moon upon evaluation of the reviewers’ comments. This work was supported by
IoT4Ag Engineering Research Center funded by the National Science Founda-
tion (N S F) through N S F Cooperative Agreement under Grant EEC-1941529.
(Corresponding author: David J. Cappelleri.)
Kitae Kim and Aarya Deb are with the School of Mechanical En-
gineering, Purdue University, West Lafayette, IN 47907 USA (e-mail:
kim3686@purdue.edu; deb8@purdue.edu).
David J. Cappelleri is with the School of Mechanical Engineering, Purdue
University, West Lafayette, IN 47907 USA, and also with the Weldon School
of Biomedical Engineering, Purdue University, West Lafayette, IN 47907 USA
(e-mail: dcappellg@purdue.edu).
Digital Object Identifier 10.1109/LRA.2024.3386466
Fig. 1.  Cornfields go through various stages and have differing levels of GPS
availability. Reliable GPS signals can be accessed (a) during early growth stages
or(b)inopenspacesoutsidetherows.However,(c)undercanopieswithcluttered
hanging leaves, GPS signals are either absent or unreliable.
and real-time semantic mapping in challenging under-canopy
environments. Although the results in[11]showed remarkable
performances, implementing the system in rows and under
canopies of cornfields remains exceedingly difficult due to
limited space. UGVs also face challenges in operating under
canopies, as unreliable Global Positioning System (GPS) signals
commonly used for vehicle localization are problematic (Fig.1).
With a growing interest in agricultural robotics, the study of
Simultaneous Localization and Mapping (SLAM) solutions has
recently increased in the area of agriculture. In the study by
Yuan et al.[9], a semantic SLAM solution for under-canopy
cornfields was proposed to address the challenge of under-
canopy corn stalk detection and localization. A multi-view
camera system mounted on a ground vehicle for SLAM was
introduced. Although the proposed system presented accurate
map generation detailing corn stalk positions and labels, it
had limitations on drifts over time. Shu et al. demonstrated
the successful application of sparse, indirect monocular visual
SLAM combined with both offline and real-time Multi-View
Stereo (MVS) reconstruction algorithms in soybean fields[15].
Its performance, however, depends on specific constraints, such
as the camera’s field of view in dynamic agricultural settings.
Krul et al. investigated the application of open-source visual
SLAM algorithms for the localization of drones within indoor
agricultural environments[16]. Their study emphasized the re-
liance of localization precision on camera inputs in controlled
indoor farming environments. The research pointed out that the
accuracy of the localization system diminishes in areas with
limited tracking features or where lighting conditions vary sub-
stantially. These studies highlighted the complexities and unique
2377-3766 © 2024 IEEE. Personal use is permitted, but republication/redistribution requires IEEE permission.
See https://www.ieee.org/publications/rights/index.html for more information.
Authorized licensed use limited to: Purdue University. Downloaded on April 24,2025 at 17:04:10 UTC from IEEE Xplore.  Restrictions apply.

KIM et al.: P-AgSLAM: IN-ROW AND UNDER-CANOPY SLAM FOR AGRICULTURAL MONITORING IN CORNFIELDS4983
Fig. 2.  (a) P-AgBot is a customized platform with a variety of sensors and a robotic arm. Its physical size is 40 cm in width and 60 cm in length, including
the front leaf storage bin. In this study, five sensors are used for P-AgSLAM: 1. Horizontal 3D LiDAR, 2. Vertical 3D LiDAR, 3. Internal IMU, 4. UGV wheel
encoders, and 5. RTK GPS module. Each axis of the robot frame, represented by the colors red, green, and blue, corresponds to the X, Y, and Z translationaland
rotational axes, respectively. (b) The proposed P-AgSLAM framework consists of two primary modules: LiDAR-based feature extractor (SectionIII) and a robot
state estimator using a factor graph with optional GPS measurements (SectionIV). The two modules collaboratively minimize drift and publish accurate robot
poses and maps.
challenges of designing SLAM solutions in dynamic agriculture
fields. Consequently, designing a suitable robot localizer for
cornfields is crucial and necessary for accurate crop monitoring
and robotic operations in the fields.
In this letter, we present a SLAM framework specifically
designed for in-row and under-canopy localization and agri-
cultural monitoring in cornfields, denoted as Purdue AgSLAM
(P-AgSLAM). The proposed SLAM framework seeks to min-
imize odometry drift and precisely estimate robot trajectories
in cornfields. The main contribution of P-AgSLAM lies in its
innovative approach to these agricultural-specific challenges.
Unlike conventional SLAM methods that depend on distinct
environmental features for precise localization and mapping,
P-AgSLAM is robust at operating in cornfields which have the
issues of clutter, repetitive patterns, rugged and slippery terrain,
and GPS unreliability. The P-AgSLAM framework is composed
of two main modules: (1) feature extraction and (2) robot pose
estimation and environmental mapping. By leveraging these
modules, the P-AgSLAM framework demonstrates superior per-
formance in robot pose estimation and three-dimensional (3D)
point cloud map generation compared to other state-of-the-art
light detection and ranging (LiDAR) odometry and mapping-
based methods[17],[18],[19]. This can be attributed to the fact
that corn plants lack distinguishable geometric features such as
edges, corners, or planes, which are typically used for SLAM
solutions in indoor or structured outdoor environments such as
warehouses or urban areas.
## II. S
## YSTEMOVERVIEW
The P-AgSLAM framework is built on an upgraded version of
theP-AgBotroboticplatform[20],specificallydesignedforcrop
monitoring and physical sampling in agricultural environments.
In order to broaden its sensing field of view, we configured the P-
AgBot with dual LiDARs placed perpendicularly, as opposed to
a single 2D laser scanner paired with a single vertical 3D LiDAR,
as illustratedinFig.2(a). EachLiDARshows distinct advantages
in terms of field of view. Considering that corn plants have an
average height of around 250 cm[21], and 3D LiDARs have a
limited vertical field of view, it is more efficient for measuring
and monitoring the entire morphological appearance of each
corn plant to mount the 3D LiDAR vertically. We substituted
the 2D laser scanner with a horizontally-mounted 3D LiDAR
for more efficient ground scanning.
The sensor configuration of P-AgBot takes into account the
distinct characteristics of cornfields, as shown in Fig.1.Cam-
eras, being inexpensive, lightweight, and providing a wealth of
visual information, hold significant value in SLAM. Yuan et
al. proposed a semantic SLAM solution using RGBD cameras
for autonomous weeding under corn canopies in[9]. However,
under the canopies (Fig.1(c)), the presence of overhanging
leaves and corn ears results in constantly varying illumination
conditions, and cameras are inherently sensitive to changes in
lighting and obstructions posed by corn plants. As a conse-
quence, the reliability of vision-based SLAM in agricultural
settings is compromised.
Several cutting-edge LiDAR odometry and mapping meth-
ods[17],[18],[19]derive environmental features from a contin-
uous series of scan points to estimate 6 DOF poses. LOAM[17]
performs scan matching by finding correspondences between
point features and edge or plane features for LiDAR odometry
and mapping. LeGO-LOAM[18]employs a two-step opti-
mization process in its LiDAR odometry module, extracting
planar and edge features through the evaluation of point cloud
roughness. LIO-SAM[19]is a framework for tightly-coupled
LiDAR inertial odometry, facilitating mobile robot trajectory
estimation and map building. However, these approaches are not
optimally suited for pose estimation in cornfields, as cornfields
present more limited or repetitive features compared to struc-
tured urban environments. Therefore, this letter presents a novel
pose estimation method specifically optimized for the unique
attributes of cornfields. The proposed P-AgSLAM framework
is illustrated in Fig.2(b). For feature extraction, particularly
in agricultural environments, LiDAR-based methods surpass
vision-based approaches. LiDAR consistently performs under
varying lighting conditions. Unlike vision sensors, LiDAR is
unaffected by crop color or texture variations, ensuring robust
performanceacrossdifferentcrops.Additionally,theprecisedis-
tance measurements from LiDAR are essential for accurate crop
feature extraction, avoiding the depth estimation complexities
Authorized licensed use limited to: Purdue University. Downloaded on April 24,2025 at 17:04:10 UTC from IEEE Xplore.  Restrictions apply.

## 4984IEEE ROBOTICS AND AUTOMATION LETTERS, VOL. 9, NO. 6, JUNE 2024
often faced by vision sensors. Therefore, to ensure robustness
in under-canopy environments, our system incorporates wheel
encoders, an IMU sensor, two 3D LiDARs (Velodyne VLP-16 s),
and a GPS module (Emlid M2). Wheel odometry (WO) and the
internal IMU sensor data are used for the initial guess of robot
poses with Extended Kalman Filter (EKF)[22]fusion. Pose
drift of the UGV, generally resulting from uneven and slippery
terrains, remains a challenge. Our LiDAR odometry, which is
computed through the feature extraction module, reduces the
drift under the canopy. Furthermore, by optionally associating
reliable RTK GPS measurements, which are available during
early growth stages (Fig.1(a)) or in open spaces (Fig.1(b)),
with a global factor graph, the robot state estimation module in
our system accurately estimates the robot trajectory in global
coordinates.
## III. F
## EATUREEXTRACTION
In the feature extraction stage, ground plane and corn stalk
features are extracted and the extracted features are used to
compute LiDAR odometry. As an initial step, the original point
clouds are downsampled using a VoxelGrid filter via the Point
Cloud Library (PCL)[23], improving the computational ef-
ficiency of the system. We represent the downsampled point
clouds gathered by horizontal (H) or vertical (V) LiDAR at time
tasP
i
t
## ={p
i
## 1
## ,p
i
## 2
## ,...,p
i
n
},i=H,V. Point cloudsP
## H
t
andP
## V
t
are employed to extract different features of the surrounding
environment, specifically ground featuresF
## H,g
t
and corn stalk
featuresF
c
t
. For the ground features, we assume that the terrain
around the robot is locally flat. The number of laser scans
reflectingfromthegroundplaneisconsiderablylargerthanthose
from individual corn plants. Given these environmental char-
acteristics, the Random Sampling and Consensus (RANSAC)
algorithm, one of the most robust model fitting algorithms, is
utilized for ground point segmentation[24].
For the stalk features, we take into account the unique mor-
phological characteristics of corn plants. Corn plants typically
grow with a single stalk accompanied by multiple leaves, and
corn ears are attached to their stalk. In certain fields of view, the
slender corn stalk may be obscured by the hanging leaves. These
morphological features make it challenging to ensure a sufficient
number of reflected scans from each height level of an individual
corn stalk are captured as the P-AgBot navigates between rows.
Chen et al. proposed a tree feature extraction method for forest
environments in[25]. While trees and corn plants share some
morphological similarities, such as having a single trunk or
stalk, there are significant differences. Trees, in contrast to corn
plants, do not possess hanging leaves surrounding their trunks,
facilitating the clear and direct imaging of tree trunk forms.
Moreover, unlike forests where there is ample space for robot
operation, the space inside corn rows is constrained, leading to
blind spots for 3D LiDARs. Given these differences, the method
proposed in[25]is unsuitable for application in cornfields.
The methodology we propose for extracting corn stalk fea-
tures involves two steps: individual corn plant segmentation,
and stalk feature parameterization. To extract corn stalk features,
## P
## V
t
is divided into two groups: corn plant point clouds
## ̄
## P
## V
t
and
the ground point cloudsF
## V,g
t
. We then filter
## ̄
## P
## V
t
into multiple
clusters, each of which is a point cloud for individual corn
plants which form the basis for extracting stalk features. Corn
plants have the following morphological characteristics: (1) the
lower section of the crop is the most stable because it moves
less and is less affected by wind; (2) the lower section is less
occluded by the overhang of leaves; and (3) the corn stalk grows
vertically. Considering these unique attributes, the clustering
process of each corn point cloud is designed as follows. We
identify the point clouds of the lower part of crops which are
located at a certain range of heights from the extracted ground
planeF
## H,g
t
. Following this, we cluster the searched points using
the density-based spatial clustering of applications with noise
(DBSCAN)[26]method. Finally, we add points located above
the clustered points into each cluster, allowing for a certain
degree of deviation that runs parallel toF
## H,g
t
. We denote the
set of clustered point clouds for corn plants with indexkat time
tas
## ̄
## P
## V,k
t
## .
To estimate robot poses using relative transformations of corn
stalk models in the local frame, as described in SectionIV,
we parameterize corn stalks as 3D straight lines. The stalk
models are parameterized using a median normalized-vector
growth (MNVG) algorithm, an approach for corn plant modeling
proposed in[27]. The MNVG algorithm performs stem and leaf
segmentation on individual corn plants scanned by 3D LiDAR
and identifies representative points for each corn stem. As
## ̄
## P
## V,k
t
is clustered based on density, it can include outliers not originat-
ing from stems. Therefore, by utilizing the representative point
identification through the MNVG algorithm, the accuracy of 3D
linemodelingisenhanced.UtilizingtheL1-medianmethod[28],
the MNVG algorithm generates a starting seed point from the
bottom points of each corn plant. From the initial seed point,
the growth direction of each corn plant is determined using
median normalized vectors, and subsequent growing seed points
are identified. This process is carried out recursively until it is
no longer available to discern the growth direction from the last
seed point. We denoteP
k
t
## ={p
## 1
## ,p
## 2
## ,...,p
n
}as the set of seed
points for thek
th
corn plant at timet, which is used for corn
plant modeling.
In our system, we utilize singular value decomposition
(SVD)[29]to solve a 3D Orthogonal Distance Regression
(ODR) problem[30]for line fitting. ODR is a mathematical
technique that finds the best-fitting line for a given set of points
by minimizing their squared orthogonal distances to the line. We
represent the representative line model of thek
th
corn plant at
timetasF
c,k
t
## =(r
k
## 0
## ,v
k
), wherer
k
## 0
denotes the mean position
value ofP
k
t
, andv
k
signifies the direction vector of the line.
Point clouds before and after feature extraction are visualized in
Fig.3. Following this process, onlyF
## H,g
t
andF
c,k
t
are retained.
## IV. S
## TATEESTIMATION&MAPPING
A. Local Pose Estimation: LiDAR Odometry
As the P-AgBot navigates between corn rows, an EKF fuses
measurements from WO and IMU data, subsequently publishing
## ̄
## R
t
, which represents the 6 DOF robot pose at timet(position:
Authorized licensed use limited to: Purdue University. Downloaded on April 24,2025 at 17:04:10 UTC from IEEE Xplore.  Restrictions apply.

KIM et al.: P-AgSLAM: IN-ROW AND UNDER-CANOPY SLAM FOR AGRICULTURAL MONITORING IN CORNFIELDS4985
Fig. 3.  (a) Downsampled point cloudsP
## V
t
before extracting features.
(b) Ground point clouds
## F
## V,g
t
with white point clouds, and stalk features
## F
c,k
t
,k=1,...,N with yellow lines (N: The number of detected corn plants).
t
## X
## ,t
## Y
## ,t
## Z
, and orientation:θ
roll
## ,θ
pitch
## ,θ
yaw
). We establish the
initial robot heading and the normal direction of the ground
plane, counteracting the gravitational force, as the positive X
and Z direction in the global frame. With the initial EKF pose
## ̄
## R
## 0
, which is set as the initial robot pose in the global map frame,
we reduce EKF pose drifts between consecutive timestamps that
are caused by the environmental challenges with our LiDAR
odometry. Let
## ̄
## T
t−1,t
denote theSE(3) transformation between
consecutive EKF poses
## ̄
## R
t−1
and
## ̄
## R
t
. This transformation main-
tains a mathematical relationship, as expressed in(1):
## ̄
## T
t−1,t
## =
## (
## ̄
## R
t−1
## )
## −1
## ·
## ̄
## R
t
## (1)
We define the corrected pose at timetfrom
## ̄
## R
t
asR
t
.The
initial estimate of the relative motion between two consecutive
corrected posesR
t−1
andR
t
is obtained using
## ̄
## T
t−1,t
. In order
to correct the poses from
## ̄
## R
t
toR
t
, we separately find the
best feature match forF
## H,g
t
andF
c,k
t
, which are extracted from
LiDAR scans (SectionIII). The objective of feature matching is
to compute the optimal transformation that minimizes the drifts
ofextractedfeaturesbetweentimet−1andt.Toexecutefeature
matching, the P-AgSLAM framework modifies the traditional
Iterative Closest Point (ICP) algorithm[31], which generates
transformation data aimed at minimizing discrepancies between
two point clouds. ICP processes two categories of input data:
reference and source point clouds. The reference points remain
stationary, while the source points undergo transformation to
achieve an optimal alignment. In our methodology, features at
timet−1andtserve as reference and source points, respec-
tively. For ground feature matching withF
## H,g
t−1
andF
## H,g
t
## ,we
calculatepoint-to-planedistancesandascertainthemostsuitable
transformation matrix between them.
For the case ofF
c,k
t
, prior to computing the 3D point-to-point
Euclidean distances within the ICP algorithm, we establish line
feature correspondences to identify identical stalks in two dis-
tinctlocalrobotframes,
## ̄
## R
t−1
and
## ̄
## R
t
.Bycomparingr
k
## 0
andv
k
of
eachF
c,k
t−1
withF
c,k
t
, we assess geometric resemblances to deter-
mine the feature correspondences. We examine the disparity in
both the direction ofv
k
and the Euclidean distance between the
projectedr
k
## 0
to the ground plane to ascertain the presence of sim-
ilarity. Once we identify stalk feature correspondences for each
reference featureF
c,k
t−1
, the system employs the seed pointsP
k
t
to form the fundamental components ofF
c,k
t
. This is used for the
3D point-to-point Euclidean distance computation using the ICP
algorithm. The ICP algorithm yields the transformation, which
combines translation and rotation to minimize the Euclidean
distance error metric, and iteratively refines the transformation
until the output fulfills the criteria during subsequent iterations.
The transformation for pose correction comprises two types
of ICP algorithm outputs fromF
## H,g
t
andF
c,k
t
. The P-AgSLAM
system depends on different motion constraint components from
## F
## H,g
t
andF
c,k
t
to account for their diverse and unique geometric
properties. The ground model yields anSE(3) transformation
## T
l,g
t−1,t
, encompassing one translational (Z) and two rotational
(roll, pitch) motions. Through the ICP result derived fromF
## H,g
t
## ,
Z, roll, and pitch constraints exhibit high dependability. How-
ever, X, Y, and yaw motion estimations fromF
## H,g
t
are not precise
due to their planar characteristics. Features presented inF
c,k
t
ef-
fectively constrain and correct poses in X, Y, and yaw directions
with the morphological attributes of cornfields. As a result, our
system acquires an additionalSE(3) transformationT
l,c
t−1,t
from
## F
c,k
t
with 3 DOF, considering two translational (X, Y) and one
rotational (yaw) motion. Therefore, the transformation for pose
correctionT
l
t−1,t
is determined by equations, as expressed in
## (2)and(3):
## T
l
t−1,t
## =T
l,g
t−1,t
## ·T
l,c
t−1,t
## (2)
## T
l,c
t−1,t
## =
## {
## T
l,c
t−1,t
,if Number Of Stalks≥N
c,min
## I,otherwise
## (3)
The system allows users to control a threshold, denoted as the
minimum required number of corn stalks, N
c,min
. Depending on
this threshold value, the system decides whether to useT
l,c
t−1,t
or not. If the number of extracted corn stalk features falls below
## N
c,min
, which can occur in situations such as when some corn
stalks are occluded by hanging leaves or weeds, the system
determines that there are insufficient line features to correct
the pose usingF
c,k
t
, and setsT
l,c
t−1,t
toI, whereIrepresents
the identity matrix, as expressed in(3). Similarly, if the ground
planeF
## H,g
t
is not extracted, the system assignsT
l,g
t−1,t
toIfor the
same reason. Ultimately, the robot pose is estimated through the
combination of
## ̄
## T
t−1,t
andT
l
t−1,t
. The mathematical expression
of the current robot poseR
t
is shown in(4):
## R
t
## =R
t−1
## ·
## ̄
## T
t−1,t
## ·T
l
t−1,t
## (4)
B. Global Pose Estimation: Optional GPS Measurements
While LiDAR odometry in SectionIV-Aallows us to achieve
reliable local pose estimation, pose drift still increases during
long-term navigation tasks. To solve this issue, we introduce
the optional use of GPS signals when reliable absolute position
information is collected. We leverage a factor graph[32]to
optionally associate GPS measurements with the locally esti-
mated poses. This factor graph aids in modeling the robot state
estimation problem, which can be structured as a maximum
a posteriori (MAP) optimization problem. When the distance
between corrected robot poses exceeds a user-defined threshold,
the corrected robot pose is incorporated into the graph as a new
Authorized licensed use limited to: Purdue University. Downloaded on April 24,2025 at 17:04:10 UTC from IEEE Xplore.  Restrictions apply.

## 4986IEEE ROBOTICS AND AUTOMATION LETTERS, VOL. 9, NO. 6, JUNE 2024
Fig. 4.  Visualization of test environments: (a) the simulation (sim) and (b) the
real cornfield (real1, real2).
## TABLE I
## S
## PECIFICATIONS OFEXPERIMENTALENVIRONMENTS
robot state factor. Upon the insertion of a new factor, the factor
graph is optimized. For the GPS factor, when reliable GPS data is
received, particularly outside the corn rows or in early-stage corn
rows which guarantee open spaces, a new GPS factor is added
to the factor graph. The reliability of the GPS measurements is
determined based on the received GPS position covariance.
## C. Mapping
To monitor and assess the conditions of agricultural environ-
ments, a high-resolution map is essential. P-AgSLAM is capable
of constructing a 3D point cloud mapM, given the optimized
robot pose trajectory from SectionIV-B.Weemploythedown-
sampledpointcloudsP
i
t
,i=H,V acquiredinSectionIII. These
point cloudsP
i
t
are initially linked to the initial guess poses
## ̄
## R
t
## .
## Giventhattherelativetransformationbetweentherobotbaseand
the 3D LiDAR is fixed, we can associateP
i
t
with the optimized
pose from the factor graph andP
i
t
are projected into the global
map frame as a point cloud map. The point clouds inMare
updated once the robot trajectory is updated. Corn plants located
in several rows on both sides of the robot can be effectively
incorporated intoMwhile navigating a particular row, owing
to the extensive scanning capabilities of the 3D LiDARs.
## V. E
## XPERIMENTALRESULTS
In order to evaluate the performance of P-AgSLAM, tests
were conducted in both simulation and real cornfield environ-
ments. The experimental environments and their specifications
are detailed in Fig.4and TableI, respectively. To thoroughly
evaluate performance, we designed scenarios that provide a
straightforward comparison in terms of robot trajectories and
generated maps. In the simulation (ROS Gazebo), the P-AgBot
## TABLE II
## F
INALROBOTPOSETRANSLATION ERROR[m]
maneuvered the rows, completing two counterclockwise rect-
angular circuits before returning to its starting position (sim),
noting that we operated under the assumption that RTK GPS
measurements were absent in the simulation. We added sen-
sor noise in the simulation to mimic real-world conditions.
In real-world tests, the robot drove through a loop, traversing
multiple rows with different driving distances (real1, real2). The
robot navigates following the rows under the supervision of the
operator, ensuring it avoids any collisions with crops or other
obstacles. For each scenario, we implemented three leading
LiDAR-based state estimation methods: LOAM[17], LeGO-
LOAM[18],LIO-SAM[19], and a fundamental ICP algorithm
with raw point cloud data (Full-ICP) as baselines. We term a
fusion of WO and IMU in the noisy condition as WO+IMU. For
LOAM and LeGO-LOAM, GPS data integration is not feasible
due to their inherent design limitations. Consequently, in our
comparative study, these methods were evaluated without incor-
porating GPS measurements. In contrast, LIO-SAM includes a
GPS factor in its system architecture, enabling the integration
of GPS data. Therefore, in our experiments, LIO-SAM was
evaluated with GPS data integration. In addition, to evaluate
the impact of upgrading the sensor configuration of P-AgBot
on the effectiveness of the proposed algorithm, P-AgSLAM,
we conducted an additional experiment using the system with
a single vertical LiDAR sensor configuration, herein referred to
as P-AgSLAM (single LiDAR). In this test, we utilizedF
## V,g
t
for
ground feature extraction in P-AgSLAM, as opposed toF
## H,g
t
used in the dual LiDAR setup. The purpose of this experiment
was to directly compare the performance of P-AgSLAM with
a single vertical LiDAR against its performance with dual Li-
DARs, thereby illustrating the enhancements achieved with the
upgraded sensor configuration.
The estimated pose trajectories from each method are shown
in Fig.5. We evaluated the performance of all methods quanti-
tatively with two metrics: the final robot pose translation error
when the robot returns to the starting position (TablesIIand
III), and the average maximum and minimum estimated robot
pose drift in each row (TableIV). The results demonstrate the
robustness of our method even during rotational movements
which are challenging scenarios for traditional LiDAR odometry
methods. Fig.4(b)and TablesIIandIIIdescribe that LOAM,
LeGO-LOAM, and LIO-SAM drift significantly in all directions
in both simulation and real testbeds. Full-ICP method appears
to be stationary. The trajectory results of baselines in the real
Authorized licensed use limited to: Purdue University. Downloaded on April 24,2025 at 17:04:10 UTC from IEEE Xplore.  Restrictions apply.

KIM et al.: P-AgSLAM: IN-ROW AND UNDER-CANOPY SLAM FOR AGRICULTURAL MONITORING IN CORNFIELDS4987
Fig. 5.  Ground truth and estimated robot trajectories by P-AgSLAM and the baselines in different scenarios (TableI). The trajectory results of baselines in the
real environment (real1, real2) are not illustrated because they failed to obtain meaningful results. In each scenario, the initial robot frame is aligned with the origin
of the map frame, with the gravitational direction aligned along the negative Z-axis.
## TABLE III
## XY
ANDZDRIFT OFFINALROBOT POSE[m]
## TABLE IV
## A
## VERAGEMAXIMUM&MINIMUMESTIMATEDROBOTPOSEDRIFT INEACH
## ROW
## [m]
environment (real1, real2) are not illustrated because they fail
to obtain meaningful results. This is primarily attributed to its
inherent limitations in handling environments characterized by
noisy and repetitive raw point clouds. The WO+IMU method
shows less drift compared to these methods but was still suscep-
tible to drifts in all directions. Notably, P-AgSLAM effectively
mitigates the drift, as illustrated in TablesIIIandIV. In addition,
our proposed method features the optional association of reliable
RTK GPS measurements, which is particularly beneficial in spe-
cific scenarios such as at the end of rows or in open spaces where
reliable RTK GPS data can be measured. It allows for precise
updating of the robot pose in the global frame, compared to other
baseline approaches. The efficiency of our method is highlighted
by the runtime metrics. With an average runtime of 36.44 ms,
the system shows runtime efficiency, maintaining a runtime of
less than 100 ms even when operating at a LiDAR rotation rate
of 10 Hz. In other words, our proposed framework processes
in real-time and it is an important factor for applications in
real environments. With the updated sensor configuration of
P-AgBot, the transition from a single LiDAR to a dual LiDAR
setup has resulted in significant performance improvements in
robot pose estimation. The single LiDAR system results in larger
Authorized licensed use limited to: Purdue University. Downloaded on April 24,2025 at 17:04:10 UTC from IEEE Xplore.  Restrictions apply.

## 4988IEEE ROBOTICS AND AUTOMATION LETTERS, VOL. 9, NO. 6, JUNE 2024
Fig. 6.  Visualization of the point cloud map of (a) simulation (3D view), (b) Top view of real1 environment side-by-side with the top view of the corresponding
actual environment image at the same scale. The gravitational direction is aligned along the negative Z-axis of the map frame.
andmoreirregulardrifts,particularlyintheZ-axis,roll,andpitch
directions, in contrast to the dual LiDAR configuration. This
difference is clearly depicted in Fig.4(c)and(d). It demonstrates
that a single LiDAR system is comparatively less efficient and
robust in continuous ground feature extraction and optimization
than the upgraded P-AgBot sensor system. The dual LiDARs,
with their wider horizontal field of view, provide more compre-
hensive scanning of the ground, leading to more accurate and
stable robot pose estimation.
Fig.6displays point cloud cornfield maps of both the sim-
ulation (Fig.6(a)) and real testbed (Fig.6(b)) generated by
P-AgSLAM. As the precision of the 3D mapping is intrinsically
linked to the extent of the accumulated trajectory drift, the
performance of P-AgSLAM, with its accurate robot state estima-
tion, makes the map quantitatively superior. With a side-by-side
comparison image (Fig.6(b)) which compares the produced map
with its actual map image, it is clear that individual rows of crops
in the produced map are not only distinctly represented but also
parallel with each other in the global frame as the real environ-
ment is. As baseline methods failed in estimating the robot poses
under the canopies, it led to corresponding failures in mapping
as well. Regarding the accuracy of the generated 3D maps, it is
important to note that it is contingent on the performance of pose
estimation. This is because the relative transformation between
the robot base and the 3D LiDAR is fixed, implying that the
accuracy of the final maps is identical to the accuracy of the
final pose correction results which are quantitatively described
in TablesII,III, andIV. This mapping result is significant in that
the robot enters an unknown environment and P-AgSLAM can
integrate an unmapped area with a previously generated map. As
a result, the P-AgSLAM approach offers significant potential for
a wide range of agricultural applications.
In conclusion, our systematic framework is robust at both ex-
tractingmorereliableandrepresentativefeaturesfromcornfields
and accurately estimating robot pose as compared to baseline
methods which primarily focus on structured geometric features.
Both feature extraction and GPS integration are critical com-
ponents that enhance the accuracy of our pose estimation sys-
tem from individual perspectives. Feature extraction accurately
estimates positions and orientations at a local level, essential
for precise in-row navigation and under-canopy monitoring.
Conversely, GPS integration extends this precision to a global
scale, ensuring consistent and reliable navigation over long
distances. However, the feature extractors and state estimators
used by baseline methods do not specifically identify or dis-
tinguish repetitive corn characteristics, making it challenging
to determine whether the robot traverses between corn rows or
stays in the same position. This is the primary reason for the
pronounced drift in the trajectories estimated by these methods,
as opposed to those estimated by P-AgSLAM.
## VI. C
## ONCLUSION
In this letter, we introduced P-AgSLAM, a novel in-row and
under-canopy SLAM framework specifically designed for corn-
fields. Our agricultural SLAM system can estimate robot states
and generate a 3D point cloud map while P-AgBot navigates
under the canopy, where GPS signals are often unreliable or
unavailable. When the robot finds itself in regions with reliable
GPS signal reception, our system performs optimizations on the
global poses. The combined capabilities of local and global pose
estimation in our method effectively reduce the drift caused by
challenging agricultural environments. We compared the pose
estimation and mapping performances of P-AgSLAM against
other state-of-the-art LiDAR state estimators across various
experimental settings and P-AgSLAM outperforms baselines.
We ultimately aim to integrate the P-AgSLAM solution into
our autonomous navigation module and crop sampling module
capable of picking or cutting target crops or leaves in diverse
agricultural environments for precision agriculture applications.
## R
## EFERENCES
[1] N. Zhang, M. Wang, and N. Wang, “Precision agriculture–A worldwide
overview,”Comput. Electron. Agriculture, vol. 36, no. 2/3, pp. 113–132,
## 2002.
[2] S. Liaghat et al., “A review: The role of remote sensing in precision
agriculture,”Amer. J. Agricultural Biol. Sci., vol. 5, no. 1, pp. 50–55, 2010.
[3] C. R. Kagan, D. P. Arnold, D. J. Cappelleri, C. M. Keske, and
K. T. Turner, “Special report: The Internet of Things for precision
Authorized licensed use limited to: Purdue University. Downloaded on April 24,2025 at 17:04:10 UTC from IEEE Xplore.  Restrictions apply.

KIM et al.: P-AgSLAM: IN-ROW AND UNDER-CANOPY SLAM FOR AGRICULTURAL MONITORING IN CORNFIELDS4989
agriculture (IoT4AG),”Comput. Electron. Agriculture, vol. 196, 2022,
Art. no. 106742.
[4] R. P. Sishodia, R. L. Ray, and S. K. Singh, “Applications of remote sensing
in precision agriculture: A review,”Remote Sens., vol. 12, no. 19, 2020,
Art. no. 3136.
[5] J. S. Duhan, R. Kumar, N. Kumar, P. Kaur, K. Nehra, and S. Duhan, “Nan-
otechnology: The new perspective in precision agriculture,”Biotechnol.
Rep., vol. 15, pp. 11–23, 2017.
[6] J. P. Vasconez, G. A. Kantor, and F. A. A. Cheein, “Human–robot in-
teraction in agriculture: A survey and current challenges,”Biosyst. Eng.,
vol. 179, pp. 35–48, 2019.
[7] A. Atefi, Y. Ge, S. Pitla, and J. Schnable, “Robotic technologies for
high-throughput plant phenotyping: Contemporary reviews and future
perspectives,”Front. Plant Sci., vol. 12, 2021, Art. no. 611940.
[8] W. McAllister, J. Whitman, J. Varghese, A. Davis, and G. Chowd-
hary, “Agbots 3.0: Adaptive weed growth prediction for mechanical
weeding agbots,”IEEE Trans. Robot., vol. 38, no. 1, pp. 556–568,
## Feb. 2022.
[9] J. Yuan, J. Hong, J. Sattar, and V. Isler, “ROW-SLAM: Under-canopy
cornfield semantic SLAM,” inProc. IEEE Int. Conf. Robot. Automat.,
2022, pp. 2244–2250.
[10] R. Manish, Z. An, A. Habib, M. R. Tuinstra, and D. J. Cappelleri, “AgBug:
Agricultural robotic platform for in-row and under canopy crop monitoring
and assessment,” inProc. Int. Des. Eng. Tech. Conf. Comput. Inf. Eng.
Conf., 2021, Paper V08BT08A017.
[11] X. Liu et al., “Large-scale autonomous flight with real-time semantic
SLAM under dense forest canopy,”IEEE Robot. Automat. Lett.,vol.7,
no. 2, pp. 5512–5519, Apr. 2022.
[12] J. ten Harkel, H. Bartholomeus, and L. Kooistra, “Biomass and crop height
estimation of different crops using UAV-based LiDAR,”Remote Sens.,
vol. 12, no. 1, 2019, Art. no. 17.
[13] M. Maimaitijiang, V. Sagan, P. Sidike, A. M. Daloye, H. Erkbol, and F.
B. Fritschi, “Crop monitoring using satellite/uav data fusion and machine
learning,”Remote Sens., vol. 12, no. 9, 2020, Art. no. 1357.
[14] A. Karami, M. Crawford, and E. J. Delp, “Automatic plant counting and
location based on a few-shot learning technique,”IEEE J. Sel. Topics Appl.
Earth Observ. Remote Sens., vol. 13, pp. 5872–5886, 2020.
[15] F. Shu, P. Lesur, Y. Xie, A. Pagani, and D. Stricker, “SLAM in the field: An
evaluation of monocular mapping and localization on challenging dynamic
agricultural environment,” inProc. IEEE/CVF Winter Conf. Appl. Comput.
Vis., 2021, pp. 1761–1771.
[16] S. Krul, C. Pantos, M. Frangulea, and J. Valente, “Visual slam for indoor
livestock and farming using a small drone with a monocular camera: A
feasibility study,”Drones, vol. 5, no. 2, 2021, Art. no. 41.
[17] J. Zhang and S. Singh, “LOAM: LiDAR odometry and mapping in real-
time,” inProc. Robot.: Sci. Syst., Berkeley, CA, 2014, pp. 1–9.
[18] T.ShanandB.Englot,“LeGO-LOAM:Lightweightandground-optimized
LiDAR odometry and mapping on variable terrain,” inProc. IEEE/RSJ Int.
Conf. Intell. Robots Syst., 2018, pp. 4758–4765.
[19] T. Shan, B. Englot, D. Meyers, W. Wang, C. Ratti, and D. Rus, “LIO-SAM:
Tightly-coupled LiDAR inertial odometry via smoothing and mapping,”
inProc. IEEE/RSJ Int. Conf. Intell. Robots Syst., 2020, pp. 5135–5142.
[20] K. Kim, A. Deb, and D. J. Cappelleri, “P-AgBot: In-row & under-canopy
agricultural robot for monitoring and physical sampling,”IEEE Robot.
Automat. Lett., vol. 7, no. 3, pp. 7942–7949, Jul. 2022.
[21] Q. Xie et al., “Crop height estimation of corn from multi-year radarsat-2
polarimetric observables using machine learning,”Remote Sens., vol. 13,
no. 3, 2021, Art. no. 392.
[22] S. J. Julier and J. K. Uhlmann, “Unscented filtering and nonlinear estima-
tion,”Proc. IEEE, vol. 92, no. 3, pp. 401–422, Mar. 2004.
[23] R. B. Rusu and S. Cousins, “3D is here: Point cloud library (PCL),” in
Proc. IEEE Int. Conf. Robot. Automat., Shanghai, China, 2011, pp. 1–4.
[24] M. A. Fischler and R. C. Bolles, “Random sample consensus: A paradigm
for model fitting with applications to image analysis and automated car-
tography,”Commun. ACM
, vol. 24, no. 6, pp. 381–395, 1981.
[25] S. W. Chen et al., “SLOAM: Semantic LiDAR odometry and mapping for
forest inventory,”IEEE Robot. Automat. Lett., vol. 5, no. 2, pp. 612–619,
## Apr. 2020.
[26] M. Ester et al., “A density-based algorithm for discovering clusters in large
spatial databases with noise,” inProc. 2nd Int. Conf. Knowl. Discov. Data
Mining, 1996, pp. 226–231.
[27] S. Jin et al., “Stem–leaf segmentation and phenotypic trait extraction
of individual maize using terrestrial LiDAR data,”IEEE Trans. Geosci.
Remote Sens., vol. 57, no. 3, pp. 1336–1346, Mar. 2019.
[28] M. Daszykowski, K. Kaczmarek, Y. Vander Heyden, and B. Walczak, “Ro-
bust statistics in data analysis–A review: Basic concepts,”Chemometrics
Intell. Lab. Syst., vol. 85, no. 2, pp. 203–219, 2007.
[29] C. Eckart and G. Young, “The approximation of one matrix by another of
lower rank,”Psychometrika, vol. 1, no. 3, pp. 211–218, 1936.
[30] P. T. Boggs and J. E. Rogers, “Orthogonal distance regression,”Contem-
porary Math., vol. 112, pp. 183–194, 1990.
[31] K. S. Arun, T. S. Huang, and S. D. Blostein, “Least-squares fitting of two
3-D point sets,”IEEE Trans. Pattern Anal. Mach. Intell., vol. PAMI-9,
no. 5, pp. 698–700, Sep. 1987.
[32] F. Dellaert, “Factor graphs and GTSAM: A hands-on introduction,”Center
Robot. Intell. Mach., Georgia Inst. Technol., Tech. Rep. GT-RIM-CP&R-
2012-002, 2012. [Online]. Available: http://hdl.handle.net/1853/45226
Authorized licensed use limited to: Purdue University. Downloaded on April 24,2025 at 17:04:10 UTC from IEEE Xplore.  Restrictions apply.