
# Desc: Device definitions for Robotnik Summit Robot.
# Date 2012-01-07

define summit_laser_front ranger
(
sensor (
# laser specific properties 
#  range_min 0.0
  range_max 5.0
  fov 235.0
  samples 270
)
  # generic model properties
  color "blue"
  size [ 0.07 0.07 0.05]
  pose [0.45 0.0 -0.15 0]
)

define summit_position position
(
  #color "red"		# Default color.

  gui_nose 1                  	# Draw a nose on the robot so we can see which way it points
  obstacle_return 1           	# Can hit things.
  laser_return 1                 # reflects laser beams
  ranger_return 1             	# reflects sonar beams
  blob_return 1               	# Seen by blobfinders  
  fiducial_return 1           	# Seen as "1" fiducial finders

  localization "gps"         	  
  localization_origin [0 0 0 0] 	# Start odometry at (0, 0, 0).
  wheelbase 0.37

  # alternative odometric localization with simple error model
  # localization "odom"         		# Change to "gps" to have impossibly perfect, global odometry
  #  odom_error [ 0.05 0.05 0.1 ]   	# Odometry error or slip in X, Y and Theta
                                   	# (Uniform random distribution)   
)

define summit_base  summit_position
(
  # actual size
  size [0.567 0.3025 0.32] # sizes

  # the Summit's center of rotation is in the center of traction axis
  # expressed as displ respect geom center
  origin [0.1925 0 0 0]

  # draw a nose on the robot so we can see which way it points
  gui_nose 0 

  # estimated mass in KG
  mass 12.0 

  # ackerman type steering drive
  drive "car"
)

define summit_no_laser summit_base
(
 # simplified Body shape:
  # the shape of Summit
  #polygons 1
  #polygon[0].points 6
  #polygon[0].point[0] [0 0]
  #polygon[0].point[1] [0.3 0]
  #polygon[0].point[2] [0.5 0.1]
  #polygon[0].point[3] [0.5 0.3]
  #polygon[0].point[4] [0.3 0.4]
  #polygon[0].point[5] [0 0.4]

  # simplified Body shape:
  block(
        points 6
        point[0] [0 0]
        point[1] [0.385 0]
        point[2] [0.567 0.040]
        point[3] [0.567 0.220]
        point[4] [0.385 0.260]
        point[5] [0 0.260]
        z [0.2 0.32]
        color "red"
    )

    # rear right wheel
    block
    (
        points 4
        point[0] [ 0.0  -0.0425 ]
        point[1] [ 0.182 -0.0425 ]
        point[2] [ 0.182  0.0425 ]
    	point[3] [ 0.0   0.0425 ]
    	z [ 0 0.182 ]
    	color "black"
    )

    #front right wheel
    block
    (
        points 4
        point[0] [ 0.385 -0.0425 ]
        point[1] [ 0.567 -0.0425 ]
        point[2] [ 0.567 0.0425 ]
        point[3] [ 0.385 0.0425 ]
        z [ 0 0.182 ]
        color "black"
    )

    # rear left wheel
    block
    (
        points 4
        point[0] [ 0.0  0.2225 ]
        point[1] [ 0.182 0.2225 ]
        point[2] [ 0.182  0.3025 ]
        point[3] [ 0.0  0.3025 ]
        z [ 0 0.182 ]
        color "black"
    )

    #front left wheel
    block
    (
        points 4
        point[0] [ 0.385 0.2225 ]
        point[1] [ 0.567 0.2225 ]
        point[2] [ 0.567 0.3025 ]
        point[3] [ 0.385 0.3025 ]
        z [ 0 0.182 ]
        color "black"
    )

)

define summit_no_laser summit_base
(
  # The Summit center of rotation is offset from its center of area
  origin [0.0 0.0 0.0 0.0]

  # Estimated mass in KG
  mass 12.0 
) 

define summit summit_no_laser
(
  summit_laser_front()
)
