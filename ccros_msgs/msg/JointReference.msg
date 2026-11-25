# One reference sample for an impedance-controlled arm
# (<arm>/reference_controller/commands). Joint order joint_0..joint_5;
# `command` adds the gripper as its 7th value.
#
# command       6 joint position targets (rad) + gripper (normalised 0..1,
#               0 = closed, 1 = open). The arm's position target.
# position      reference position r, rad
# velocity      reference velocity, rad/s
# acceleration  reference acceleration, rad/s^2
float64[7] command
float64[6] position
float64[6] velocity
float64[6] acceleration
