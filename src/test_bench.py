
import master as mm

psm,group = mm.init_and_home()

# update the location of the moveit robot
# mm.update_mc_start_go(psm, group)
# read full dvrk pose and extract both XYZ and Quaternion
cur_pose = mm.read_display_dvrk_pose(psm, verbose=False)
[qx,qy,qz,qw] = cur_pose.M.GetQuaternion()
[x,y,z] = cur_pose.p
print "XYZ:\n", [x,y,z]
print "Quaternion:\n", [qx,qy,qz,qw]
print "\nPose from MC:\n", group.get_current_pose()

mc_pose = group.get_current_pose()
[qx,qy,qz,qw] = mc_pose.pose.orientation.x, mc_pose.pose.orientation.y, mc_pose.pose.orientation.z, mc_pose.pose.orientation.w


# set pose as target an go
target = [x,y,z,qx,qy,qz,qw]
raw_input('Press enter to go to new pose')
# mm.goto_pose(psm,target,6000,group)

mm.goto_xyz(psm, [x,y+0.05,z],6000,group,fixed_orientation=[qx,qy,qz,qw])


# close
psm.shutdown()
