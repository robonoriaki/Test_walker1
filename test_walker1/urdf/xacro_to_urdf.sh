#!/bin/sh

rosrun xacro xacro test_walker1.xacro --inorder >test_walker1.urdf

check_urdf test_walker1.urdf
