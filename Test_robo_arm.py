#!/usr/bin/env python
# coding: utf-8

# This is the file that passes all the tests from the unit test portion of the assignment

import numpy as np
from vedo import *
from vedo import Arrow, Sphere, Cylinder, Axes, show


def forward_kinematics(Phi, L1, L2, L3, L4):

    phi1, phi2, phi3, phi4 = Phi

    rad = .4

    # Matrix of Frame 0 (world frame)
    T_00 = np.array([[1, 0, 0, 3],
                     [0, 1, 0, 2],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

    # Matrix of Frame 1 
    R_01 = RotationMatrix(phi1, axis_name='z')  # Rotation matrix
    p1 = np.array([[0], [0], [0.0]])  # Frame's origin
    t_01 = p1  # Translation vector
    T_01 = getLocalFrameMatrix(R_01, t_01)  # Matrix of Frame 1 -> Frame 0 (i.e., the world frame)

    # Matrix of Frame 2 
    R_12 = RotationMatrix(phi2, axis_name='z')  # Rotation matrix
    p2 = np.array([[L1 + (2 * rad)], [0.0], [0.0]])  # Frame's origin 
    t_12 = p2  # Translation vector
    T_12 = getLocalFrameMatrix(R_12, t_12)
    T_02 = T_00 @ T_01 @ T_12  # Matrix of Frame 2 -> Frame 0 (i.e., the world frame)

    # Matrix of Frame 3 
    R_23 = RotationMatrix(phi3, axis_name='z')  # Rotation matrix
    p3 = np.array([[L2 + (2 * rad)], [0.0], [0.0]])  # Frame's origin 
    t_23 = p3  # Translation vector
    T_23 = getLocalFrameMatrix(R_23, t_23)
    T_03 = T_00 @ T_01 @ T_12 @ T_23  # Matrix of Frame 3 -> Frame 0 (i.e., the world frame)

    # Matrix of Frame 4 (end effector)
    R_34 = RotationMatrix(phi4, axis_name='z')  # Rotation matrix
    p4 = np.array([[L3 + rad], [0.0], [0.0]])  # Frame's origin 
    t_34 = p4  # Translation vector
    T_34 = getLocalFrameMatrix(R_34, t_34)
    T_04 = T_00 @ T_01 @ T_12 @ T_23 @ T_34  # Matrix of Frame 4 ->Frame 0 (i.e., the world frame)

    # Calculate end-effector coordinates
    e = T_04[:3, 3]

    return T_01, T_02, T_03, T_04, e


def RotationMatrix(theta, axis_name):
    """ calculate single rotation of $theta$ matrix around x,y or z
        code from: https://programming-surgeon.com/en/euler-angle-python-en/
    input
        theta = rotation angle(degrees)
        axis_name = 'x', 'y' or 'z'
    output
        3x3 rotation matrix
    """

    c = np.cos(theta * np.pi / 180)
    s = np.sin(theta * np.pi / 180)
    
    if axis_name =='x':
        rotation_matrix = np.array([[1, 0,  0],
                                    [0, c, -s],
                                    [0, s,  c]])
    if axis_name =='y':
        rotation_matrix = np.array([[ c,  0, s],
                                    [ 0,  1, 0],
                                    [-s,  0, c]])
    elif axis_name =='z':
        rotation_matrix = np.array([[c, -s, 0],
                                    [s,  c, 0],
                                    [0,  0, 1]])
    return rotation_matrix



def createCoordinateFrameMesh():
    """Returns the mesh representing a coordinate frame
    Args:
      No input args
    Returns:
      F: vedo.mesh object (arrows for axis)

    """
    _shaft_radius = 0.05
    _head_radius = 0.10
    _alpha = 1

    # x-axis as an arrow
    x_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(1, 0, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='red',
                        alpha=_alpha)

    # y-axis as an arrow
    y_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 1, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='green',
                        alpha=_alpha)

    # z-axis as an arrow
    z_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 0, 1),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='blue',
                        alpha=_alpha)

    originDot = Sphere(pos=[0, 0, 0],
                       c="black",
                       r=0.10)

    # Combine the axes together to form a frame as a single mesh object
    F = x_axisArrow + y_axisArrow + z_axisArrow + originDot

    return F



def getLocalFrameMatrix(R_ij, t_ij): 
    """Returns the matrix representing the local frame
    Args:
      R_ij: rotation of Frame j w.r.t. Frame i 
      t_ij: translation of Frame j w.r.t. Frame i 
    Returns:
      T_ij: Matrix of Frame j w.r.t. Frame i. 
      
    """             
    # Rigid-body transformation [ R t ]
    T_ij = np.block([[R_ij,                t_ij],
                     [np.zeros((1, 3)),       1]])
    
    return T_ij


def main():
    # Set the limits of the graph x, y, and z ranges
    axes = Axes(xrange=(0, 20), yrange=(-2, 10), zrange=(0, 6))

    # Lengths of arm parts
    L1 = 5  # Length of link 1
    L2 = 8  # Length of link 2
    L3 = 3
    L4 = 0

    # Joint angles
    phi1 = -30  # Rotation angle of part 1 in degrees
    phi2 = 50  # Rotation angle of part 2 in degrees
    phi3 = 30  # Rotation angle of the end-effector in degrees

    # Matrix of Frame 1 (written w.r.t. Frame 0, which is the previous frame)
    R_01 = RotationMatrix(phi1, axis_name='z')  # Rotation matrix
    p1 = np.array([[3], [2], [0.0]])  # Frame's origin (w.r.t. previous frame)
    t_01 = p1  # Translation vector

    T_01 = getLocalFrameMatrix(R_01, t_01)  # Matrix of Frame 1 w.r.t. Frame 0 (i.e., the world frame)

    # coordinate frame mesh and transform
    Frame1Arrows = createCoordinateFrameMesh()

    # cylinder add to the local coordinate frame
    link1_mesh = Cylinder(r=0.4,
                          height=L1,
                          pos=(L1 / 2, 0, 0),
                          c="yellow",
                          alpha=.8,
                          axis=(1, 0, 0)
                          )

    # Also create a sphere to show as an example of a joint
    r1 = 0.4
    sphere1 = Sphere(r=r1).pos(-r1, 0, 0).color("gray").alpha(.8)  #changed to -r1 * 2

    # Combine all parts into a single object
    Frame1 = Frame1Arrows + link1_mesh + sphere1

    # Transform the part to position it at its correct location and orientation
    Frame1.apply_transform(T_01)

    # Matrix of Frame 2 (written w.r.t. Frame 1, which is the previous frame)
    R_12 = RotationMatrix(phi2, axis_name='z')  # Rotation matrix
    p2 = np.array([[L1], [0.0], [0.0]])  # Frame's origin (w.r.t. previous frame)
    t_12 = p2  # Translation vector

    # Matrix of Frame 2 w.r.t. Frame 1
    T_12 = getLocalFrameMatrix(R_12, t_12)

    # Matrix of Frame 2 w.r.t. Frame 0 (i.e., the world frame)
    T_02 = T_01 @ T_12

    # Create the coordinate frame mesh and transform
    Frame2Arrows = createCoordinateFrameMesh()

    # Now, let's create a cylinder and add it to the local coordinate frame
    link2_mesh = Cylinder(r=0.4,
                          height=L2,
                          pos=(L2 / 2, 0, 0),
                          c="red",
                          alpha=.8,
                          axis=(1, 0, 0)
                          )

    # Combine all parts into a single object
    Frame2 = Frame2Arrows + link2_mesh

    # Transform the part to position it at its correct location and orientation
    Frame2.apply_transform(T_02)

    # Matrix of Frame 3 (written w.r.t. Frame 2, which is the previous frame)
    R_23 = RotationMatrix(phi3, axis_name='z')  # Rotation matrix
    p3 = np.array([[L2], [0.0], [0.0]])  # Frame's origin (w.r.t. previous frame)
    t_23 = p3  # Translation vector

    # Matrix of Frame 3 w.r.t. Frame 2
    T_23 = getLocalFrameMatrix(R_23, t_23)

    # Matrix of Frame 3 w.r.t. Frame 0 (i.e., the world frame)
    T_03 = T_01 @ T_12 @ T_23

    # Create the coordinate frame mesh and transform. This point is the end-effector. So, I am
    # just creating the coordinate frame.
    Frame3 = createCoordinateFrameMesh()

    #Frame3.apply_transform(T_03).pos(T_03[:3, 3] - np.array([r1 * 2, 0, 0]))   # added t multiply radius by 2

    # Transform the part to position it at its correct location and orientation
    Frame3.apply_transform(T_03)

    # Show everything
    show([Frame1, Frame2, Frame3], axes, viewup="z").close()


if __name__ == '__main__':
    main()

