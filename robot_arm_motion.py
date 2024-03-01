import numpy as np
from vedo import *

def RotationMatrix(theta, axis_name):
    """ calculate single rotation of theta matrix around x,y or z """
    c = np.cos(theta * np.pi / 180)
    s = np.sin(theta * np.pi / 180)
    if axis_name =='x':
        rotation_matrix = np.array([[1, 0,  0],
                                    [0, c, -s],
                                    [0, s,  c]])
    elif axis_name =='y':
        rotation_matrix = np.array([[ c,  0, s],
                                    [ 0,  1, 0],
                                    [-s,  0, c]])
    elif axis_name =='z':
        rotation_matrix = np.array([[c, -s, 0],
                                    [s,  c, 0],
                                    [0,  0, 1]])
    return rotation_matrix

def createCoordinateFrameMesh():
    """Returns the mesh representing a coordinate frame"""
    _shaft_radius = 0.05
    _head_radius = 0.10
    _alp = 1
    
    # x-axis as an arrow  
    x_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(1, 0, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='red',
                        alpha=_alp)
    # y-axis as an arrow  
    y_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 1, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='green',
                        alpha=_alp)

    # z-axis as an arrow  
    z_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 0, 1),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='blue',
                        alpha=_alp)
    
    originDot = Sphere(pos=[0,0,0], 
                       c="black", 
                       r=0.10)

    # Combine the axes together to form a frame as a single mesh object 
    F = x_axisArrow + y_axisArrow + z_axisArrow + originDot
    return F

def getLocalFrameMatrix(R_ij, t_ij): 
    """Returns the matrix representing the local frame"""            
    # Rigid-body transformation [ R t ]
    T_ij = np.block([[R_ij,                t_ij],
                     [np.zeros((1, 3)),       1]])
    return T_ij

def main():
    # Set the limits of the graph x, y, and z ranges 
    vp = Plotter(axes=0, interactive=False)

    # Lengths of arm parts 
    L1 = 5   # Length of link 1
    L2 = 8   # Length of link 2

    # Joint angles 
    phi1_start = 30     # Start rotation angle of part 1 in degrees
    phi1_end = 150      # End rotation angle of part 1 in degrees
    phi2_start = -10    # Start rotation angle of part 2 in degrees
    phi2_end = 50       # End rotation angle of part 2 in degrees
    phi3 = 0            # Rotation angle of the end-effector in degrees
    
    steps = 100  # Number of interpolation steps
    frame_rate = 30  # Frame rate of the video


    phi1_values = np.linspace(phi1_start, phi1_end, steps)
    phi2_values = np.linspace(phi2_start, phi2_end, steps)
    
    base = Sphere(r=0.2).color("gray").alpha(0.5).pos(0, 0, -0.1)
    vp += base
    
    for phi1, phi2 in zip(phi1_values, phi2_values):
        show_everything(phi1, phi2, phi3, L1, L2, vp)
        vp.show(interactive=False)  # Display the plot without blocking
        
        
    vp.close()

def show_everything(phi1, phi2, phi3, L1, L2, vp):
    vp.clear()
    
    base = Sphere(r=0.2).color("gray").alpha(0.5).pos(0, 0, -0.1)  # Place base in xy-plane
    vp += base
    
    R_01 = RotationMatrix(phi1, axis_name='z')
    p1 = np.array([[0],[0], [-0.1]])  # Place Frame 1 in xy-plane
    t_01 = p1
    T_01 = getLocalFrameMatrix(R_01, t_01)
    Frame1Arrows = createCoordinateFrameMesh()
    link1_mesh = Cylinder(r=0.4, height=L1, pos=(L1/2,0,-0.1), c="yellow", alpha=.8, axis=(1,0,0))
    sphere1 = Sphere(r=0.4).pos(-0.4,0,-0.1).color("gray").alpha(.8)
    Frame1 = Frame1Arrows + link1_mesh + sphere1
    Frame1.apply_transform(T_01)
    
    R_12 = RotationMatrix(phi2, axis_name='z')
    p2 = np.array([[L1],[0.0], [-0.1]])  # Place Frame 2 in xy-plane
    t_12 = p2
    T_12 = getLocalFrameMatrix(R_12, t_12)
    T_02 = T_01 @ T_12
    Frame2Arrows = createCoordinateFrameMesh()
    link2_mesh = Cylinder(r=0.4, height=L2, pos=(L2/2,0,-0.1), c="red", alpha=.8, axis=(1,0,0))
    Frame2 = Frame2Arrows + link2_mesh
    Frame2.apply_transform(T_02)

    R_23 = RotationMatrix(phi3, axis_name='z')
    p3 = np.array([[L2],[0.0], [-0.1]])  # Place Frame 3 in xy-plane
    t_23 = p3
    T_23 = getLocalFrameMatrix(R_23, t_23)
    T_03 = T_01 @ T_12 @ T_23
    Frame3 = createCoordinateFrameMesh()
    Frame3.apply_transform(T_03)

    vp += Frame1
    vp += Frame2
    vp += Frame3

if __name__ == '__main__':
    main()

