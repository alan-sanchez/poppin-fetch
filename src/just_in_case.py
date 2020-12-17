#!/usr/bin/env python

# Import what we need
import rospy
import sys
import math
import numpy as np

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from scipy.spatial import ConvexHull, Delaunay
from sympy import solve, Poly, Eq, Function, exp, Symbol


class Convex_hull:
    def __init__(self):
        # Initialize Subscribers
        self.clicked_point_sub = rospy.Subscriber('clicked_point', PointStamped, self.makeInteractiveMarker)

        # Initialize Publishers
        self.convex_hull_pub = rospy.Publisher('convex_hull',PolygonStamped, queue_size=1)

        # Setup header for Interactive Markers (IM)
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        # Setup PolygonStamped for convex hull
        self.poly = PolygonStamped()
        self.poly.header = self.header

        # create an IM server on the topic namespace interactive_marker
        self.server = InteractiveMarkerServer("interactive_marker")

        # Initialize a lists for the projected points of each IM.
        self.proj_x = []
        self.proj_y = []
        self.proj_z = []

        # Initialize id of IM's.
        self.id = 0

        # Set X,Y, and Z lists for the IM positions
        self.X = []
        self.Y = []
        self.Z = []

        # Initialize coefficent values.Equation: ax+by+cz=d. Normal Vector: <a,b,c>
        self.a = None
        self.b = None
        self.c = None

        # Initialize the unit normal vector.
        self.u_n = None

        # Initialize the 3D to 2D sub-plane Matrix, M, and the Inverse.
        self.M = None
        self.M_inv = None

    # processFeedback gets called when an IM's position is changed
    def processFeedback(self,feedback):
        # update the clicked_points_3D list and recalculate the convex_hull
        p = feedback.pose.position
        id = int(feedback.marker_name)
        self.X[id] = p.x
        self.Y[id] = p.y
        self.Z[id] = p.z
        if self.id > 1:
            self.best_fit_plane()


    def makeInteractiveMarker(self, msg):
        # store x,y, and z point values from subscribed clicked points.
        self.X.append(msg.point.x)
        self.Y.append(msg.point.y)
        self.Z.append(msg.point.z)

        # create an IM for our server
        int_marker = InteractiveMarker()
        int_marker.header = self.header
        int_marker.scale = 0.2
        int_marker.name = str(self.id)
        int_marker.pose.position = msg.point

        # create a white sphere marker
        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = int_marker.scale*0.5
        sphere_marker.scale.y = int_marker.scale*0.5
        sphere_marker.scale.z = int_marker.scale*0.5
        sphere_marker.color.r = 1
        sphere_marker.color.g = 1
        sphere_marker.color.b = 1
        sphere_marker.color.a = 1

        # create an interactive control which contains the sphere and allows the
        # marker to move in any direction when you click and drag (MOVE_3D)
        sphere_control = InteractiveMarkerControl()
        sphere_control.always_visible = True
        sphere_control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        sphere_control.markers.append(sphere_marker)

        # add the control to the IM
        int_marker.controls.append(sphere_control)

        # create interactiver control to move linearly in the x direction
        sphere_control = InteractiveMarkerControl()
        sphere_control.orientation.w = 1
        sphere_control.orientation.x = 1
        sphere_control.orientation.y = 0
        sphere_control.orientation.z = 0
        sphere_control.name = "move_x"
        sphere_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(sphere_control)

        # create interactiver control to move linearly in the z direction
        sphere_control = InteractiveMarkerControl()
        sphere_control.orientation.w = 1
        sphere_control.orientation.x = 0
        sphere_control.orientation.y = 1
        sphere_control.orientation.z = 0
        sphere_control.name = "move_z"
        sphere_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(sphere_control)

        # create interactiver control to move linearly in the y direction
        sphere_control = InteractiveMarkerControl()
        sphere_control.orientation.w = 1
        sphere_control.orientation.x = 0
        sphere_control.orientation.y = 0
        sphere_control.orientation.z = 1
        sphere_control.name = "move_y"
        sphere_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(sphere_control)

        # Add the IM to our collection & tell the server to call processFeedback()
        # when feedback arrives for it
        self.server.insert(int_marker, self.processFeedback)

        # 'commit' changes and send to all clients
        self.server.applyChanges()

        # Run a convex hull once there are 3 or more clicked points
        if self.id > 1:
            self.best_fit_plane()

        # update the IM id for next clicked point.
        self.id += 1


    def best_fit_plane(self):
        # coordinates (XYZ) of the IM's
        XYZ = np.array([self.X,self.Y,self.Z])

        # Begin plane fit computation. Reference:https://stackoverflow.com/questions/12299540/
        tmp_A = []
        tmp_b = []
        for i in range(len(self.X)):
            tmp_A.append([self.X[i], self.Y[i], 1])
            tmp_b.append(self.Z[i])
        b = np.matrix(tmp_b).T
        A = np.matrix(tmp_A)
        fit = (A.T * A).I * A.T * b
        errors = b - A * fit

        # Equation: fit[0]*x + fit[1]*y + fit[2] = z
        # assign a,b, and c values by rearranging equation.
        # Rearranged Eq. : (-fit[0]/fit[2])*x + (-fit[1]/fit[2])*y + (1/fit[2])*z = 1
        self.a = float(-fit[0]/fit[2])
        self.b = float(-fit[1]/fit[2])
        self.c = float(1/fit[2])
        #print("%f x + %f y + %f = z" % (fit[0], fit[1], fit[2]))
        #print("Normal vector: < {0}, {1}, {2}> ".format(self.a,self.b,self.c))

        self.project_on_plane()


    def project_on_plane(self):
        # Create an origin point by using x and y pose of the first IM and solve for z.
        orig_x = self.X[0]
        orig_y = self.Y[0]
        orig_z = self.solve_for_z(orig_x, orig_y)

        # Create unit normal vector
        mag = math.sqrt(self.a**2 + self.b**2 + self.c**2)
        u_a = self.a / mag
        u_b = self.b / mag
        u_c = self.c / mag
        self.u_n = np.array([u_a, u_b, u_c])

        # Create a list of the projected points
        # proj_points = []

        # forloop to compute the projections of each IM
        for i in range(len(self.X)):
            # Make a vector from the origin points to the IM's location.
            # v = Interactive_maker_location - orig (with respect to each dimension)
            vx = self.X[i] - orig_x
            vy = self.Y[i] - orig_y
            vz = self.Z[i] - orig_z
            v  = np.array([vx,vy,vz])

            # Take dot product of vector <vx,vy,vz> and unit normal vector <nx,ny,nz>
            # to get the scalar distance from the IM to the plane along the normal.
            dist =np.matmul(v,self.u_n)

            # Multiply the unit normal vector by the dist, and subtract from the IM pose.
            # Append to appropriate lists.
            self.proj_x.append(self.X[i] - dist*self.u_n[0])
            self.proj_y.append(self.Y[i] - dist*self.u_n[1])
            self.proj_z.append(self.Z[i] - dist*self.u_n[2])

        self.transfer_matrix()


    def solve_for_z(self,x_value,y_value):
        z = Symbol('z')
        z_value = solve(self.a*x_value + self.b*y_value + self.c*z - 1, z)
        return float(z_value[0])


    def transfer_matrix(self):
        # Create a vector in the bestfit plane using the first two projected points
        ABx = self.proj_x[0] - self.proj_x[1]
        ABy = self.proj_y[0] - self.proj_y[1]
        ABz = self.proj_z[0] - self.proj_z[1]

        # Normalize the componentes of the vector AB
        mag = math.sqrt(ABx**2 + ABy**2 + ABz**2)
        u_ABx = ABx / mag
        u_ABy = ABy / mag
        u_ABz = ABz / mag
        u_AB  = np.array([u_ABx, u_ABy, u_ABz])

        # Create second base vector (it is in unit and lies in the best fit plane)
        # Perform cross-product of u_AB and u_n
        V = np.cross(u_AB,self.u_n)

        # Now we can calculate the four basis points A, u=A+u_AB, v=A+V, n=A+u_n
        Ax = self.proj_x[1]
        Ay = self.proj_y[1]
        Az = self.proj_z[1]

        ux = Ax + u_ABx
        uy = Ay + u_ABy
        uz = Az + u_ABz

        vx = Ax + V[0]
        vy = Ay + V[1]
        vz = Az + V[2]

        nx = Ax + self.u_n[0]
        ny = Ay + self.u_n[1]
        nz = Az + self.u_n[2]

        # Set S matrix
        S = np.array([[Ax, ux, vx, nx],
                      [Ay, uy, vy, ny],
                      [Az, uz, vz, nz],
                      [1 , 1 , 1 , 1 ]])

        # Transformation maps into matrix below (D)
        D = np.array([[0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1],
                      [1, 1, 1, 1]])

        # Inverse matrix of S
        S_inv = np.linalg.inv(S)

        # Transform Matrix!
        self.M = np.matmul(D,S_inv)

        # Inverse of Tranform Matrix
        self.M_inv = np.linalg.inv(self.M)

        print(self.M)


        # self.dimension_reduction()

    def dimension_reduction(self):
        # Create an array to store all of the 2D_sub plane coordinates
        coordinates_2D = np.empty(shape=[len(self.X)])

        # for i in range(len(self.X)):
        #     IM_proj = np.array([self.X[i], self.Y[i], self.Z[i], 1])
        #     reduction = np.matmul(M,IM_proj)


#s = np.array([[1,2,1,1], [1,1,1,2], [1,1,2,1], [1,1,1,1]])
# ss = np.linalg.inv(s)
#d = np.array([[0,1,0,0],[0,0,1,0], [0,0,0,1], [1,1,1,1]])
#m = np.matmul(d,ss)
#A = np.array([1,1,1,1])
#aa = np.matmul(M,A)








if __name__=="__main__":
    # Initialize the node
    rospy.init_node('convex_hull')
    Convex_hull()
    rospy.spin()
