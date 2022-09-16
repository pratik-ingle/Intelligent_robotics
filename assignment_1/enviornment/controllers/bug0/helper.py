
import numpy as np

class Helper():
    
    def get_heading_angle(self,R):
        """ 
        Arguments:
        ----------
        R: orientation matrix

        Returns:
        --------
        theta_z = angle normal to z axis
        """
        theta_z = np.arctan2(R[3], R[0])
        angle = np.degrees(theta_z)
        # if angle < 0:
            # angle = 360 + angle
        return angle

    def angel_line_horizontal(self,p1, p2):
        """ 
        Arguments:
        ----------
        p1, p2: two pints on a line

        Returns:
        --------
        angle: degree
        """
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        theta = np.arctan2(dy, dx)
        angle = np.degrees(theta)  # angle is in (-180, 180]
        # if angle < 0:
            # angle = 360 + angle
        return angle
    
    def line_bw_points(self,A, B):
        """
        Parameters:
        ----------
        A: first point, tuple
        B: second point, tuple
        Returns:
        --------
        tuple: (x coefficient, y coeffiecient, slop of line)
        """
        x_coff = B[1] - A[1] 
        y_coff = A[0] - B[0]
        c = x_coff*A[0] + y_coff*A[1]
        # print(f"the line b/w {A} and {B} is {x_coff}x + {y_coff}y = {c}")
        return (x_coff, y_coff, c)


    def perpendicular_dis(self, A, line):
        """
        Arguments:
        ----------
        A: a point, tuple
        line: points of lines, tuple
        Returns:
        --------
        dis: float
        """
        return abs((line[0] * A[0] + line[1] * A[1] + line[2])) / np.sqrt(np.square(line[0]) + np.square(line[1]))
        
    def distance_bw_points(self,A, B):
        """
        Arguments:
        ----------
        A: first point, tuple
        B: second point, tuple
        Returns:
        --------
        dist: float, distance 
        """
        A = np.array(A)
        B = np.array(B)
    
        return np.linalg.norm(A - B)
            
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        