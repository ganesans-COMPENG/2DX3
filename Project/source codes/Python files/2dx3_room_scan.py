#Modify the following line with your own serial port details
#   Currently set COM3 as serial port at 115.2kbps 8N1
#   Refer to PySerial API for other options.  One option to consider is
#   the "timeout" - allowing the program to proceed if after a defined
#   timeout period.  The default = 0, which means wait forever.

import numpy as np
import open3d as o3d
import serial
import math
import time

s = serial.Serial('COM3 ', 115200, timeout=10)
print("Opening: " + s.name)

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()

rotations = int(input("Enter number of rotations: "))
print("\n be ready to quickly press onboard button PJ1 after each rotation\n")

# wait for user's signal to start the program
input("Press 's' to start communication...")
# send the character 's' to MCU via UART
# This will signal MCU to start the transmission
s.write('s'.encode())

text_file = open("measurements.txt", "w")

for j in range(rotations):
    print("\n press PJ1\n")
    for i in range(32):
        x = s.readline()
        print(x.decode())

        string = str(x)
        string = string.strip("b'")
        string = string[0:-4]         
            
        text_file.write(string+"\n")

    time.sleep(9)
       
text_file.close()

# the encode() and decode() function are needed to convert string to bytes
# because pyserial library functions work with type "bytes"

#close the port
print("Closing: " + s.name)
s.close()

if __name__ == "__main__":
    #Remember the goals of modularization
    #   -- smaller problems, reuse, validation, debugging
    #To simulate the data from the sensor lets create a new file with test data 
    f = open("coords.xyz", "w")    #create a new file for writing 
    text_file = open("measurements.txt", "r")
    #Test data: Lets make a rectangular prism as a point cloud in XYZ format
    #   A simple prism would only require 8 vertices, however we
    #   will sample the prism along its x-axis a total of 10x
    #   4 vertices repeated 10x = 40 vertices
    #   This for-loop generates our test data in xyz format
    for i in range(rotations):
        for x in range(32):

            string = text_file.readline()
            string = string.rstrip()
            x_coor = int(string) * math.cos(x*(math.pi/16))
            y_coor = int(string) * math.sin(x*(math.pi/16))
            z_coor = i*600            

            f.write('{0:f} {1:f} {2:f}\n'.format(x_coor,y_coor,z_coor))  #write x,y,z to file 

    f.close()   #there should now be a file containing all vertex coordinates                               
    
    #Read the test data in from the file we created        
    print("Read in the prism point cloud data (pcd)")
    pcd = o3d.io.read_point_cloud("coords.xyz", format="xyz")

    #Lets see what our point cloud data looks like numerically       
    print("The PCD array:")
    print(np.asarray(pcd.points))

    #Lets see what our point cloud data looks like graphically       
    print("Lets visualize the PCD: (spawns seperate interactive window)")
    o3d.visualization.draw_geometries([pcd])
    yz_slice_vertex = []
    for x in range(0,rotations*32):
        yz_slice_vertex.append([x])

    #Define coordinates to connect lines in each yz slice        
    lines = []  
    for x in range(rotations):
        for point in range(32):
            if point == 31:
                lines.append([yz_slice_vertex[(32*x) + point], yz_slice_vertex[(32*x)]])
            else:                
                lines.append([yz_slice_vertex[(32*x) + point], yz_slice_vertex[(32*x) + point + 1]])        

    #Define coordinates to connect lines between current and next yz slice     
    for x in range(rotations - 1):
        for plane in range(32):
            lines.append([yz_slice_vertex[(32*x) + plane], yz_slice_vertex[(32*x) + plane + 32]])        
    
    #This line maps the lines to the 3d coordinate vertices
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

    #Lets see what our point cloud data with lines looks like graphically       
    o3d.visualization.draw_geometries([line_set])
