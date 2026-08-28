import math

# 1. INSERT YOUR CORNERS HERE (x, y) in meters
# To close the polygon, make sure the last coordinate is the same as the first.
corners = [
    (2.0, 4.2),   #1.
    (1.0, 4.2),   #2.
    (-0.2, 4.2),   #3.
    (1.5, 4.2),   #4.
    (-2.0, 3.0),   #5.
    (-2.3, 2.0),   #6.
    (-2.5, 0.5),   #7.
    (-2.5, 0.0),   #8.
    (-2.5, -0.2),   #9.
    (-2.5, -0.7),   #10.
    (-2.5, -1.0),   #11.
    (-2.0, -1.5),   #12.
    (-1.2, -2.0),   #13.
    (-0.5, -2.5),   #14.
    (0.5, -2.5),   #15.
    (0.9, -2.2),   #16.
    (1.5, -1.2),   #17.
    (2.0, -0.6),   #18.
    (2.2, 0.0),   #19.
    (2.0, 0.9),   #20.
    (1.8, 1.5),     #21.
    (2.0, 2.5),     #22.
    (2.0, 4.2),     #23. Close the polygon by repeating the first point
            

]

line_width = 0.1
thickness = 0.001

print('<!-- Paste this entire block into your .world file -->')
print('<model name="yellow_polygon_line">')
print('  <static>true</static>')

for i in range(len(corners) - 1):
    x1, y1 = corners[i]
    x2, y2 = corners[i+1]
    
    # Calculate length and center point of the line segment
    dx = x2 - x1
    dy = y2 - y1
    length = math.sqrt(dx**2 + dy**2)
    cx = (x1 + x2) / 2.0
    cy = (y1 + y2) / 2.0
    
    # Calculate rotation angle (yaw)
    yaw = math.atan2(dy, dx)
    
    print(f'  <link name="segment_{i}">')
    print(f'    <pose>{cx:.3f} {cy:.3f} {thickness} 0 0 {yaw:.3f}</pose>')
    print('    <visual name="visual">')
    print('      <geometry>')
    print(f'        <box><size>{length:.3f} {line_width} {thickness}</size></box>')
    print('      </geometry>')
    print('      <material>')
    print('        <ambient>1.0 1.0 0.0 1.0</ambient>')
    print('        <diffuse>1.0 1.0 0.0 1.0</diffuse>')
    print('      </material>')
    print('    </visual>')
    print('  </link>')

print('</model>')