


cols = 8
rows = 1
chars = 13
size = 0.5
spacing = 0.8

digits = [
    'digit0',
    'digit1',
    'digit2',
    'digit3',
    'digit4',
    'digit5',
    'digit6',
    'digit7',
    'digit8',
    'digit9',
    'digitdot',
    'digitdash',
    'digitsp',
    ]

model = '''
<sdf version="1.5">
  <model name="scoreboard">
    <static>true</static>
'''

for r in range(rows):
    for c in range(cols):
        for d in range(chars):
            idx = '%02d%02d%02d' % (r, c, d)
            xp = c * size * spacing
            yp = r * size
            digit = digits[d]
            model += f'''
<link name="{idx}"><pose>{xp} {yp} 0 0 0 1.5708</pose><visual name="{idx}"><material><script><uri>file://media/materials/scripts/digit.material</uri><name>{digit}</name></script></material><geometry><box><size>{size} {size} 0.00011</size></box></geometry></visual></link>'''
model += '''
    <plugin name="scoreboard_controller" filename="libgazebo_ros_text.so"> </plugin>
  </model>
</sdf>
'''


with open('src/dots_gazebo/dots_sim/models/scoreboard/model.sdf', 'w') as f:
    f.write(model)


