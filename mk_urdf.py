#!/usr/bin/env python3

class URDFPrinter():
  header = """
<?xml version="1.0" ?>

<robot name="%(name)s">

  <!-- Colors -->
  <material name="Black">
    <color rgba="0 0 0 1.0"/>
  </material>
  <material name="Grey">
    <color rgba="0.2 0.2 0.2 1.0"/>
  </material>
  <material name="Orange">
    <color rgba="1.0 0.423529411765 0.0392156862745 1.0"/>
  </material>
  <material name="Blue">
  <color rgba="0.0 0.0 1 1.0"/>      
  </material>
  <material name="Red">
    <color rgba="1 0 0 1.0"/>      
  </material>

  <!-- Base Link -->
  <link name="%(base_name)s">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="100"/>
      <inertia ixx="9.3" ixy="0" ixz="0" iyy="9.3" iyz="0" izz="9.3"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="1 1 0" />
      </geometry>
      <material name="black"/>
    </visual>
  </link>
  """

  section_template = """
  <!-- Section %(index)d -->

  <joint name="Joint_%(index)da" type="revolute">
    <parent link="%(parent)s"/>
    <child link="Block%(index)d"/>
    <origin rpy="0 0 0" xyz="0 0 %(joint_z)f"/>
    <axis xyz="1 0 0"/>
    <limit effort="1000.0" lower="0.0" upper="0.548" velocity="0.5"/>    
  </joint>

  <link name="Block%(index)d">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="5"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="%(block_size)s"/>
      </geometry>
      <material name="Grey"/>
    </visual>
  </link>

  <joint name="Joint_%(index)db" type="revolute">
    <parent link="Block%(index)d"/>
    <child link="Plate%(index)d"/>
    <origin rpy="0 0 0" xyz="0 0 %(joint_z)f"/>
    <axis xyz="0 1 0"/>
    <limit effort="1000.0" lower="0.0" upper="0.548" velocity="0.5"/>    
  </joint>

  <link name="Plate%(index)d">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="5"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder radius="%(plate_radius)f" length="0.01"/>
      </geometry>
      <material name="%(plate_color)s"/>
    </visual>
  </link>
  """

  footer = """
  </robot>
  """

  base_name = "SprutBase"

  def print_manipulator(self, f):
    self.print_header(f, "Manipulator")
    self.print_manipulator_body(f)
    self.print_footer(f)

  def print_cage(self, f):
    self.print_header(f, "Cage")
    self.print_cage_body(f)
    self.print_footer(f)

  def print_target(self, f):
    self.print_header(f, "Target")
    self.print_target_body(f)
    self.print_footer(f)

  def print_header(self, f, name):
    print(self.header % {"base_name": self.base_name, 'name': name}, file=f)

  def print_footer(self, f):
    print(self.footer,file = f)

  def print_manipulator_body(self, f):
    for i in range(16):
      if i > 0:
        parent = "Plate%d" % (i-1)
      else:
        parent = self.base_name

      if (i + 1) % 4 == 0:
        plate_color = "Orange"
      else:
        plate_color = "Grey"

      print(self.section_template % {
        'parent': parent, 'index': i,
        'block_size': "0.05 0.05 0.025",
        'plate_color': plate_color, 'plate_radius': 0.2,
        'joint_z': 0.05}, file=f)

  def print_cage_body(self, f):
    print("<!-- fixme cage -->", file=f)

  def print_target_body(self, f):
    print("<!-- fixme target -->", file=f)


p = URDFPrinter()
p.print_manipulator(open("manipulator.urdf", "w"))
p.print_target(open("target.urdf", "w"))
p.print_cage(open("cage.urdf", "w"))
