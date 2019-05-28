import sys
from lxml import etree

# Splitting up the responsibilities for the MeshGenerator.Generator so it doesn't become a God object anti-pattern
pose_template = "{} {} {} {} {} {}"
wall_template = "wall_{}"
vec2_template = "{} {}"
vec3_template = "{} {} {}"
vec4_template = "{} {} {} {}"


def create_element(el_name, _text=None, **kwargs):
    el = etree.Element(el_name, **kwargs)
    el.text = _text
    return el


def create_plane():
    pass


class SDFGenerator:
    def __init__(self, filename="assets/output.sdf", scale=1.):
        self.filename = filename
        self.scale = scale
        self.sdf = etree.Element("sdf", version="1.6")
        self.world = etree.Element("world", name="building_model")
        self.walls_model = etree.Element("model", name="walls")
        self.floor_model = etree.Element("model", name="floors")
        self.sdf.append(self.world)
        self.world.append(self.walls_model)
        self.world.append(self.floor_model)
        print("Scale set to:", scale)

    def write_file(self):
        file = open(self.filename, 'w')
        file.write(etree.tostring(self.sdf, xml_declaration=True, pretty_print=True).decode('utf-8'))
        file.close()

    def add_walls(self, pos, walls_obj_file):
        self.walls_model.append(create_element("static", _text="1"))
        self.walls_model.append(create_element("pose", frame="walls_frame",
                                               _text=pose_template.format(pos[0], pos[1], pos[2], 0., 0., 0.)))

        # Write the visual link
        link = create_element("link", name="walls_link")
        self.walls_model.append(link)
        inertial = create_element("inertial")
        link.append(inertial)

        inertial.append(create_element("mass", _text="0"))

        # Add the inertia tensor
        inertia = create_element("inertia")
        inertial.append(inertia)
        inertia.append(create_element("ixx", _text="1."))
        inertia.append(create_element("ixy", _text="0."))
        inertia.append(create_element("ixz", _text="0."))
        inertia.append(create_element("iyy", _text="1."))
        inertia.append(create_element("iyz", _text="0."))
        inertia.append(create_element("izz", _text="1."))

        visual = create_element("visual", name="walls_mesh")
        link.append(visual)
        geometry = create_element("geometry")
        visual.append(geometry)
        mesh = create_element("mesh")
        geometry.append(mesh)
        scale = self.scale
        mesh.append(create_element("scale", _text=vec3_template.format(scale, scale, scale)))
        mesh.append(create_element("uri", _text=walls_obj_file))

        material = create_element("material")
        visual.append(material)
        material.append(create_element("lighting", _text=str(1)))
        material.append(create_element("ambient", _text=vec4_template.format(.3, .3, .3, 1.)))
        material.append(create_element("diffuse", _text=vec4_template.format(.6, .6, .6, 1.)))
        material.append(create_element("specular", _text=vec4_template.format(.3, .3, .3, 1.)))

        collision = create_element("collision", name="walls_collision")
        link.append(collision)
        geometry = create_element("geometry")
        collision.append(geometry)
        mesh = create_element("mesh")
        geometry.append(mesh)
        scale = self.scale
        mesh.append(create_element("scale", _text=vec3_template.format(scale, scale, scale)))
        mesh.append(create_element("uri", _text=walls_obj_file))

    def add_floors(self, pos, floor_obj_file):
        self.floor_model.append(create_element("static", _text="1"))
        self.floor_model.append(create_element("pose", frame="floors_frame",
                                               _text=pose_template.format(pos[0], pos[1], pos[2], 0., 0., 0.)))

        # Write the visual link
        link = create_element("link", name="floors_link")
        self.floor_model.append(link)
        inertial = create_element("inertial")
        link.append(inertial)

        inertial.append(create_element("mass", _text="0"))

        # Add the inertia tensor
        inertia = create_element("inertia")
        inertial.append(inertia)
        inertia.append(create_element("ixx", _text="1."))
        inertia.append(create_element("ixy", _text="0."))
        inertia.append(create_element("ixz", _text="0."))
        inertia.append(create_element("iyy", _text="1."))
        inertia.append(create_element("iyz", _text="0."))
        inertia.append(create_element("izz", _text="1."))

        visual = create_element("visual", name="floors_mesh")
        link.append(visual)
        geometry = create_element("geometry")
        visual.append(geometry)
        mesh = create_element("mesh")
        geometry.append(mesh)
        scale = self.scale
        mesh.append(create_element("scale", _text=vec3_template.format(scale, scale, scale)))
        mesh.append(create_element("uri", _text=floor_obj_file))

        material = create_element("material")
        visual.append(material)
        material.append(create_element("lighting", _text=str(1)))
        material.append(create_element("ambient", _text=vec4_template.format(.3, .3, .3, 1.)))
        material.append(create_element("diffuse", _text=vec4_template.format(.6, .6, .6, 1.)))
        material.append(create_element("specular", _text=vec4_template.format(.3, .3, .3, 1.)))

        collision = create_element("collision", name="floors_collision")
        link.append(collision)
        geometry = create_element("geometry")
        collision.append(geometry)
        mesh = create_element("mesh")
        geometry.append(mesh)
        scale = self.scale
        mesh.append(create_element("scale", _text=vec3_template.format(scale, scale, scale)))
        mesh.append(create_element("uri", _text=floor_obj_file))

    def add_extra(self):
        """This extra stuff is required to load the SDF into Gazebo for testing"""
        light = create_element("light", type="directional", name="sun")
        self.world.append(light)
        light.append(create_element("pose", frame='', _text=pose_template.format(0, 0, 10, 0, 0, 0)))
        light.append(create_element("diffuse", _text=vec4_template.format(.8, .8, .8, 0)))
        light.append(create_element("specular", _text=vec4_template.format(.2, .2, .2, 0)))
        atten = create_element("attenuation")
        light.append(atten)
        atten.append(create_element("range", "1000"))
        atten.append(create_element("constant", "1."))
        atten.append(create_element("linear", ".1"))
        atten.append(create_element("quadratic", ".001"))
        light.append(create_element("direction", _text=vec3_template.format(-1., -1., -1.)))

        self.world.append(create_element("gravity", vec3_template.format(0, 0, -9.8)))
        self.world.append(create_element("magnetic_field", vec3_template.format(6e-06, 2.3e-05, -4.2e-05)))
        self.world.append(create_element("atmosphere", type="adiabatic"))
        physics = create_element("physics", name="default_physics", default="0", type="ode")
        self.world.append(physics)
        physics.append(create_element("max_step_size", _text=".001"))
        physics.append(create_element("real_time_factor", _text="1"))
        physics.append(create_element("real_time_update_rate", _text="1000"))
        scene = create_element("scene")
        self.world.append(scene)
        scene.append(create_element("ambient", vec4_template.format(.6, .6, .6, 1.)))
        scene.append(create_element("background", vec4_template.format(.7, .7, .7, 1.)))
        scene.append(create_element("shadows", _text="1"))
        self.world.append(create_element("wind"))

        coords = create_element("spherical_coordinates")
        self.world.append(coords)
        coords.append(create_element("surface_model", _text="EARTH_WGS84"))
        coords.append(create_element("latitude_deg", _text="0"))
        coords.append(create_element("longitude_deg", _text="0"))
        coords.append(create_element("elevation", _text="0"))
        coords.append(create_element("heading_deg", _text="0"))

        state = create_element("state", world_name="default")
        self.world.append(state)
        state.append(create_element("sim_time", vec2_template.format(0, 0)))
        state.append(create_element("real_time", vec2_template.format(0, 0)))
        state.append(create_element("wall_time", vec2_template.format(0, 0)))
        state.append(create_element("iterations", _text="0"))

        light = create_element("light", name="sun")
        state.append(light)
        light.append(create_element("pose", frame="", _text=pose_template.format(0, 0, 0, 0, 0, 0)))

        gui = create_element("gui", fullscreen="0")
        self.world.append(gui)
        cam = create_element("camera", name="user_camera")
        gui.append(cam)

        cam.append(create_element("pose", frame="", _text=pose_template.format(14.0123, -16.1314, 2.86746, 0, 0.275643, 2.35619)))
        cam.append(create_element("view_controller", _text="orbit"))
        cam.append(create_element("projection_type", _text="perspective"))
