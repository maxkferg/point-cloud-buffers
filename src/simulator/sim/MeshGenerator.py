import sys
from PIL import Image
import numpy as np
import math
import random
import SDFGenerator

'''Lines are based on the assumption that they are horizontal or vertical and
therefore are identified by a range over x or y and a single value for the coordinate
axis thus:

Vertical Lines   = [ x,              [y_min, y_max] ]
Horizontal Lines = [ [x_min, x_max], y              ]'''


def is_horizontal(line): return isinstance(line, list) and isinstance(line[0], list)


def is_vertical(line): return isinstance(line, list) and isinstance(line[1], list)


def start_pos(line):
    return [line[0][0], line[1]] if is_horizontal(line) else [line[0], line[1][0]]


def end_pos(line):
    return [line[0][1], line[1]] if is_horizontal(line) else [line[0], line[1][1]]


def centre_pos(line):
    SP = start_pos(line)
    EP = end_pos(line)
    return [(SP[0] + EP[0])/2., (SP[1] + EP[1])/2.]


def line_len(line):
    return line[0][1] - line[0][0] if is_horizontal(line) else line[1][1] - line[1][0]


# Basically the 2D perp operator
def turn_left(normal):
    return [-normal[1], normal[0]]


# And the inverse thereof
def turn_right(normal):
    return [normal[1], -normal[0]]


def get_angle(normal):
    if normal[0] == 0:
        if normal[1] == 1: return math.pi
        elif normal[1] == -1: return 0.
    elif normal[1] == 0:
        if normal[0] == 1: return math.pi/2.
        elif normal[0] == -1: return 3.*math.pi/2.


def compute_normal(neigh_line, neigh_normal, curr_line):
    nSP = start_pos(neigh_line)
    nEP = end_pos(neigh_line)
    cSP = start_pos(curr_line)
    cEP = end_pos(curr_line)

    if is_horizontal(curr_line):
        if cSP == nEP: return turn_right(neigh_normal)
        elif cSP == nSP: return turn_left(neigh_normal)
        elif cEP == nEP: return turn_left(neigh_normal)
        elif cEP == nSP: return turn_right(neigh_normal)
    elif is_vertical(curr_line):
        if cSP == nEP: return turn_left(neigh_normal)
        elif cSP == nSP: return turn_right(neigh_normal)
        elif cEP == nEP: return turn_right(neigh_normal)
        elif cEP == nSP: return turn_left(neigh_normal)

    return []


def make_translation(pos):
    return np.array([[1., 0., 0., pos[0]],
                     [0., 1., 0., pos[1]],
                     [0., 0., 1., pos[2]],
                     [0., 0., 0., 1.]])


def make_scale(scale):
    return np.array([[scale[0], 0.,       0.,       0.],
                     [0.,       scale[1], 0.,       0.],
                     [0.,       0.,       scale[2], 0.],
                     [0.,       0.,       0.,       1.]])


# Adapted to affine version from https://stackoverflow.com/questions/6802577/rotation-of-3d-vectors
def make_rotation(axis, theta):
    axis = np.asarray(axis)
    axis = axis/math.sqrt(np.dot(axis, axis))
    a = math.cos(theta/2.0)
    b, c, d = -axis*math.sin(theta/2.0)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return np.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac), 0.],
                     [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab), 0.],
                     [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc, 0.],
                     [0., 0., 0., 1.]])


def get_rand3(minimum, maximum):
    return (random.randint(minimum, maximum+1), random.randint(minimum, maximum+1), random.randint(minimum, maximum+1))


# Define the rect type and related functions
# A rect is defined by [ [x_min, y_min], [x_max, y_max] ]

def is_rect(rect): return isinstance(rect, list) and isinstance(rect[0], list) and isinstance(rect[1], list)


def bottom_left(rect): return rect[0]


def bottom_right(rect): return [rect[1][0], rect[0][1]]


def top_left(rect): return [rect[0][0], rect[1][1]]


def top_right(rect): return rect[1]


# Notes:
# 1) All lines are vertical or horizontal
# 2) Any corner in the image must be split into two lines
# 3) A line can be queried by a fixed column or row and a start and an end pixel (pixels in between are assumed to have
#    been processed.
# 4) A line is defined by:
#       horizontal: [ [ x_min, x_max ], y               ]
#       vertical:   [ x,                [ y_min, y_max] ]
# 5) Lines are held in different containers, first search the horizontal by y or vertical by x, if it contains lines in
#    fixed column or row, search the [ min, max ] values to determine if it can be appended or must be split
# 6) A line [ x, [ y, y ] ] is valid (a single pixel)
# 7) There is no need to check if two lines merge due to the image being processed top to bottom, left to right, in that
#    order, meaning that we first try to join verticals and then horizontals.


# Compute the normals of the walls
#
# Notes:
# 1) Seed the first two lines with the correct normal
# 2) For each line, find all connectors and rotate normal accordingly
# 3) If no connector is found, assume must be interior wall, if vertical, normal is -1 if horizontal, normal is -1


# The outside wall is a connected graph so we can traverse it and induce the first normal to derive the rest
# Inner walls will have to be induced by making the assumption that the next line without a normal must be
# a vertical line (lines = verticals + horizontals) and due to the scan order, must be a wall facing the left because
# it must be an internal wall


group_string = "o {}\n"
matlib_string = "mtllib {}\n"
material_string = "usemtl {}\n"
vertex_string = "v {} {} {}\n"
texcoord_string = "vt {} {}\n"
normal_string = "vn {} {} {}\n"
face_string = "f {}/{}/{} {}/{}/{} {}/{}/{}\n"


class Generator:
    def __init__(self):
        self.horizontals = []
        self.verticals = []
        self.wall_colour = None
        self.lines = []
        self.normals = []
        self.size = (0, 0)
        self.pixels = None
        self.rects = []
        self.bounds = [[10000000, 10000000], [0, 0]]

    def clear(self):
        self.horizontals = []
        self.verticals = []
        self.wall_colour = None
        self.lines = []
        self.normals = []
        self.size = (0, 0)
        self.pixels = None
        self.rects = []
        self.bounds = [[10000000, 10000000], [0, 0]]

    def process_image(self, image_path="assets/building.png"):
        self.clear()

        image = Image.open(image_path)
        self.pixels = image.load()
        px = self.pixels
        print("Image Size:", image.size)
        self.size = image.size
        width, height = self.size

        for row in range(image.size[1]):
            if row % 10 == 0 and row != 0:
                sys.stdout.write('.')
                sys.stdout.flush()
            if row % 500 == 0 and row != 0:
                print('#')
            for col in range(image.size[0]):
                if px[col, row][0] != 255:
                    if not self.wall_colour: self.wall_colour = px[col, row]
                    verts = list(filter(lambda l: l[0] == col and l[1][1] == row - 1, self.verticals))
                    horzs = list(filter(lambda l: l[1] == row and l[0][1] == col - 1, self.horizontals))

                    if len(verts) == 0 and len(horzs) == 0:
                        self.verticals.append([col, [row, row]])  # Add a pixel
                        self.horizontals.append([[col, col], row])
                    else:
                        if len(verts) > 0:
                            verts[0][1][1] += 1
                        else:
                            if row + 1 < height and px[col, row + 1][0] != 255: self.verticals.append([col, [row, row]])

                        if len(horzs) > 0:
                            horzs[0][0][1] += 1
                        else:
                            if col + 1 < width and px[col + 1, row][0] != 255: self.horizontals.append([[col, col], row])

        # Remove all single-pixel lines
        clean_horizontals = list(filter(lambda h: h[0][0] != h[0][1], self.horizontals))
        clean_verticals = list(filter(lambda v: v[1][0] != v[1][1], self.verticals))

        self.compute_bounds()

        self.horizontals = clean_horizontals
        self.verticals = clean_verticals
        self.lines = self.verticals + self.horizontals
        self.normals = [[]]*len(self.lines)

        self.compute_normals()
        self.compute_covering()

    def find_adjoining(self, line):
        if is_horizontal(line):  # [ [x_min, x_max], y ], [ x, [y_min, y_max] ]
            return list(filter(lambda l: (line[1] == l[1][0] or line[1] == l[1][1]) and
                                         (line[0][0] == l[0] or line[0][1] == l[0]), self.verticals))
        elif is_vertical(line):  # [ x, [y_min, y_max] ], [ [x_min, x_max], y ]
            return list(filter(lambda l: (line[0] == l[0][0] or line[0] == l[0][1]) and
                                         (line[1][0] == l[1] or line[1][1] == l[1]), self.horizontals))
        else:
            print("Error, not a valid vertical or horizontal line")
            return []

    def is_interior(self, line):
        if self.verticals[0] == line: return True

        visited = [False] * len(self.lines)
        visited[0] = True
        curr_line = self.verticals[0]

        while curr_line:
            neighbours = self.find_adjoining(curr_line)
            nidx = list(map(lambda x: self.lines.index(x), neighbours))

            if neighbours[0] == line or neighbours[1] == line: return True
            elif not visited[nidx[0]]:
                visited[nidx[0]] = True
                curr_line = self.lines[nidx[0]]
            elif not visited[nidx[1]]:
                visited[nidx[1]] = True
                curr_line = self.lines[nidx[1]]
            else:
                curr_line = None

        return False

    def compute_normals(self, starting_normal=[+1, 0]):
        curr_line = self.lines[0]
        self.normals[0] = starting_normal
        curr_normal = starting_normal

        while curr_line:
            neighbours = self.find_adjoining(curr_line)
            nidx = list(map(lambda x: self.lines.index(x), neighbours))

            if not self.normals[nidx[0]]:
                self.normals[nidx[0]] = compute_normal(curr_line, curr_normal, self.lines[nidx[0]])
                curr_normal = self.normals[nidx[0]]
                curr_line = self.lines[nidx[0]]
            elif len(nidx) > 1 and not self.normals[nidx[1]]:
                self.normals[nidx[1]] = compute_normal(curr_line, curr_normal, self.lines[nidx[1]])
                curr_normal = self.normals[nidx[1]]
                curr_line = self.lines[nidx[1]]
            else:
                # Search for the next line without a normal, it must, by inference, be a vertical line to the left and
                # therefore has a normal of [-1,0], repeat until no more vertical lines with null normals remain
                idx = self.normals.index([])
                curr_line = self.lines[idx] if is_vertical(self.lines[idx]) else None
                if curr_line:
                    curr_normal = [-1, 0]
                    self.normals[idx] = [-1, 0]

    def compute_bounds(self):
        self.bounds[0][0] = self.size[0]
        self.bounds[1][0] = 0
        self.bounds[0][1] = self.size[1]
        self.bounds[1][1] = 0

        for v in self.verticals:
            SP = start_pos(v)
            EP = end_pos(v)
            if SP[0] < self.bounds[0][0]: self.bounds[0][0] = SP[0]
            if SP[1] < self.bounds[0][1]: self.bounds[0][1] = SP[1]
            if EP[0] > self.bounds[1][0]: self.bounds[1][0] = EP[0]
            if SP[1] > self.bounds[1][1]: self.bounds[1][1] = EP[1]

    def find_horizontal(self, start):
        return list(filter(lambda x: start_pos(x) == start, self.horizontals))

    def flood_fill(self, px, pos, colour):
        width = self.size[0]
        height = self.size[1]

        old_colour = px[pos[0], pos[1]]
        queue = [pos]
        while len(queue) > 0:
            curr = queue[0]
            queue.pop(0)
            w = list(curr)
            e = list(curr)
            while e[0]+1 < width and px[e[0]+1, e[1]] == old_colour:
                e[0] += 1
            while w[0]-1 >= 0 and px[w[0]-1, w[1]] == old_colour:
                w[0] -= 1

            for i in range(w[0], e[0]+1):
                px[i, curr[1]] = colour
                if [i, curr[1]] in queue: queue.remove([i, curr[1]])
                if curr[1] + 1 < height and px[i, curr[1]+1] == old_colour:
                    queue.append([i, curr[1]+1])
                if curr[1] - 1 >= 0 and px[i, curr[1]-1] == old_colour:
                    queue.append([i, curr[1]-1])

    def is_wall(self, colour):
        return colour == self.wall_colour

    def is_outside(self, colour):
        return colour == (1, 0, 0)

    def is_inside(self, colour):
        return colour == (2, 0, 0)

    def is_space(self, colour):
        return colour == (255, 255, 255)

    # Note: Calculate the floor tessellation for the interior, decompose rectilinear polygon to maximal rectangles
    #
    # Algorithm:
    # 1) Start scanning at first interior wall.  Each horizontal line is called, in order, from top to bottom.
    # 2) Create first rectangle and increment x until the first wall found.
    #    - If another wall is found in the same scanline, start a new polygon.  Keep an open polygon list.
    # 3) If a corner is found, close current rectangle and start new rectangle.

    def compute_covering(self, outside_pos=[0, 0]):
        # Use the source image and flood fill the inside and outside to make open rectangle tracking easier

        px = self.pixels

        if not self.is_space(px[outside_pos[0], outside_pos[1]]):
            print("Please set a valid outside pixel position")

        self.flood_fill(px, outside_pos, (1, 0, 0))

        startP = start_pos(self.verticals[0])       # Start inside the first wall
        startP[0] += 1
        startP[1] += 1

        self.flood_fill(px, startP, (2, 0, 0))

        # The space is partitioned into an outside, an inside, and a clear space (all interior inaccessible polys)

        open = []
        closed = []

        for row in range(self.bounds[0][1], self.bounds[1][1]+1):
            for col in range(self.bounds[0][0], self.bounds[1][0]+1):
                curr = px[col, row]

                if self.is_wall(curr):
                    for o in open:
                        left, top = top_left(o)
                        right, bottom = bottom_right(o)

                        if top == row-1 and left < col:
                            closed.append(o)
                            open.remove(o)
                            break
                        if top == row and right > col:
                            open.append([[left, top], [col-1, row]])
                            o[1][1] -= 1
                            closed.append(o)
                            open.remove(o)
                            break
                elif self.is_inside(curr):
                    found = False
                    for o in open:
                        left, top = top_left(o)
                        right, bottom = bottom_right(o)

                        if row == top and bottom == top and col-1 == right:
                            # This is the first row, increase until we hit a wall
                            o[1][0] += 1
                            found = True
                            break
                        elif row-1 == top and left == col:
                            # New line starting at beginning of rect
                            o[1][1] += 1
                            found = True
                            break
                        elif top == row and col <= right:
                            # Interior of existing rect
                            found = True
                            break

                    if not found:
                        open.append([[col, row], [col, row]])

        self.rects = closed

    def render_to_image(self, filename="assets/output.png", normal_len=5):
        img = Image.new('RGB', self.size, color='black')
        px = img.load()

        # Draw the floor first on the bottom

        for r in self.rects:
            colour = get_rand3(0, 256)
            for row in range(r[0][1], r[1][1]+1):
                for col in range(r[0][0], r[1][0]+1):
                    px[col, row] = colour

        for h in self.horizontals:
            for col in range(h[0][0], h[0][1] + 1):
                px[col, h[1]] = (255, 0, 0)

        for v in self.verticals:
            for row in range(v[1][0], v[1][1] + 1):
                px[v[0], row] = (255, 0, 0)

        # Draw the normals

        for l in range(len(self.lines)):
            if is_horizontal(self.lines[l]):
                nor = self.normals[l]
                lne = self.lines[l]
                # print(nor, lne)
                hw = (lne[0][0] + lne[0][1]) / 2
                if nor != [] and nor[1] == 1:
                    for row in range(lne[1], lne[1] + normal_len):
                        px[hw, row] = (0, 0, 255)
                elif nor != [] and nor[1] == -1:
                    for row in range(lne[1] - normal_len, lne[1]):
                        px[hw, row] = (0, 0, 255)
            elif is_vertical(self.lines[l]):
                nor = self.normals[l]
                lne = self.lines[l]
                # print(nor, lne)
                hh = (lne[1][0] + lne[1][1]) / 2
                if nor != [] and nor[0] == 1:
                    for col in range(lne[0], lne[0] + normal_len):
                        px[col, hh] = (0, 0, 255)
                elif nor != [] and nor[0] == -1:
                    for col in range(lne[0] - normal_len, lne[0]):
                        px[col, hh] = (0, 0, 255)

        self.flood_fill(px, [0, 0], (50, 50, 50))

        pos = start_pos(self.verticals[0])
        pos[0] += 1
        pos[1] += 1

        self.flood_fill(px, pos, (128, 128, 128))

        img.save(filename, "PNG")

    def find_centre(self, vertices):
        vtx_count = len(vertices)
        inv_vtx_count = 1./vtx_count

        if vtx_count == 0: return []

        centre = [0., 0., 0]
        for v in vertices:
            centre[0] += v[0][0]
            centre[1] += v[0][1]

        centre[0] *= inv_vtx_count
        centre[1] *= inv_vtx_count

        return centre

    def centre_vertices(self, vertices, centre):
        return list(map(lambda x: [[x[0][0] - centre[0], x[0][1] - centre[1], x[0][2]], x[1], x[2]], vertices))

    def centre_model(self, vertices):
        """This method will centre the model in the XY-plane and place the floor at z == 0"""

        vtx_count = len(vertices)
        inv_vtx_count = 1./vtx_count

        if vtx_count == 0: return []

        centre = [0., 0., 0]
        for v in vertices:
            centre[0] += v[0][0]
            centre[1] += v[0][1]

        centre[0] *= inv_vtx_count
        centre[1] *= inv_vtx_count

        return list(map(lambda x: [[x[0][0] - centre[0], x[0][1] - centre[1], x[0][2]], x[1], x[2]], vertices))

    def write_mtl_file(self, type="", filename="assets/output.mtl"):
        """type must be 'walls' or 'floors'"""
        mat_def = "newmtl {}\n"
        amb_def = "Ka {} {} {}\n"
        diff_def = "Kd {} {} {}\n"
        spec_def = "Ks {} {} {}\n"
        exp_def = "Ns {}\n"
        amb_map = "map_Ka {}\n"
        diff_map = "map_Kd {}\n"

        file = open(filename, "w")

        # Walls Material
        if type == "walls":
            file.write(mat_def.format("walls_material"))
            file.write(amb_def.format(.5, .5, .5))
            file.write(diff_def.format(.9, .9, .9))
            file.write(spec_def.format(.3, .3, .3))
            file.write(exp_def.format(50))
            file.write(amb_map.format("wall.jpg"))
            file.write(diff_map.format("wall.jpg"))

        # Floors Material
        if type == "floors":
            file.write(mat_def.format("floors_material"))
            file.write(amb_def.format(.5, .5, .5))
            file.write(diff_def.format(.9, .9, .9))
            file.write(spec_def.format(.3, .3, .3))
            file.write(exp_def.format(50))
            file.write(amb_map.format("floor.jpg"))
            file.write(diff_map.format("floor.jpg"))

        file.close()

    def strip_name(self, filename):
        """Return the filename stripped of its suffix"""
        offset = filename.rfind('.')
        if offset != -1:
            return filename[:offset]
        else:
            return filename

    def strip_filename(self, path):
        offset = path.rfind('/')
        if offset != -1:
            return path[offset+1:]
        else:
            return path

    def export_to_object(self, filename="assets/output.obj"):
        """Export the plan file to an object file.  Call this only after the file has been processed."""
        # Normalise to the image size taking the longer axis as the dimension for the model
        dim = max(self.size[0], self.size[1])

        # Note: The SDF provides limited ability to scale the model.  We do this here
        inv_dim = 1./dim

        # Note: The walls and floors require a repeat texture and are defined in world space with the repeat texture
        # assumed to be of unit length and wrapable

        # Flip y due to difference between image space and world space
        scale = make_scale([inv_dim, -inv_dim, inv_dim])

        # Turn a generic plane into a plane centered at the midpoint, with the normal in the correct position
        rot_axis = [0, 0, 1]

        wall_vertices = []
        wall_indices = []

        line_height = 10.

        # Add the walls
        for i in range(len(self.lines)):
            normal = self.normals[i]
            if not normal: continue
            line = self.lines[i]
            rot = make_rotation(rot_axis, get_angle(normal))
            ln = line_len(line)
            hlen = ln/2.

            positions = [
                [+hlen, 0., 0., 1.],
                [-hlen, 0., 0., 1.],
                [-hlen, 0., line_height, 1.],
                [+hlen, 0., line_height, 1.]]

            texcoords = [
                [ln/10., 0.],
                [0.,     0.],
                [0.,     line_height/10],
                [ln/10., line_height/10]
            ]

            centre = centre_pos(line) + [0.]
            trans = make_translation(centre)

            model_mat = np.dot(scale, np.dot(trans, rot))
            vidx = len(wall_vertices)
            for i in range(4):
                wall_vertices.append([np.dot(model_mat, positions[i]), texcoords[i], normal + [0.]])

            wall_indices.append(vidx)
            wall_indices.append(vidx+1)
            wall_indices.append(vidx+2)

            wall_indices.append(vidx)
            wall_indices.append(vidx+2)
            wall_indices.append(vidx+3)

        floor_vertices = []
        floor_indices = []

        # Add the floor planes
        for rect in self.rects:
            width = (rect[1][0] - rect[0][0])
            height = (rect[1][1] - rect[0][1])

            left, bottom = bottom_left(rect)

            positions = [
                [left - 1, bottom + height + 1, 0., 1.],
                [left + width + 1, bottom + height + 1, 0., 1.],
                [left + width + 1, bottom - 1, 0., 1.],
                [left - 1, bottom - 1, 0., 1.]
            ]

            texcoords = [
                [0., 0.],
                [width/10, 0.],
                [width/10, height/10],
                [0.,       height/10]
            ]

            model_mat = scale

            vidx = len(floor_vertices)
            for i in range(4):
                floor_vertices.append([np.dot(model_mat, positions[i]), texcoords[i], [0., 0., 1.]])

            floor_indices.append(vidx)
            floor_indices.append(vidx+1)
            floor_indices.append(vidx+2)

            floor_indices.append(vidx)
            floor_indices.append(vidx+2)
            floor_indices.append(vidx+3)

        # Centre the model in XY-plane (We need to find it for all walls and floors)
        centre = self.find_centre(wall_vertices + floor_vertices)

        wall_vertices = self.centre_vertices(wall_vertices, centre)
        floor_vertices = self.centre_vertices(floor_vertices, centre)

        walls_filename = self.strip_name(filename) + "_walls.obj"
        matpath = self.strip_name(filename) + "_walls.mtl"
        matfile = self.strip_filename(matpath)

        file = open(walls_filename, "w")

        for v in wall_vertices:
            file.write(vertex_string.format(v[0][0], v[0][1], v[0][2]))

        file.write("\n")

        for v in wall_vertices:
            file.write(texcoord_string.format(v[1][0], v[1][1]))

        file.write("\n")

        for v in wall_vertices:
            file.write(normal_string.format(v[2][0], v[2][1], v[2][2]))

        file.write("\n")

        # Write Walls Group
        for i in range(int(len(wall_indices)/3)):
            file.write(group_string.format("wall_" + str(i)))
            file.write(matlib_string.format(matfile))
            file.write(material_string.format("walls_material"))

            idx = 3*i
            # Note: Add one to each index because OBJ indexing starts at 1 not 0
            file.write(face_string.format(wall_indices[idx]+1, wall_indices[idx]+1, wall_indices[idx]+1,
                                          wall_indices[idx+1]+1, wall_indices[idx+1]+1, wall_indices[idx+1]+1,
                                          wall_indices[idx+2]+1, wall_indices[idx+2]+1, wall_indices[idx+2]+1))

        file.close()

        # Add the matching material file
        self.write_mtl_file(type="walls", filename=matpath)

        floors_filename = self.strip_name(filename) + "_floors.obj"
        matpath = self.strip_name(filename) + "_floors.mtl"
        matfile = self.strip_filename(matpath)

        file = open(floors_filename, "w")

        for v in floor_vertices:
            file.write(vertex_string.format(v[0][0], v[0][1], v[0][2]))

        file.write("\n")

        for v in floor_vertices:
            file.write(texcoord_string.format(v[1][0], v[1][1]))

        file.write("\n")

        for v in floor_vertices:
            file.write(normal_string.format(v[2][0], v[2][1], v[2][2]))

        file.write("\n")

        # Write Floor Group
        for i in range(int(len(floor_indices)/3)):
            file.write(group_string.format("floor_" + str(i)))
            file.write(matlib_string.format(matfile))
            file.write(material_string.format("floors_material"))

            idx = 3*i
            file.write(face_string.format(floor_indices[idx]+1, floor_indices[idx]+1, floor_indices[idx]+1,
                                          floor_indices[idx+1]+1, floor_indices[idx+1]+1, floor_indices[idx+1]+1,
                                          floor_indices[idx+2]+1, floor_indices[idx+2]+1, floor_indices[idx+2]+1))

        file.close()

        # Add the matching material file
        self.write_mtl_file(type="floors", filename=matpath)

    def export_to_sdf(self, offset, scale, filename="assets/output.sdf"):
        sdf = SDFGenerator.SDFGenerator(filename, scale)
        sdf.add_walls(offset, self.strip_name(filename) + "_walls.obj")
        sdf.add_floors(offset, self.strip_name(filename) + "_floors.obj")
        sdf.add_extra()         # This is possibly only required for gazebo
        sdf.write_file()
