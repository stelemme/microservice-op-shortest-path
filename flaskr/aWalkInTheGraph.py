import ifcopenshell

import shapely.geometry as slg
# Load OCC functions
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_VERTEX
from OCC.Core.BRep import BRep_Tool
from OCC.Core.gp import gp_Pnt2d
# Load PlanGraph functions
# define path to dll
assembly_path1 = "flaskr/dll"
import sys
sys.path.append(assembly_path1)
import clr
clr.AddReference("PlanGraphLibPy")
from PlanGraphLibPy.The_graph import Graph_Plan, Graph, Node_Space, Node, Edge_Space, Edge
from PlanGraphLibPy.Geometry import Point, Polyline
from PlanGraphLibPy import GeneralFunctions
clr.AddReference('System.Collections')
from System.Collections.Generic import List

class UGSpace:
    def __init__(self, id):
        self.id = id
        self.name = ""
        self.boundaries = []
        self.neighbours = {}
        self.doors = []

    def __init__(self, id, boundaries):
        self.id = id
        self.name = ""
        self.boundaries = boundaries
        self.neighbours = {}
        self.doors = []

    def __init__(self, id, name, boundaries):
        self.id = id
        self.name = name
        self.boundaries = boundaries
        self.neighbours = {}
        self.doors = []

    def SearchDoors(self):
#        print(str(len(self.boundaries))+" boundaries found")
        for b in self.boundaries:
            if b.RelatedBuildingElement != None:
                if 'IfcDoor' == b.RelatedBuildingElement.is_a():
                    self.doors.append(b.RelatedBuildingElement)
        print("    -  "+self.name +" has " + str(len(self.doors))+(" door" if len(self.doors) == 1 else " doors"))

    def SearchNeighbours(self, spaces):
        for s in spaces:
            if s.id == self.id: continue
            for d in s.doors:
                for sd in self.doors:
                    if d.GlobalId == sd.GlobalId and s not in self.neighbours:
                        self.neighbours[s] = sd
        print("    -  "+self.name +" has " + str(len(self.neighbours))+" neighbours")

def find_shape_boundary(shape):
    """
    From the shape, the boundary is extracted as a Polyline (Plan Graph) and added to the node as a dynamic property.

    :param shape: The ifcopenshell shape extracted from the ifc
    :return: The Polyline (Plan Graph)
    """
    faceExplorer = TopExp_Explorer(shape, TopAbs_FACE)
    lowestZ = float("inf")
    lowestSurf = None

    while(faceExplorer.More()):
        currentFace = faceExplorer.Current()
        # do st with face
        #face = BRep_Tool.Surface(currentFace)
        vertexExplorer = TopExp_Explorer(currentFace, TopAbs_VERTEX)
        face_points = List[Point]()
        minZ = float("inf")
        while (vertexExplorer.More()):
            currentVertex = vertexExplorer.Current()
            # do st with vertex
            shp_g = BRep_Tool.Pnt(currentVertex)  # get the geometry of the Vertex ->point
            pt = Point(shp_g.Coord()[0], shp_g.Coord()[1], shp_g.Coord()[2])
            if pt.Z < minZ:
                minZ = pt.Z
            #print(shp_g.Coord()[0], shp_g.Coord()[1], shp_g.Coord()[2])
            face_points.Add(pt)
            vertexExplorer.Next()
        polyline = Polyline(face_points)
        if polyline.IsHorizontal() and minZ <= lowestZ:
            lowestSurf = polyline
        if minZ <= lowestZ:
            lowestZ = minZ
        #print("NEXT!!!")
        faceExplorer.Next()
    return lowestSurf


def generate_graph_in_pl(pl, spacing=1):
    """
    A grid as a graph (Plan Graph) is generated inside a Plan Graph Polyline.
    This function will mainly be used to project a grid in a space to find the shortest path.

    :param pl: A Polyline (Plan Graph)
    :param spacing: The distance between the neighbouring grid nodes.
    :return: The grid as a graph.
    """
    # find the bounding box
    plpts = []
    for plpt in pl.Points:
        plpts.append((plpt.X, plpt.Y))
    polygon = slg.Polygon(plpts)
    minx, miny, maxx, maxy = polygon.bounds

    # generate the points
    gridnodes_prevLevel = []
    gridnodes_thisLevel = []
    gridnodes_all = List[Node_Space]()
    gridedges_all = List[Edge_Space]()
    gridpoints = []
    x = minx + spacing/2
    y = miny + spacing/2
    x_id = 0
    y_id = 0
    while x < maxx:
        while y < maxy:
            gridpoint = Point(x, y, 0)
            if ptInPl(pl, gridpoint):
                # make gridnode
                gridpoints.append(gridpoint)
                gridnode = Node_Space("gridnode", 0, gridpoint)
                #print(gridnode.Center.X, gridnode.Center.Y, gridnode.Center.Z)
                gridnodes_thisLevel.append(gridnode)
                gridnodes_all.Add(gridnode)
                # make gridedges
                if y_id > 0:
                    if gridnodes_thisLevel[y_id-1] != None:
                        edge_prev = Edge_Space("gridedge")
                        edge_prev.SetNodeStart(gridnode)
                        edge_prev.SetNodeEnd(gridnodes_thisLevel[y_id-1])
                        gridedges_all.Add(edge_prev)
                if x_id > 0:
                    if gridnodes_prevLevel[y_id] != None:
                        edge_prevlevel = Edge_Space("gridedge")
                        edge_prevlevel.SetNodeStart(gridnode)
                        edge_prevlevel.SetNodeEnd(gridnodes_prevLevel[y_id])
                        gridedges_all.Add(edge_prevlevel)
            else:
                gridnodes_thisLevel.append(None)
            y += spacing
            y_id += 1
        gridnodes_prevLevel = gridnodes_thisLevel
        gridnodes_thisLevel = []
        y = miny + spacing / 2
        y_id = 0
        x += spacing
        x_id += 1

    # make graph
    graph = Graph_Plan()
    graph.AddNodes(gridnodes_all)
    graph.AddEdges(gridedges_all)
    return graph


def ptInPl(pl, pt):
    """
    Discover if a point lies in the boundaries of a polyline.

    :param pl: A Polyline (Plan Graph)
    :param pt: A Point (Plan Graph)
    :return: Boolean value, true if point is inside the polyline.
    """
    point = slg.Point(pt.X, pt.Y)
    plpts = []
    for plpt in pl.Points:
        plpts.append((plpt.X, plpt.Y))
    polygon = slg.Polygon(plpts)
    return polygon.contains(point)


def walk_the_spaces(start, finish, graph, edgeLabelsToConsider):
    """
    :param start: The start Node (Plan Graph)
    :param finish: The finish as Node (Plan Graph)
    :param graph: The Graph (Plan Graph) that contains the two nodes.
    :param edgeLabelsToConsider: A List<string> of the edge labels that may be used to travel through the graph.
    :return: 
    """
    # dictNameNode er nog uit halen en met name, niet label
    steps = graph.ShortestPathLength(finish, start, edgeLabelsToConsider)
    """
    print("The "+start.NodeLabel+" and "+finish.NodeLabel+" are "+str(steps)+" steps away from each other.")
    print("The path will be displayed in a moment...")
    print()
    print()
    """

    parentNodesWalk = GeneralFunctions.dijkstraShortestPathWalk(graph, finish, edgeLabelsToConsider)
    done = False
    safety = 0
    sequence = [start]
    currentRoom = start
    while not done and safety < 1000:
        safety += 1
        currentRoom = parentNodesWalk[graph.GetNodes().index(currentRoom)]
        sequence.append(currentRoom)
        if currentRoom == finish:
            done = True
    return sequence, steps


def make_graph(ifcspaces, display_settings):
    # Make UG spaces and nodes
    UGspaces = []
    nodes = List[Node]()
    dictSpaceNode = {}
    dictNameNode = {}
    for ifcspace in ifcspaces:
        space = UGSpace(ifcspace.GlobalId, ifcspace.LongName, ifcspace.BoundedBy)
        UGspaces.append(space)
        node = Node(space.name)
        node.dynamicProperties["ifcSpace"] = ifcspace
        # Get the geometric shape (for displaying and for shortest path)
        if ifcspace.Representation is not None:
            shape = ifcopenshell.geom.create_shape(display_settings, ifcspace).geometry
            node.dynamicProperties["shape"] = shape
        # Add the node
        nodes.Add(node)
        dictSpaceNode[space] = node
        dictNameNode[space.name] = node

    print("We found " + str(len(UGspaces)) + " spaces:")
    for ugs in UGspaces:
        ugs.SearchDoors()

    # Find neighbours and make edges
    print("Now the neighbours through these doors are calculated:")
    registeredDoors = []
    edges = List[Edge]()
    dictDoorEdge = {}
    for ugs in UGspaces:
        ugs.SearchNeighbours(UGspaces)
        for n, d in ugs.neighbours.items():
            if d not in registeredDoors:
                registeredDoors.append(d)
                # make edge
                edge = Edge(d.ObjectType)
                edge.SetNodeStart(dictSpaceNode[ugs])
                edge.SetNodeEnd(dictSpaceNode[n])
                edge.dynamicProperties["ifcDoor"] = d
                # Get the geometric shape (for displaying and for shortest path)
                if d.Representation is not None:
                    shape = ifcopenshell.geom.create_shape(display_settings, d).geometry
                    edge.dynamicProperties["shape"] = shape
                # Add the edge
                print("        --> We added an edge between " + ugs.name + " and " + n.name + ".")
                dictDoorEdge[d] = edge
                edges.Add(edge)

    # Make the graph
    graph = Graph()
    graph.AddNodes(nodes)
    graph.AddEdges(edges)

    return graph, dictNameNode


# --------------------------------------------------------------------
# ----------------------------Shortest Path---------------------------
# ------------------------------functions-----------------------------
# --------------------------------------------------------------------

def find_closest_node(graph, point):
    min_dist = float("inf")
    min_node = None
    for node in graph.GetNodes():
        dist = point.DistanceTo(node.Center)
        if dist < min_dist:
            min_node = node
            min_dist = dist
    return min_node

def from_sequence_to_walk(graph, sequence, accuracy):
    # find the grid in the room
    for node in sequence:
        if not node.dynamicProperties.ContainsProperty("interngraph"):
            bound = find_shape_boundary(node.dynamicProperties["shape"])
            graphInPl = generate_graph_in_pl(bound, accuracy)
            node.dynamicProperties["interngraph"] = graphInPl
            print("The intern grid in the space with name '" + node.NodeLabel + "' was made.")

    # Find grid nodes in the spaces that link with doors
    points_to_display = []
    points_to_display_doors = []
    for i in range(len(sequence)-1):
        for edge in graph.GetEdges():
            if (edge.GetNodeStart() == sequence[i] or edge.GetNodeEnd() == sequence[i]) \
            and (edge.GetNodeStart() == sequence[i+1] or edge.GetNodeEnd() == sequence[i+1]):
                door_shape = edge.dynamicProperties["shape"]
                door_bound = find_shape_boundary(edge.dynamicProperties["shape"])
                door_location = Point(door_bound.Centroid().X, door_bound.Centroid().Y, door_bound.Centroid().Z)
                points_to_display_doors.append(door_location)
                node_enter = find_closest_node(sequence[i].dynamicProperties["interngraph"], door_location)
                sequence[i].dynamicProperties["interngraph"].dynamicProperties["end"] = node_enter
                points_to_display.append(node_enter.Center)
                node_arrive = find_closest_node(sequence[i+1].dynamicProperties["interngraph"], door_location)
                sequence[i+1].dynamicProperties["interngraph"].dynamicProperties["start"] = node_arrive
                points_to_display.append(node_arrive.Center)

    # Find center grid nodes of the first and the last space in the sequence
    bound_start = find_shape_boundary(sequence[0].dynamicProperties["shape"])
    center_start = Point(bound_start.Centroid().X, bound_start.Centroid().Y, bound_start.Centroid().Z)
    node_start = find_closest_node(sequence[0].dynamicProperties["interngraph"], center_start)
    sequence[0].dynamicProperties["interngraph"].dynamicProperties["start"] = node_start
    points_to_display.append(node_start.Center)

    bound_finish = find_shape_boundary(sequence[-1].dynamicProperties["shape"])
    center_finish = Point(bound_finish.Centroid().X, bound_finish.Centroid().Y, bound_finish.Centroid().Z)
    node_finish = find_closest_node(sequence[-1].dynamicProperties["interngraph"], center_finish)
    sequence[-1].dynamicProperties["interngraph"].dynamicProperties["end"] = node_finish
    points_to_display.append(node_finish.Center)

    # Walk the spaces and find the shortest path
    total_steps = 0
    walk_to_display = []
    for node in sequence:
        sequence_grid, steps = walk_the_spaces(node.dynamicProperties["interngraph"].dynamicProperties["start"],\
                                        node.dynamicProperties["interngraph"].dynamicProperties["end"],\
                                        node.dynamicProperties["interngraph"],\
                                        node.dynamicProperties["interngraph"].EdgeLabelsCollection)
        total_steps += steps
        for grid_node in sequence_grid:
            walk_to_display.append(grid_node.Center)

    # The result
    print("The walk was " + str(total_steps) + " steps long and one step has a length of " + str(accuracy) + ".")
    print("Total: " + str(accuracy * total_steps))

    vogelvlucht = center_start.DistanceTo(center_finish)
    print("Total bird's eye: " + str(vogelvlucht))

    # Display the walk
    displayed_points = []
    for point in points_to_display:
        displayed_points.append(gp_Pnt2d(point.X, point.Y))
    for point in points_to_display_doors:
        displayed_points.append(gp_Pnt2d(point.X, point.Y))
    for point in walk_to_display:
        displayed_points.append(gp_Pnt2d(point.X, point.Y))

    return displayed_points, total_steps, vogelvlucht


# --------------------------------------------------------------------
# -------------------------------DISPLAY------------------------------
# ------------------------------functions-----------------------------
# --------------------------------------------------------------------

def get_shapes_to_display(ifcObjects, settings):
    # ifcObjects are spaces and doors
    shapes = []
    for product in ifcObjects:
        if product.Representation is not None:
            shape = ifcopenshell.geom.create_shape(settings, product).geometry
            shapes.append((product, shape))
    return shapes


def get_shapes_to_display_walk_spaces(sequence, settings):
    # start and finish space as shape
    space_shapes_red = [(sequence[0], sequence[0].dynamicProperties["shape"]),
                        (sequence[-1], sequence[-1].dynamicProperties["shape"])]
    # spaces in sequence between start en finish as shape
    ifcspaces_blue = []
    for node in sequence[1:-1]:
        product = node.dynamicProperties["ifcspace"]
        ifcspaces_blue.append(product)
    space_shapes_blue = get_shapes_to_display(ifcspaces_blue, settings)

    return space_shapes_red, space_shapes_blue

