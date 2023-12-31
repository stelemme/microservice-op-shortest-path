# Load ifcopenshell functions to read the ifc file
import ifcopenshell
from ifcopenshell import geom

# Load OCC functions for displaying shapes
from OCC.Display.SimpleGui import init_display
from OCC.Display.OCCViewer import rgb_color

# Load own (new!) functions
from ..aWalkInTheGraph import get_shapes_to_display, get_shapes_to_display_walk_spaces,\
                              make_graph, from_sequence_to_walk


from flask import (
    Blueprint, render_template, request, make_response, jsonify
)

import datetime
import os

# Initiating the blueprint and assigning to it the "/op/" URI
bp = Blueprint('shortest-path', __name__, url_prefix='/op')

# Adding a function to the "/op/some-operation" URI
@bp.route('/shortest-path', methods=('GET', 'POST'))
def operation():
    if request.method == 'GET':
        response = make_response({
            "supported_methods": ["GET", "POST"],
            "POST_request_data": "multipart/form-data",
            "POST_response_data": "application/json",
        })

        return response

    elif request.method == 'POST':
        # The data is retrieved out of the incoming HTTP POST request.
        start = str(request.form['start'])
        finish = str(request.form['finish'])
        accuracy = float(request.form['accuracy'])
        activate_display = eval(request.form['display'])
        file = request.files['ifcFile'].read()

        timestamp = datetime.datetime.now().timestamp()
        filename = f"ifcFile-{timestamp}.ifc"

        with open(f"base-files/{filename}", "wb") as binary_file:
            binary_file.write(file)

        ifc_file = ifcopenshell.open(f"base-files/{filename}")

        os.remove(os.path.join(os.getcwd(), f"base-files/{filename}"))

        # Manage display settings
        settings = geom.settings()
        settings.set(settings.USE_PYTHON_OPENCASCADE, True)

        # Collect the required information
        ifcspaces = ifc_file.by_type("IfcSpace")

        graph, dictNameNode = make_graph(ifcspaces, settings)

        if start not in dictNameNode:
            error_message = {"error": "The start space is not found in the IFC-file."}
            return jsonify(error_message), 400
        if finish not in dictNameNode:
            error_message = {"error": "The finish space is not found in the IFC-file."}
            return jsonify(error_message), 400

        # doors as shapes
        ifcdoors_toDisplay = []
        for edge in graph.GetEdges():
            product = edge.dynamicProperties["ifcdoor"]
            ifcdoors_toDisplay.append(product)
        door_shapes = get_shapes_to_display(ifcdoors_toDisplay, settings)

        edgeLabelsToConsider = graph.EdgeLabelsCollection

        # Shortest path in meters, so first find all sequences of spaces that connect the start and finish
        sequencelist = graph.AllPaths(dictNameNode[start], dictNameNode[finish], edgeLabelsToConsider)

        # from c# list to python list
        sequencelist_py = []
        for i in range(sequencelist.Count):
            sequence_i = []
            for value in sequencelist[i]:
                sequence_i.append(value)
            sequencelist_py.append(sequence_i)

        summary = "\n"

        response = {
            "start_space": start,
            "finish_space": finish,
            "step_length": accuracy,
            "paths": [],
        }

        if len(sequencelist_py) == 0:
            error_message = {"error": "Their is no path found between the start and finish space."}
            return jsonify(error_message), 400

        for n, sequence in enumerate(sequencelist_py):
            # the walk to be displayed as points
            walk_points, total_steps, vogelvlucht = from_sequence_to_walk(graph, sequence, accuracy)

            if activate_display:
                # Manage display settings
                display, start_display, add_menu, add_function_to_menu = init_display()

                # the spaces to be displayed as shapes
                space_shapes_red, space_shapes_blue = get_shapes_to_display_walk_spaces(sequence, settings)
                for shape in space_shapes_blue: display.DisplayShape(shape[1], transparency=0.7, color=rgb_color(0, 0, 1))
                for shape in space_shapes_red: display.DisplayShape(shape[1], transparency=0.7, color=rgb_color(1, 0, 0))
                
                for p in walk_points: display.DisplayShape(p)

                # Display all spaces and doors
                for node in graph.GetNodes(): display.DisplayShape(node.dynamicProperties["shape"], transparency=0.7)
                for shape in door_shapes: display.DisplayShape(shape[1], transparency=0.2)

                start_display()

                # Destroy display to be able to get a new display
                del display
                del start_display
            
            path = {
                "name": f'path_{n}',
                "total_steps": total_steps,
                "total_length": accuracy * total_steps,
                "spaces_passed": len(sequence),
            }

            response["paths"].append(path)

            # The result
            summary += "The walk was " + str(total_steps) + " steps long and one step has a length of " + str(accuracy) + ".\n"
            summary += "Total: " + str(accuracy * total_steps) + "\n"
            summary += "We passed through " + str(len(sequence)) + " spaces (incl. start and finish).\n" + "\n"

        shortest_path = min(response["paths"], key=lambda x: x["total_length"])
        least_spaces_passed = min(response["paths"], key=lambda x: x["spaces_passed"])

        response["shortest_path"] = shortest_path["name"]
        response["least_spaces_passed"] = least_spaces_passed["name"]

        summary += "Total bird's eye: " + str(vogelvlucht) + "\n"
        print()
        print()
        print()
        print("SUMMARY TEXT")
        print(summary)

        print("The end... for real now.")
        print()

        return response
    
    


