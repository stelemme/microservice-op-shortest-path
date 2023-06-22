# Shortest Path Operation Microservice
The Shortest Path calculation libraries are provided by Deprez (2023).

https://www.food4rhino.com/en/app/plan-graph
https://users.ugent.be/~llidprez/html/deef45b8-01e1-7aa4-2406-012e4deb15ba.htm
## Installation
To install this Microservice, make sure Anaconda or Miniconda is installed on your device. Installing the Microservice can be done by first starting a Conda virtual environment with the help of the environment.yml file. The virtual environment is activated using the following commands.
```
conda env create -f environment.yml
conda activate shortest_path
```
This could take a while, all the dependencies are now correctly installed.

**IMPORTANT**: Add to the root of the Microservice a new folders: "base-files". In this folder, the IFC-files will be temporarily saved during the calculations.
## Running the Microservice
To run the Microservice, the Conda virtual environment needs to be activated using the following command.
```
conda activate shortest_path
```
After the virtual environment is activated the Microservice can be run using the following command. Additional arguments can be added to the command to for example run the Microservice in debug mode (--debug), which enables live reloading. The port on which the Microservice runs can also be changed with an additional argument (--port <port-number>).
```
flask --app flaskr run
```
## Using the Microservice
The functionality of this Microservice can be accessed via the following endpoint.
  
[http://localhost:3000/op/shortest-path](http://localhost:3000/op/shortest-path)
  
This endpoint has a GET and POST method. The GET response of the endpoint returns a JSON object that specifies which methods and data types are supported by the endpoint. In this case, an IFC-file must be sent in a POST request to the endpoint. The endpoint then returns a JSON file containing all the information about the calculated shortest path. The start and end space also need to be give in the post request to make a correct calculation of the shortest path as well as some additional parameters. The endpoint will thus need to receive a form-data request containing the IFC-file and the additional parameters.

Mandatory Paramters:
- ifcFile: The IFC-file.
- start: The start space of the path that needs to be calculated.
- finish: The finish space of the path that needs to be calculated.
- accuracy: The length of steps in meters.
- display: A boolean indicating whether or not a graphical display needs to be returned.
### Example request
```
url = <the URL>
payload = {
  'start': 'lobby',
  'finish': 'badkamer 012',
  'accuracy': '0.8',
  'display': 'False'
}
files=[
  ('ifcFile',<the IFC-file>)
]

response = requests.request("POST", url, data=payload, files=files)
```
### Example response
```
{
    "start_space": "badkamer 012",
    "finish_space": "lobby",
    "step_length": 0.8
    "paths": [
        {
            "name": "path_0",
            "spaces_passed": 11,
            "total_length": 109.60000000000001,
            "total_steps": 137
        },
        {
            "name": "path_1",
            "spaces_passed": 14,
            "total_length": 152.8,
            "total_steps": 191
        }
    ],
    "shortest_path": "path_0",
    "least_spaces_passed": "path_0",
}
```
