
import rospy
from lib.servo_dict import servo_dict
from src.robot_base.scripts.base.base_node import BaseNode


def get_factor(name):
    factor = 0
    if name.endswith("a"):
        if name.split("_")[2].split("a")[0] == 'l':
            factor = 1
        elif name.split("_")[2].split("a")[0] == 'r':
            factor = -1
    if name.endswith("b"):
        if name.split("_")[2].split("b")[0] == 'l':
            factor = 1
        elif name.split("_")[2].split("b")[0] == 'r':
            factor = -1
    if name.endswith("c"):
        if name.split("_")[2].split("c")[0] == 'l':
            factor = -1
        elif name.split("_")[2].split("c")[0] == 'r':
            factor = 1

    return factor


hoek = "min"+ factor*85
speed_up = 139
speed_down = 35

parcour_dict = {
    "1": {
        "time": 0,
        "id_array": [4,5,6,7,8,9],
        "po_array": ["min", "min","min","min","min","min"],
        "sp_array": [50,50,50,50,50,50]
    },
    "2": {
        "time": 3,
        "id_array": [4,5,6,7,8,9],
        "po_array": [hoek,hoek,"min","min",hoek,hoek],
        "sp_array": [50,50,50,50,50,50]
    },
    "3": {
        "time": 3,
        "id_array": [8,9],
        "po_array": ["min","min"],
        "sp_array": [speed_down,speed_down]
    }
    "4": {
        "time": 4,
        "id_array": [6,7],
        "po_array": [hoek,hoek],
        "sp_array": [speed_up,speed_up]
    }
}