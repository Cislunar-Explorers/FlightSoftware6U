import json


def get_angular_seperations(time):
    json_object = json.load(open("observations.json"))
    mylst = []
    item = json_object["observations"]
    for times in item:
        if times["time"] == time:
            target = times["observed_bodies"]
            for obj in target:
                mylst.append(obj["direction_body"])
            break
    return mylst


get_angular_seperations(10800.0)
