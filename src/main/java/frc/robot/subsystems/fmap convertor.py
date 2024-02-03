import math
import json
def convert_to_meters(x: float):
    return (x/39.37)

def convert_array_to_fmap(coords: tuple):
    x = round(convert_to_meters(coords[0])-8.27,2)
    y = round(convert_to_meters(coords[1])-4.105,2)
    z = round(convert_to_meters(coords[2]),2)
    a = math.radians(coords[3])
    return [round(math.cos(a),2),round(-math.sin(a),2),0,x,
            round(math.sin(a),2),round(math.cos(a),2),0,y,
            0,0,1,z,
            0,0,0,1]


all_coords = [
    (593.68,9.68,53.38,120),
    (637.21,34.79,53.38,120),
    (652.73,196.17,57.13,180),
    (652.73,218.42,57.13,180),
    (578.77,323.00,53.38,270),
    (72.5,323.00,53.38,270),
    (-1.50,218.42,57.13,0),
    (-1.50,196.17,57.13,0),
    (14.02,34.79,53.38,60),
    (57.54,9.68,53.38,60),
    (468.69,146.19,52.00,300),
    (468.69,177.10,52.00,60),
    (441.74,161.62,52,180),
    (209.48,161.62,52,0),
    (182.73,177.10,52,120),
    (182.73,146.19,52,240)
]

fmap = {"fiducials":list(all_coords)}
for i,item in enumerate(all_coords):
    fmap["fiducials"][i]={"unique":1,
                          "family":"apriltag3_16h5_classic",
                          "size":152.4,
                          "id":str(i+1),
                          "transform":convert_array_to_fmap(item)}

print(fmap)
with open("src/main/java/frc/robot/subsystems/AprilTags.fmap", "w") as jsonf:
    json.dump(fmap,jsonf)
    jsonf.close()
    