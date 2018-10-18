import math
import random
import simplekml


def get_random_coordinate():
    brc_center_lat = -119.20610
    brc_center_lon = 40.78684
    r_earth = 6371393
    r = random.uniform(0.0, 2200.0)
    offset = math.pi / 9.3
    angle = random.uniform(offset, 2*math.pi- (2./3.)*math.pi + offset )
    dx = 1.0*r * math.cos(angle)
    dy = 0.4*r * math.sin(angle)
    new_latitude  = brc_center_lat  + (dx / r_earth) * (180 / math.pi)
    new_longitude = brc_center_lon + (dy / r_earth) * (180 / math.pi) / math.cos(brc_center_lat * math.pi/180)
    return new_latitude, new_longitude


kml = simplekml.Kml()
for i in range(0,1200):
    kml.newpoint(name="%d" % i, coords=[get_random_coordinate()])
    kml.save("random_coordinates.kml")
