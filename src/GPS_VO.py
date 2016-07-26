"""

MIT License (MIT)

Copyright (c) SUMMER 2016, Carnegie Mellon University

Author: Jahdiel Alvarez

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

"""

import exifread
import re
from utm import from_latlon
import haversine
from collections import OrderedDict


def parse_GPS_Lat_Lon(gps_lat_lon):
    """Returns the GPS_Latitude and GPS_Longitude variables as a list,
     this is because ExifRead returns them as type 'instance'"""
    # Convert to a string
    gps_lat_lon = str(gps_lat_lon)
    # Delete the first and last square bracket characters by slicing
    gps_lat_lon = gps_lat_lon[1:len(gps_lat_lon)-1]
    # Eliminate all whitespaces and split using the commas
    gps_lat_lon = gps_lat_lon.replace(' ','').split(',')

    if re.search("[/]", gps_lat_lon[2]):
        gps_lat_lon[2] = map(float, gps_lat_lon[2].split('/'))
        gps_lat_lon[2] = gps_lat_lon[2][0]/gps_lat_lon[2][1]

    gps_lat_lon = map(float, gps_lat_lon)

    return gps_lat_lon


def sexag_to_dec(sexag_unit):
    """ Converts Latitude and Longitude Coordinates from the Sexagesimal Notation
        to the Decimal/Degree Notation"""

    add_to_degree = (sexag_unit[1] + (sexag_unit[2]/60))/60
    return sexag_unit[0]+add_to_degree

def gps_filename_dict(images_list):
    """Returns the filename of the image and its GPS Coordinates
    as dictionary data structure."""

    # List which will hold the coordinate pairs and pixel values
    coordinates_dict = OrderedDict()
    # Produce the text for each image in the stack
    for file_name in images_list:
        # Open image file for reading (binary mode) and processing with PIL
        f = open(file_name, 'rb')
        # Return Exif tags
        tags = exifread.process_file(f)

        # Initializing some variables
        GPS_Longitude = 0
        GPS_Latitude = 0

        # Returns all the GPS Related Metadata
        for tag in tags.keys():

            if tag == 'GPS GPSLongitude':
                GPS_Longitude = parse_GPS_Lat_Lon(tags[tag])
                #Change from sexagesimal to decimal/degree notation
                GPS_Longitude = sexag_to_dec(GPS_Longitude)

            elif tag == 'GPS GPSLatitude':
                GPS_Latitude = parse_GPS_Lat_Lon(tags[tag])
                # Change from sexagesimal to decimal/degree notation
                GPS_Latitude = sexag_to_dec(GPS_Latitude)

            if GPS_Latitude and GPS_Longitude:
                coordinates_dict[file_name] = (GPS_Latitude, GPS_Longitude)

    return coordinates_dict


# Returns a dict of utm coord {filename: utm}
def gps_to_utm(gps_dic):
    """Converts Lat/Lon coordinates into UTM coordinates"""
    utm_dic = OrderedDict()
    for file_name in gps_dic.keys():
        lat, log = gps_dic[file_name][0], gps_dic[file_name][1]
        holder = from_latlon(lat, -log)         #changes lat, log to utm values (EASTING, NORTHING, ZONE NUMBER, ZONE LETTER)
        '''Pitt is in zone 17 so we choice not to return this now because only easting and northing is needed'''
        utm_dic[file_name] = (holder[0], holder[1])
    return utm_dic


def getGPS_distance(prevGPS, curGPS):
    """ Returns the distance between two GPS coordinates(in Lat/Lon Coord System) in meters"""
    distance = haversine.haversine(prevGPS, curGPS) * 1000  # meters

    return distance
