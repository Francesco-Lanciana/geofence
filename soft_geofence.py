# SoftGeofence Module
"""
This module is responsible for taking a kmz/kml file that contains a geofence, and outputting a proportional geofence
that has been shrunken.

The code snippet below shows how to use :py:func:`get_geofence` and :py:func:`mod_geofence` to obtain an array of 
instantiated LocationGlobal objects representing the gps coordinates making up your new geofence.

.. code:: python

    from softgeofence import get_geofence, modify_geofence

    # Parse a kml/kmz file using a the name of the file (can be .kml or .kmz)
    geofence = get_geofence('sample_mission.kmz')

    # modify the :py:class:`Geofence` object either shrinking it (shrink = True) or enlarging it (enlarge = True) by some distace
	shrunken_geofence = modify_geofence(geofence, 100, create_kml=True, shrink = True)

:py:class:`Geofence` stores an array of :py:class:`LocationGlobal` objects, which store the geofence coordinates as lat, lon, and alt.

This python file can also be called from the command line. Use:
	
	python softgeofence.py -h

For information on how to supply command line arguments.

This geofence does not take into account the altitude of the drone, and is therefore not strictly a 3D geofence.
----
"""

import zipfile
import os
import pip
import subprocess
import glob
import re
import sys
import argparse
from xml.etree.ElementTree import Element
from math import sin, cos, asin, atan2, radians, degrees, pi, sqrt
from datetime import datetime
from lxml import etree
from pykml.factory import KML_ElementMaker as KML
from pykml.factory import GX_ElementMaker as GX


class LocationGlobal(object):
	"""
	A global location object.

	The latitude and longitude are relative to the `WGS84 coordinate system <http://en.wikipedia.org/wiki/World_Geodetic_System>`_.
	The altitude is relative to mean sea-level (MSL).

	For example, a global location object with altitude 30 metres above sea level might be defined as:

	.. code:: python

		LocationGlobal(-34.364114, 149.166022, 30)

	:param lat: Latitude in degrees
	:param lon: Longitude in degrees
	:param lat_rad: Latitude in radians
	:param lon_rad: Longitude in radians
	:param alt: Altitude in meters relative to mean sea-level (MSL).
	"""

	def __init__(self, lat, lon, alt=None, coords_as_radians = False):
		self.alt = alt
		if coords_as_radians:
			self.lat = degrees(lat)
			self.lon = degrees(lon)
			self.lat_rad = lat
			self.lon_rad = lon
		else:
			self.lat = lat
			self.lon = lon
			self.lat_rad = radians(lat)
			self.lon_rad = radians(lon)


		# This is for backward compatibility.
		self.local_frame = None
		self.global_frame = None


	def get_target_gps_coord(self, bearing, distance_m):
		"""
		Given original location (calling instance), and a bearing and distance (in meters) to a target calculate new gps coords

	    :param bearing: compass bearing from original location to target
	    :param distance_m: distance in meters from original location to target
		"""

		R = 6378137.0 #Radius of the Earth

		bearing = radians(bearing)
		lat1 = self.lat_rad
		lon1 = self.lon_rad

		lat2 = asin(sin(lat1)*cos(distance_m/R) + cos(lat1)*sin(distance_m/R)*cos(bearing))
		lon2 = lon1 + atan2(sin(bearing)*sin(distance_m/R)*cos(lat1), cos(distance_m/R) - sin(lat1)*sin(lat2))

		return LocationGlobal(lat2,lon2, coords_as_radians = True)


	def distance_between(self, location2):
		"""
		Takes the second coordinate as LocationGlobal objects and returns the distance between it and the calling object.

		:param location2: second location as LocationGlobal object
		"""

		# Extract necessary information
		lon1 = self.lon_rad
		lat1 = self.lat_rad
		lon2 = location2.lon_rad
		lat2 = location2.lat_rad
		dlon = lon2 - lon1 
		dlat = lat2 - lat1 

		# haversine formula 
		a = sin(dlat/2)**2 + cos(lat1)*cos(lat2)*sin(dlon/2)**2
		c = 2*asin(sqrt(a)) 
		dist_meters = (6367 * c)*1000

		return dist_meters


	def bearing_between(self, location2):
		"""
		Takes two coordinates as LocationGlobal objects and returns the compass bearing between the two

		:param location1: first location as LocationGlobal object
		:param location2: second location as LocationGlobal object
		"""

		# Extract necessary information
		lon1 = self.lon_rad
		lat1 = self.lat_rad
		lon2 = location2.lon_rad
		lat2 = location2.lat_rad
		dlon = lon2 - lon1 
		dlat = lat2 - lat1 

		#Bearing between the two points
		x = (sin(dlon)*cos(lat2))*1000
		y = ((cos(lat1)*sin(lat2)) - (sin(lat1)*cos(lat2)*cos(dlon)))*1000
		initial_bearing = atan2(x, y)
		initial_bearing = degrees(initial_bearing)

		#atan2 returns values from -180 to 180 which is not what we want for a compass
		compass_bearing = (initial_bearing + 360) % 360

		return compass_bearing


	def get_location_north_east(self, dNorth, dEast):
		"""
		Returns a LocationGlobal object of the coordinate `dNorth` and `dEast` metres from location represented 
		by the calling object. The returned Location has the same `alt` value as `original_location`.

		:param dNorth: Amount of meters north we are from the original location
		:param dEast: Amount of meters east we are from the original location
		"""
		earth_radius=6378137.0 #Radius of "spherical" earth

		#Coordinate offsets in radians
		dLat = dNorth/earth_radius
		dLon = dEast/(earth_radius*cos(self.lat_rad))

		#New position in decimal degrees
		newlat = self.lat + degrees(dLat)
		newlon = self.lon + degrees(dLon)

		return LocationGlobal(newlat, newlon)


	def __str__(self):
		return "LocationGlobal:lat=%s,lon=%s,alt=%s" % (self.lat, self.lon, self.alt)


class Coordinate(object):
	"""
	A coordinate object containing x/y coordinates
	"""

	def __init__(self, x, y):
		"""
		:param x: x-coordinate
		:param y: y-coordinate
		"""

		self.x = x
		self.y = y


	@classmethod
	def generate_coord_from_geo(cls, location1, location2):
		distance_between_points = location1.distance_between(location2)
		bearing_between_points = location1.bearing_between(location2)
		return cls.generate_rel_coordinate(distance_between_points, bearing_between_points)


	@classmethod
	def generate_rel_coordinate(cls, distance, bearing):
		"""
		Return x/y coordinates reresentative of how far north/east (can be negative) a point is relative to some 0 point.

		:param distance: Distance from some center point
		:param bearing: Bearing between the center point and current point in that order.
		"""
		x = distance*sin(radians(bearing))
		y = distance*cos(radians(bearing))

		return cls(x,y)


	def adjust_origin(self, adj_x, adj_y):
		"""
		Adjust the coordinates, making them relative to a new origin

		:param adj_x: Amount to adjust current x coordinate
		:param adj_x: Amount to adjust current y coordinate
		"""
		self.x += adj_x
		self.y += adj_y


	def __str__(self):
		return "Coordinate:x=%s,y=%s" % (self.x, self.y)


class Line(object):
	"""
	A Linear function object containing the gradient and y-intercept

	:param grad: gradient of the line
	:param y_int: y-intercept of the line
	"""

	def __init__(self, grad, y_int):
		self.grad = grad
		self.y_int = y_int


	@classmethod
	def generate_line(cls,coord1,coord2):
		"""
		Return the gradient and intercept describing the straight line between two given points made up by x and y coordinates.

		:param coord1: coordinate containing distance (east/west) of first point from the origin
		:param coord2: coordinate containing distance (east/west) of second point from the origin
		"""

		if (coord1.x != coord2.x):
			gradient = (coord2.y-coord1.y)/(coord2.x-coord1.x)
			intercept = coord1.y - gradient*coord1.x
		else:
			#We approximate a line paralell to the y-axis (North) by having a very small gradient
			gradient = (coord2.y-coord1.y)/0.0000001

		return cls(gradient,intercept)


	def shift_intercept(self, shift, inwards = False):
		"""
		Shift the y-intercept of a line proportional to the left shift specified and return the new y-intercept

		:param shift: Amount in meters we would like to shift our y-intercept by.
		:param inwards: When true our change in y-intercept will move the line inwards, therefore shrinking the geofence.
		"""
		if inwards:
			self.y_int += shift
		else:
			self.y_int -= shift

		return self.y_int

	def calculate_intersection(self, line2):
		"""
		Calculate the intercept between two straight lines and return corresponding :py:class: `Coordinate`
		
		:param line2: Line of interest (aside from calling object line)
		"""

		#If our lines aern't paralell then there is an intersection
		if (self.grad != line2.grad):
			x_int = (line2.y_int - self.y_int)/(self.grad - line2.grad)
			y_int = self.grad*x_int + self.y_int
			return Coordinate(x_int,y_int)
		else:
			return None


	def __str__(self):
		return "Linear line: y=%s*x + %s" % (self.grad, self.y_int)


class Geofence(object):
	"""
	A class representing a geofence, which is comprised of various gps coordinates

	This is a collection of :py:class:`LocationGlobal` objects
	"""

	AbsoluteCoord = 0

	def __init__(self, coords):
		"""
		:param coords: Array of LocationGlobal objects
		"""

		self.coords = coords
		self.num_coords = len(coords)
		self.kml_string = self.generate_kml_rep()


	def generate_kml_rep(self):
		"""
		Generates a string that can be used to fill a kml <coordinates> tag
		"""
		geofence_coords = []

		for coord in self.coords:
			str_lat = str(coord.lat)
			str_lon = str(coord.lon)
			str_comb = "%s,%s,0" % (str_lon,str_lat)
			geofence_coords.append(str_comb)

		return ' '.join(geofence_coords)


	def shrink_geofence(self, distance):
		"""
		Shrink the geofence by calling mod_geofence

		:param distance: distance to shrink geofence by
		"""
		return self.mod_geofence(distance, shrink = True)

	def enlarge_geofence(self, distance):
		"""
		Enlarge the geofence by calling mod_geofence

		:param distance: distance to enlarge geofence by
		"""
		return self.mod_geofence(distance, shrink = False)

	def get_line_representations(self):
		"""Get linear equations representing geofence"""
		if self.lineRep == null:
			self.generate_line_representation()

		return self.lineRep

	def generate_line_representation(self):
		"""
		Generates linear equations using an absolute coordinate defined by our initial geofence
		"""

		# Our first gps coordinate shall be defined using the absolute coord created with first geofence
		prev_coord = Coordinate.generate_coord_from_geo(Geofence.AbsoluteCoord, self.coords[0])

		# Store line representations
		self.lineRep = []

		# Store distance between points
		self.dbp = []

		# Store bearing between points
		self.bbp = []

		# Store new coordinates generated by distance and bearing to the absolute coordinate
		self.coords_xy = []

		for i in range(0, self.num_coords - 1):

			self.coords_xy.append(prev_coord)

			#Grab current and next gps coordinates
			current_gps_coord = self.coords[i]
			next_gps_coord = self.coords[i+1]

			#Find the distance and bearing between our two points
			distance_between_points = current_gps_coord.distance_between(next_gps_coord)
			bearing_between_points = current_gps_coord.bearing_between(next_gps_coord)

			self.dbp.append(distance_between_points)
			self.bbp.append(bearing_between_points)

			"""
			Generate the next absolute coordinate (relative to (0,0)).

			The last coordinate of a geofence is equal to the first coordinate, therefore we can set the last
			coordinate to be at (0,0). This is to stop rounding errors from generating a different point.
			"""
			if (i != self.num_coords - 2):
				next_coord = Coordinate.generate_rel_coordinate(distance_between_points, bearing_between_points)
				next_coord.adjust_origin(prev_coord.x, prev_coord.y)
			else:
				next_coord = Coordinate.generate_coord_from_geo(Geofence.AbsoluteCoord, self.coords[0])

			#Find the linear line between the two coordinates and store it in objects array
			self.lineRep.append(Line.generate_line(prev_coord,next_coord))

			prev_coord = next_coord

		#Append the final coordinate 
		self.coords_xy.append(prev_coord)
		return self.lineRep

	def mod_geofence(self, distance, shrink):
		"""
		Shrinks the geofence represented by the calling object by `distance`

		:param distance: Distance to shrink geofence by.
		"""

		mod_geofence = []
		coordinates = []

		if shrink:
			inwards = True
		else:
			inwards = False

		# Generate the linear lines between every adjacent set of points
		self.generate_line_representation()

		# To get the starting coordinate equaling the last coordinate we use the last linear equation stored
		prev_line = self.lineRep[-1]
		shift_distance = calculate_shift_distance(distance, self.bbp[-1])
		prev_line.shift_intercept(shift_distance, inwards = inwards)

		for i in range(0, self.num_coords - 1):

			current_line = self.lineRep[i]
			#Shifting the y-intercept moves the line, this shift will be proportional to the bearing.
			if (i != self.num_coords - 2):
				shift_distance = calculate_shift_distance(distance, self.bbp[i])
				current_line.shift_intercept(shift_distance, inwards = inwards)

			#Store the first line generated between two geofence points
			if (i == 0):
				initial_line = current_line

			# Calculate the intersection between the two shifted lines which is our new coordinate
			intersection = prev_line.calculate_intersection(current_line)
			coordinates.append(intersection)

			prev_line = current_line

		# Calculate the intersection between the lines connected to the starting point
		intersection = prev_line.calculate_intersection(initial_line)
		coordinates.append(intersection)

		# Convert all intersection coordinates back into gps coordinates represented by :py:class:`LocationGlobal` objects
		for i in range(0, self.num_coords):
			mod_geofence.append(self.coords[0].get_location_north_east(coordinates[i].y, coordinates[i].x))

		return Geofence(mod_geofence)

	def is_coord_inside(self, location):

		#Generate a linear line approximation of the geofence if it doesn't already exist
		if not hasattr(self, 'lineRep'):
			self.generate_line_representation()

		# We need the bearing between the absolute (0,0) coord and our point of interest to later check if we are inside the fence.
		bearing = Geofence.AbsoluteCoord.bearing_between(location)

		# Generate a coordinate for our point of interest (poi)
		poi = Coordinate.generate_coord_from_geo(Geofence.AbsoluteCoord, location)

		# Now create a linear line from the origin using our poi
		poi_line = Line.generate_line(Coordinate(0,0), poi)

		# Keep track of how many times our poi_line intersects any of the lines that make up the geofence
		fence_intersections = 0

		for i in range(0,self.num_coords - 1):

			#Find the intersection between the poi_line and the current geofence line
			intersection = poi_line.calculate_intersection(self.lineRep[i])

			#If the lines are paralell return control to the beginning of the loop
			if intersection is None:
				continue

			""" 
			Since all lines extend to infinity (aern't rays) then unless we have paralell lines every other 
				combination of lines will intersect. Therefore we define the acceptable range that this intersection
				can happen in, in order for it to count. This is possible because we know the coordinates (x,y) of each
				gps point that makes up our geofence.
			"""
			if (self.coords_xy[i].x > self.coords_xy[i+1].x):
				x_largest = self.coords_xy[i].x
				x_smallest = self.coords_xy[i+1].x
			else:
				x_largest = self.coords_xy[i+1].x
				x_smallest = self.coords_xy[i].x

			if (self.coords_xy[i].y > self.coords_xy[i+1].y):
				y_largest = self.coords_xy[i].y
				y_smallest = self.coords_xy[i+1].y
			else:
				y_largest = self.coords_xy[i+1].y
				y_smallest = self.coords_xy[i].y

			"""
			In order to find if the poi is inside the fence we need to know if the number of valid intersections our poi_line
				makes with the geofence are odd or even (Ray Casting Algorithm). For this we would usually use a ray from the poi 
				however we define an infinite line from the origin that intersects our poi. We can treat this line like a ray
				by discounting any intersections that occur between the origin and the poi. To achieve this we split the coordinate 
				space into quadrants about the poi, and check which quadrant the origin falls in using the bearing. We can then 
				define a condition on the x and y coordinates of the intersection, that they must be bigger/smaller than the poi x
				and y coordinates depending on which quadrant the origin was in.
			"""
			if (bearing == 0):
				condition = (intersection.y > poi.y)
			elif ((0 < bearing) & (bearing < 90)):
				condition = (intersection.x > poi.x) & (intersection.y > poi.y)
			elif (bearing == 90):
				condition = (intersection.x > poi.x)
			elif ((90 < bearing) & (bearing < 180)):
				condition = (intersection.x > poi.x) & (intersection.y < poi.y)
			elif (bearing == 180):
				condition = (intersection.y < poi.y)
			elif ((180 < bearing) & (bearing < 270)):
				condition = (intersection.x < poi.x) & (intersection.y < poi.y)
			elif (bearing == 270):
				condition = (intersection.x < poi.x)
			elif ((270 < bearing) & (bearing < 360)):
				condition = (intersection.x > poi.x) & (intersection.y > poi.y)

			# If all conditions are met, then we count the intersection as truly valid
			if (condition & (intersection.x < x_largest) & (intersection.x > x_smallest) & (intersection.y < y_largest) & (intersection.y > y_smallest)):
				fence_intersections+=1

		# If the number of intersections is odd we are inside the fence, otherwise we are outside
		if (fence_intersections%2 != 0):
			return True
		else:
			return False


	def __str__(self):
		return "Geofence comprised of %s points, at location lat: %s, lon: %s" % (self.num_coords, self.coords[0].lat, self.coords[0].lon)


class KmlGenerator(object):
	"""A .kml file generator"""

	def __init__(self, document_name, styleKey, generate_template = True):
		"""
		:param document_name: The name used to display the kml file in google maps
		:param styleKey: Can be 'outline', 'danger_fill' or 'safe_fill' depending on how you would like to display the line/polygon in google maps.
		"""

		self.document_name = document_name

		if generate_template:
			self.doc = self.generate_kml_template(styleKey)


	def style_lookup(self, key):
		""" 
		A dictionary holding different kml plot styles
        
        The order of expression is aabbggrr, where aa=alpha (00 to ff); bb=blue (00 to ff); gg=green (00 to ff); rr=red (00 to ff)

		:param key: Used to retrieve the corresponding style information
		"""
		dictionary = {
			'outline': {
				'name': "black_outline",
				'lineColor': "ff000000",
				'polyColor': "00000000",
				'lineWidth': 10,
				'highlightLine': "99000000",
				'highlightPoly': "00000000",
			},
			'danger_fill': {
				'name': "red_fill",
				'lineColor': "7f1400d2",
				'polyColor': "7f1400d2",
				'lineWidth': 10,
				'highlightLine': "661400d2",
				'highlightPoly': "661400d2",
			},
			'safe_fill': {
				'name': "green_fill",
				'lineColor': "b23cb414",
				'polyColor': "b23cb414",
				'lineWidth': 10,
				'highlightLine': "663cb414",
				'highlightPoly': "663cb414",
			}
		}

		return dictionary[key]



	def generate_kml_template(self, styleKey):
		"""Create a .kml file skeleton"""

		self.doc = KML.kml(
			KML.name(self.document_name),
			KML.Document(
				KML.name(self.document_name)
			)
		)
		self.doc = self.append_style(self.doc, styleKey)

		self.doc.Document.append(
			KML.Folder(
				KML.name(self.document_name),
				KML.open(1),
				KML.Placemark(
					KML.name(self.document_name),
				)
			)
		)

		return self.doc


	def append_style(self, doc, styleKey):
		"""
		Appends a style map to the document

		:param doc: The current kml document
		:param styleKey: The key used to lookup colors and names used in each style
		"""

		self.style = self.style_lookup(styleKey)

		self.stylename = self.style['name']
		self.styleMapName = self.stylename + '_map'

		doc.Document.append(
			KML.Style(
				KML.LineStyle(
					KML.color(self.style['lineColor']),
				),
				KML.PolyStyle(
					KML.color(self.style['polyColor']),
				),
				id=self.stylename,
			)
		)
		doc.Document.append(
			KML.Style(
				KML.LineStyle(
					KML.color(self.style['highlightLine']),
				),
				KML.PolyStyle(
					KML.color(self.style['highlightPoly']),
				),
				id=self.stylename + "_highlight",
			)
		)
		doc.Document.append(
			KML.StyleMap(
				KML.Pair(
					KML.key("normal"),
					KML.styleUrl("#"+self.stylename),
				),
				KML.Pair(
					KML.key("highlight"),
					KML.styleUrl("#"+self.stylename+"_highlight"),
				),
				id=self.styleMapName,
			)
		)

		return doc



	def make_kml_geofence(self, geofence):
		"""
		Given a :py:class:`Geofence` generate a corresponding styled kml document.

		:param geofence: A :py:class:`Geofence` object containing all gps coordinates (lat/lon) making up the geofence 
		"""

		# Set the centre view location of a map to the first geopoint
		self.viewLocation = geofence.coords[0]

		self.doc.Document.Folder.Placemark.append(
			KML.LookAt(
				KML.longitude(self.viewLocation.lon),
				KML.latitude(self.viewLocation.lat),
				KML.altitude(0),
				KML.heading(0),
				KML.tilt(0),
				KML.range("10595.78450948416"),
			)
		)
		self.doc.Document.Folder.Placemark.append(
			KML.styleUrl("#"+self.styleMapName)
		)
		self.doc.Document.Folder.Placemark.append(
			KML.Polygon(
				KML.tesselate(1),
				KML.outerBoundaryIs(
					KML.LinearRing(
						KML.coordinates(geofence.kml_string)
					)
				)
			)
		)

		with open("./kml_modGeofence/Shrunk_Geofence.kml", "w") as text_file:
			text_file.write("""<?xml version="1.0" encoding="UTF-8"?>\n""")
			text_file.write(etree.tostring(self.doc, pretty_print=True))


class KmlFile():
	"""
	A .kml file object generated by a parsed kml file (as an xml string)
	"""

	def __init__(self, xml_string, filename):
		"""
		:param xml_string: An xml structured file (heirarchical with tags) that has been converted to a string.
		"""

		#fromstring parses xml from a string directly into an element (root element of parsed tree)
		#root has a tag, a dictionary of attributes, and also children nodes.
		self.root = etree.fromstring(xml_string)

		self.filename = filename

		self.def_namespace = self.get_default_namespace()


	def get_default_namespace(self):
		"""
		Gets default namespace (schema it follows) of the kml document

		All tags are prefixed by the default namespace (e.g {http://www.opengis.net/kml/2.2}) Therefore we extract the 
		namespace inside the {} which is done below.

		:param kml_root: A :py:class:`Element` which is the root element of the parsed tree, containing all other elements.
		"""
		return self.root.tag.split('}')[0][1:]


	def get_element(self, root_elem, element):
		"""
		Find spefic elements that have root_elem as an ancestor 

		Note: If there is a default namespace, that full URI gets prepended to all of the non-prefixed tags

		:param root_elem: The root element whose subelements we shall search
		:param element: The element (tag) of interest 
		"""
		return root_elem.findall(".//{%s}%s" % (self.def_namespace, element))


	def get_coords(self, elem_type):
		"""
		Get the coordinates of a specific tag in the kml file

		:param elem_type: The element (tag) whose coordinates we would like to find.
		"""

		#Placemarks are features with associated geometry (ie point, polygon) that contain the different features of interest
		placemark_elems = self.get_element(self.root, 'Placemark')

		coord_string = []
		names = []
		location = None

		for elem in placemark_elems:

			#If no child element is of the necessary type, this will return null.
			related_subelement = self.get_element(elem, self.kml_element_lookup(elem_type))

			if related_subelement:
				#Get the name given to each placemark
				names.append(self.get_element(elem, 'name')[0].text)

				#Find all coordinate elements that are subelements of the most relevant tag
				coords = self.get_element(elem, 'coordinates')

				#Make sure coordinate string starts with the first longitude
				for coord in coords:
					m = re.search("\d", coord.text)
					coord_string.append(coord.text[m.start():])

		if elem_type == 'geofence':
			location = self.get_geofence(coord_string)
		elif elem_type == 'home':
			location = self.get_home_location(coord_string, names)

		return location

	def change_geo_color(self):
		"""
		Change the geofence color in existing kml file to red

		Insanely hacky, NEED TO CHANGE!
		"""

		#Placemarks are features with associated geometry (ie point, polygon) that contain the different features of interest
		placemark_elems = self.get_element(self.root, 'Placemark')
		Document = self.get_element(self.root, 'Document')
		Folder = self.get_element(self.root, 'Folder')

		#Define different strings representing kml tags and parse them as xml
		content_normal = '''\
		<Style id="red_fill">
			<LineStyle>
				<color>7f1400d2</color>
			</LineStyle>
			<PolyStyle>
				<color>7f1400d2</color>
			</PolyStyle>
		</Style>
		'''

		content_highlight = '''
		<Style id="red_fill_highlight">
			<LineStyle>
				<color>661400d2</color>
			</LineStyle>
			<PolyStyle>
				<color>661400d2</color>
			</PolyStyle>
		</Style>
		'''

		content_map = '''
		<StyleMap id="red_fill_map">
			<Pair>
				<key>normal</key>
				<styleUrl>#red_fill</styleUrl>
			</Pair>
			<Pair>
				<key>highlight</key>
				<styleUrl>#red_fill_highlight</styleUrl>
			</Pair>
		</StyleMap>
		'''

		content_line = '''
		<Style id="black_outline">
			<LineStyle>
				<color>ff000000</color>
			</LineStyle>
			<PolyStyle>
				<color>ff000000</color>
			</PolyStyle>
			<LineStyle>
				<width>3</width>
			</LineStyle>
		</Style>
		'''
		content_normal_tree = etree.fromstring(content_normal)
		content_highlight_tree = etree.fromstring(content_highlight)
		content_map_tree = etree.fromstring(content_map)
		content_line_tree = etree.fromstring(content_line)

		#Insert these elements into the kml file
		for child in Document:
			child.insert(1,content_map_tree)
			child.insert(1,content_highlight_tree)
			child.insert(1,content_normal_tree)
			child.insert(1,content_line_tree)

		coord_string = []
		names = []
		location = None
		max_coords = 0

		for elem in placemark_elems:

			#If no child element is of the necessary type, this will return null.
			related_subelement = self.get_element(elem, self.kml_element_lookup('geofence'))

			line_string = self.get_element(elem, self.kml_element_lookup('flight path'))
			if line_string:
				#Change the styleUrl text of the geofence to the appropriate id
				styleUrl = self.get_element(elem, 'styleUrl')[0]
				styleUrl.text = "#black_outline"

			if related_subelement:

				#Find the element that corresponds to the geofence (HACKY: checks size of coordinate text)
				coords = self.get_element(elem, 'coordinates')
				if (len(coords[0].text[0:]) > max_coords):
					max_coords = len(coords[0].text[0:])
					geofence = elem

		#Change the styleUrl text of the geofence to the appropriate id
		styleUrl = self.get_element(geofence, 'styleUrl')[0]
		styleUrl.text = "#red_fill_map"

		#Write to the file to update it.
		with open(self.filename, "w") as f:
			f.write("""<?xml version="1.0" encoding="UTF-8"?>\n""")
			f.write(etree.tostring(self.root, pretty_print = True))

		return


	def get_home_location(self, coord_strings, names):
		"""
		Find the point coordinate corresponding to base location, and return the corresponding :py:class:`LocationGlobal` object

		:param coord_strings: There can be multiple tags with strings representing "lat,lon,alt", so we differentiate using the name
		:param names: We search through names and see if the name 'Base' exists. This is our home location.
		"""
		index = names.index('Base')
		home_location = coord_strings[index]
		home_location = self.make_tidy_array(home_location)

		hl_longitude = home_location[0]
		hl_latitude = home_location[1]
		hl_altitude = home_location[2]

		return LocationGlobal(hl_latitude, hl_longitude, hl_altitude)


	def make_tidy_array(self, string_list):
		"""
		Split the string of gps coords so each element is either a longitude, latitude, or altitude

		:param string_list: Coordinate string that we wish to trim down.
		"""
		array_split = re.split(r'[;,\s]\s*', string_list)
		array_split = filter(None, array_split)
		array_floats = [float(i) for i in array_split]
		return array_floats


	def get_geofence(self, coord_string):
		"""
		Find the longest coordnate string which should be the geofence, and return an array of :py:class: `LocationGlobal` objects

		:param coord_string: An array of strings representing "lat,lon,alt". This has to be trimmed to remove unncessasary characters.
		"""

		maxlength = max(len(s) for s in coord_string)
		geofence = [s for s in coord_string if len(s) == maxlength]
		geofence = self.make_tidy_array(geofence[0])

		geofence_object_arr = []



		geof_longitude = geofence[0::3]
		geof_latitude = geofence[1::3]
		geof_altitude = geofence[2::3]

		for index in range(len(geof_longitude)):
			geofence_object_arr.append(LocationGlobal(geof_latitude[index],geof_longitude[index],geof_altitude[index]))

		return Geofence(geofence_object_arr)


	def kml_element_lookup(self, key):
		"""
		Lookup container element for different objects of interest (ie geofence or home location)

		geofence -> polygon tag
		home -> point tag

		:param key: Used to retreive the tag corresponding to a certain feature of interest
		"""

		dictionary = {
			'geofence': 'Polygon',
			'home': 'Point',
			'flight path': 'LineString'
		}

		return dictionary[key]



def unzip_kmz(fileName):
	"""
	Unzip the kmz file in the current directory

	:param fileName: Used to find and unzip kmz file with identical file name
	"""
	zip_ref = zipfile.ZipFile(fileName, 'r')
	contents = zip_ref.namelist()
	zip_ref.extractall("./kml_files")
	zip_ref.close()

	for file in contents:
		if file.endswith(".kml"):
			kml_file_name = file	#Kmz will only have one kml file

	return rename_kml('kml_files/' + kml_file_name)


def rename_kml(fileName):
	"""
	Rename kml files to allow us to easily see the most recent

	:param fileName: Used to find and rename file with identical file name
	"""
	current_time = datetime.now().strftime("%H_%M_%S")
	os.rename(fileName, 'kml_files/mission_' + current_time + '.kml')

	return 'kml_files/mission_' + current_time + '.kml'


def calculate_shift_distance(distance_normal, bearing):
	"""
	Given a distance normal to our line calculate the necessary shift of the y-intercept

	This distance depends on the compass bearing between the two points

	:param distance_normal: Amount in meters we would like to shift our line in the direction of the normal unit vector
	:param bearing: The compass bearing where 0 degrees represents North (our line being paralell to it).
	"""
	if (bearing%180 == 0):
		#We approximate a line paralell to the y-axis (North) by having a very small gradient
		bearing = 0.000001

	return distance_normal/sin(radians(bearing))


def get_most_recent_file(directory):
	"""
	Return the most recently modified/created file as a string

	:param directory: Directory relative to current path that we should search for most recently created file.
	"""
	newest_file = min(glob.iglob('%s/*.kml' % directory), key=os.path.getctime)

	return newest_file


def get_geofence(orig_fileName):
	"""
	Returns a :py:class:`LocationGlobal` object represeting the shrunken geofence.

	This method is usually called with ``create_kml = True`` to ensure a kml file representing the shrunken
	geofence is created.

	:param origFileName: The KML/KMZ file to open
	"""

	#Unzip and rename if in kmz format, otherwise simply rename
	if ".kmz" in orig_fileName:
		file = unzip_kmz(orig_fileName)
	else:
		file = rename_kml('kml_files/' + orig_fileName)

	#Directory can contain more than one kml file so find the latest kml file
	#file = get_most_recent_file('kml_files')
	
	#Open the kml file ad obtain the longitude, latitude, and altitudes of the geofence
	with open(file, 'r') as f:

		kml_file = KmlFile(f.read(), file)
		geofence = kml_file.get_coords('geofence')
		Geofence.AbsoluteCoord = geofence.coords[0]
		kml_file.change_geo_color() #Incredibly hacky, if something goes wrong its probably this.
		#home_location= kml_file.get_coords('home')

	return geofence


def modify_geofence(geofence, distance, create_kml=False, shrink = False, enlarge = False):
	"""
	Returns a :py:class:`LocationGlobal` object represeting the shrunken geofence.

	This method is usually called with ``create_kml = True`` to ensure a .kml file representing the shrunken
	geofence is created.

	:param geofence: A :py:class:`Geofence` object representing the geofence to be modified
	:param distance: Distance to shrink or enlarge geofence by
	:param create_kml: If True create a kml file representing the shrunken geofence
	:param shrink: If True shrink the geofence boundary in by `distance`
	:param enlarge: If True enlarge the geofence boundary by `distance`
	"""

	if shrink:
		geofence_mod = geofence.shrink_geofence(distance)
	elif enlarge:
		geofence_mod = geofence.enlarge_geofence(distance)
	else:
		sys.exit("modify_geofence() must be called with either enlarge or shrink set to True")

	if create_kml:
		geofence_mod_kml = KmlGenerator("Shrunk Geofence", "safe_fill")
		geofence_mod_kml.make_kml_geofence(geofence_mod)
	
	return geofence_mod


if __name__ == "__main__":

	parser = argparse.ArgumentParser(description='Allows a soft geofence to be created')
	parser.add_argument('-f', '--file', 
		help=".kmz/.kml file to be parsed.")
	parser.add_argument('-d', '--distance', type = int,
		help="Distance to shrink or enlarge geofence by.")
	parser.add_argument('-kml', '--create_kml', type= bool,
		help="If True a kml file with the soft geofence is generated.")
	parser.add_argument('-sh', '--shrink', type= bool,
		help="If True the geofence with be shrunk by some distance.")
	parser.add_argument('-en', '--enlarge', type= bool,
		help="If True the geofence with be enlarged by some distance.")
	args = parser.parse_args()

	if not args.distance:
		sys.exit("A distance must be provided")
	if not args.file:
		sys.exit("A file name must be provided to proceed")

	geofence = get_geofence(args.file)
	Geofence.AbsoluteCoord = geofence.coords[0]
	shrunken_geofence = modify_geofence(geofence, args.distance, create_kml=args.create_kml, shrink = args.shrink, enlarge = args.enlarge)



#Frame format
# QGC WPL <VERSION>
# <INDEX> <CURRENT WP> <COORD FRAME> <COMMAND> <PARAM1> <PARAM2> <PARAM3> <PARAM4> <PARAM5/X/LONGITUDE> <PARAM6/Y/LATITUDE> <PARAM7/Z/ALTITUDE> <AUTOCONTINUE>

# Index -> What number command
# Current WP -> ??
# Coord Frame -> Essentially whether the altitude is absolute (measured from sea level) or relative to ground : 0 for absolute, 3 for relative
# Command -> What kind of message are we sending/receiving, ie. Waypoint, set paramater, ect.
# Param 1 -> Hold time in decimal seconds (Ignored by fixed wing)
# Param 2 -> Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)
# Param 3 -> 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
# Param 4 -> Desired yaw angle at MISSION (rotary wing)
# Autocontinue -> Continue mission when reached