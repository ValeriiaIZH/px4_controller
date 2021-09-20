import sys
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import matplotlib.pyplot as plt


def withinObstacleSpace(point, radius, clearance):

	x = point[0]
	y = point[1]

	flag = False
	point = Point(x, y)

	# creating rectangles
	rectangle_1 = Polygon([(-5, -5), (5, -5), (5, 5), (-5, 5)])
	rectangle_2 = Polygon([(-5, 93), (5, 93), (5, 103), (-5, 103)])
	rectangle_3 = Polygon([(-3, 5), (-2, 5), (-2, 103), (-3, 103)])
	rectangle_4 = Polygon([(3, 5), (2, 5), (2, 103), (3, 103)])

	if point.distance(rectangle_1) <= radius + clearance:
		flag = True
	if point.distance(rectangle_2) <= radius + clearance:
		flag = True
	if point.distance(rectangle_3) <= radius + clearance:
		flag = True
	if point.distance(rectangle_4) <= radius + clearance:
		flag = True

	
	# creating circles


	return flag


def withinObstacleSpaceFake((x, y), radius, clearance):
	"""
	Always returns False. Only used for debugging purposes.

	:param      (x,y)):     Point to be checked
	:type       (x,y)):     tuple of (x,y) coordinates
	:param      radius:     The robot radius
	:type       radius:     float
	:param      clearance:  The robot clearance from objects
	:type       clearance:  float
	:param      plotter:    The plotter for visualization
	:type       plotter:    matplotlib.pyplot

	:returns:   Always False
	:rtype:     boolean
	"""
	return False


def generateMap(plotter=plt):
	"""
	Genarting the map for the obstacle space

	:param      plotter:  The plotter
	:type       plotter:  matplotlib.pyplot
	"""
	#circle_1 = plt.Circle((0, 18), 5, color='b')

	rectangle_1 = plt.Polygon([(-5, -5), (5, -5), (5, 5), (-5, 5)])
	rectangle_2 = plt.Polygon([(-5, 93), (5, 93), (5, 103), (-5, 103)])
	rectangle_3 = plt.Polygon([(-3, 5), (-2, 5), (-2, 103), (-3, 103)])
	rectangle_4 = plt.Polygon([(3, 5), (2, 5), (2, 103), (3, 103)])
	
	#plotter.add_artist(circle_1)
	plotter.add_line(rectangle_1)
	plotter.add_line(rectangle_2)
	plotter.add_line(rectangle_3)
	plotter.add_line(rectangle_4)


def testMain():
	# x = float(sys.argv[1])
	# y = float(sys.argv[2])

	fig, ax = plt.subplots()
	ax.set(xlim=(-10, 10), ylim=(-10, 110))
	ax.set_aspect('equal')
	# ax.plot([x], [y], color="black", marker="+", markersize=3)

	# print(withinObstacleSpace((x, y), 0.105, 0.2, plotter=ax))
	generateMap(ax)

	plt.show()


if __name__ == '__main__':
	testMain()
