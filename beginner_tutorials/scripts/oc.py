from numpy import *
import scipy.linalg


def bresenham(start, end):
    """Bresenham's Line Algorithm
    MATLAB CHECK OK
    Produces a list of tuples from start and end
    >>> points1 = bresenham((0, 0), (3, 4))
    >>> print points1
    [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
    :param start: Start Coordinate
    :param end: Ending Coordinate
    :return: A list of tuples from start and end
    """
    # Setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1

    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)

    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True

    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1

    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1

    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx

    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    return points


def inverse_scanner_bres(M, N, x, y, theta, r, rmax, p_occ, p_free):
    '''
    MATLAB CHECK OK
    % Input:
    %   M = Total Height of map
    %   N = Total Width of map
    %   x = Robot x position
    %   y = Robot y position
    %   theta = Angle of laser
    %   r = Range to measured object
    %   rmax = Max range of laser
    %   p_occ = Probability of an occupied cell
    %   p_free = Probability of an unoccupied cell
    % Output:
    %   m = Matrix representing the inverse measurement model
    '''
    # Bound the robot within the map dimensions
    # x1 = max(1,min(c_[M round(x)]))
    x1 = int(maximum(1, minimum(M, round(x))))
    # y1 = max(1,min(N,round(y)))
    y1 = int(maximum(1, minimum(N, round(y))))
    # Calculate position of measured object (endpoint of ray)
    endpt = c_[x, y] + r * c_[cos(theta), sin(theta)]
    endpt = endpt[0]
    # print(endpt)
    # Bound the endpoint within the map dimensions
    # x2 = max(1,min(M,round(endpt(1))))
    x2 = int(maximum(1, minimum(M, round(endpt[0]))))
    # y2 = max(1,min(N,round(endpt(2))))
    y2 = int(maximum(1, minimum(N, round(endpt[1]))))
    # Get coordinates of all cells traversed by laser ray
    # [list(:,1), list(:,2)] = bresenham(x1,y1,x2,y2);
    points = bresenham((x1, y1), (x2, y2))
    # print(points)
    # Assign probabilities
    # m = [list p_free * ones(length(list(:, 1)), 1)];
    b = p_free * ones(len(points))
    m = c_[points, b]

    if r < rmax:
        m[-1, 2] = p_occ

    return m


def ogmap(M, N, ogl, x, phi_m, r_m, r_max):
    """ Occupancy Grid Mapping Algorithm
    M, N - size of the map we want, to formulate L0
    og - Log odds representation of occupancy at time t-1, equals to {l(t-1,i)}
    x - Current state Xt
    phi_m, r_m, r_max - Measurement, Zt
    """
    print x

    # Initial belief map
    m = 0.5 * ones((M, N))
    L0 = log(m / (1 - m))

    # Probabilities of cells
    p_occ = 0.7
    p_free = 0.3

    # The cells affected by this specific measurement in log odds (only used
    # in Bresenham ray trace mode)
    imml = zeros((M, N))

    # print(phi_m)
    # print(r_m)

    # Loop through each laser measurement
    for i in range(0, len(phi_m) - 1):

        if isnan(r_m[i]):
            # print r_m[i]
            # print "NAN!"
            continue

        # Get inverse measurement model
        inverse_model = inverse_scanner_bres(M, N, x[0], x[1], phi_m[i] + x[2],
                                      r_m[i], r_max, p_occ, p_free)

        # Loop through each cell from measurement model
        for j in range(0, len(inverse_model[:, 0]) - 1):
            ix = inverse_model[j, 0]
            iy = inverse_model[j, 1]
            il = inverse_model[j, 2]

            # Calculate updated log odds
            ogl[ix, iy] = ogl[ix, iy] + log(il / (1 - il)) - L0[ix, iy]
            imml[ix, iy] = imml[ix, iy] + log(il / (1 - il)) - L0[ix, iy]

    return {'ogl': ogl, 'imml': imml}
