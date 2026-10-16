# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Fit opaque foot plinths to selected planar walls and the existing floor.

Only short, source-pinned wall/floor gaps qualify. Driving surfaces and larger
passages are not extended or flattened. Coordinates and tolerances use metres.
"""
import numpy as np


def floor_height(faces, x, y):
    """Return the highest floor triangle at a CAD x/y point, rejecting missing floor."""
    a = faces[:, 0, :2]
    v, w = faces[:, 1, :2]-a, faces[:, 2, :2]-a
    determinant = v[:, 0]*w[:, 1]-v[:, 1]*w[:, 0]
    valid = np.abs(determinant) > 1e-14
    delta = np.array([x, y])-a
    s, t = np.zeros(len(faces)), np.zeros(len(faces))
    s[valid] = (delta[valid, 0]*w[valid, 1]-delta[valid, 1]*w[valid, 0])/determinant[valid]
    t[valid] = (v[valid, 0]*delta[valid, 1]-v[valid, 1]*delta[valid, 0])/determinant[valid]
    valid &= (s >= -1e-7) & (t >= -1e-7) & (s+t <= 1+1e-7)
    if not valid.any():
        raise ValueError('no floor beneath the selected side wall')
    return float(np.max(faces[valid, 0, 2]+s[valid]*(faces[valid, 1, 2]-faces[valid, 0, 2])
                        +t[valid]*(faces[valid, 2, 2]-faces[valid, 0, 2])))


def lower_edge(faces_uv, u):
    """Maximum downward local coordinate of a triangle section at horizontal u."""
    cuts = []
    for triangle in faces_uv:
        for a, b in zip(triangle, np.roll(triangle, -1, axis=0)):
            if min(a[0], b[0]) <= u <= max(a[0], b[0]) and abs(a[0]-b[0]) > 1e-10:
                cuts.append(a[1]+(b[1]-a[1])*(u-a[0])/(b[0]-a[0]))
    return max(cuts) if cuts else None


def foot_plinths(wall_faces, floor_faces, normal, level, thickness_m=.003,
                 min_gap_m=.010, max_gap_m=.080, min_span_m=.100):
    """Bridge short gaps to the floor with closed, sloped rectangular prisms.

    `level` is the desired exterior plane dot(normal, point). Faces within
    0.2 mm of this plane supply the wall's bottom profile. Top and floor edges
    are checked at each interval's ends and midpoint; intervals crossing a
    floor crease or non-linear bottom edge are refused rather than flattened.
    Returns vertices, triangles and measurements; no input geometry is removed.
    """
    n = np.array(normal, dtype=float, copy=True)
    n /= np.linalg.norm(n)
    u = np.cross(n, [0, 0, 1])
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    if v[2] > -.9:
        raise ValueError('expected a nearly vertical wall with downward local V')
    basis = np.column_stack([u, v, n])
    selected = np.abs(wall_faces @ n-level).max(axis=1) < .0002
    faces = wall_faces[selected] @ basis
    if not len(faces):
        raise ValueError('no wall at the requested plinth plane')
    coordinates = np.sort(np.unique(faces[:, :, 0]))
    coordinates = coordinates[np.r_[True, np.diff(coordinates) > 1e-5]]

    def floor_v(horizontal):
        downward = 0.
        for _ in range(8):
            point = np.array([horizontal, downward, level]) @ basis.T
            z = floor_height(floor_faces, point[0], point[1])
            downward = (z-horizontal*u[2]-level*n[2])/v[2]
        point = np.array([horizontal, downward, level]) @ basis.T
        if abs(point[2]-floor_height(floor_faces, point[0], point[1])) > 1e-7:
            raise ValueError('floor intersection did not converge')
        return downward

    runs = []
    for a, b in zip(coordinates, coordinates[1:]):
        if b-a < 1e-5:
            continue
        positions = [a+(b-a)*1e-6, (a+b)/2, b-(b-a)*1e-6]
        upper = [lower_edge(faces, x) for x in positions]
        if any(x is None for x in upper):
            continue
        lower = [floor_v(x) for x in positions]
        gaps = (np.array(lower)-upper)*abs(v[2])
        if not (np.all(gaps >= min_gap_m) and np.all(gaps <= max_gap_m)):
            continue
        if abs(upper[1]-(upper[0]+upper[2])/2) > 1e-5 or abs(lower[1]-(lower[0]+lower[2])/2) > 1e-5:
            raise ValueError('plinth interval crosses a non-linear boundary')
        # Extrapolate the interior samples to the exact shared breakpoints.
        top_slope = (upper[2]-upper[0])/(positions[2]-positions[0])
        bottom_slope = (lower[2]-lower[0])/(positions[2]-positions[0])
        top_a, top_b = [upper[1]+(x-positions[1])*top_slope for x in [a, b]]
        bottom_a, bottom_b = [lower[1]+(x-positions[1])*bottom_slope for x in [a, b]]
        run = [a, b, top_a, top_b, bottom_a, bottom_b]
        if runs:
            previous = runs[-1]
            same = (abs(previous[1]-a) < 1e-5 and abs(previous[3]-top_a) < 1e-5
                    and abs(previous[5]-bottom_a) < 1e-5
                    and abs((previous[3]-previous[2])/(previous[1]-previous[0])-top_slope) < 1e-5
                    and abs((previous[5]-previous[4])/(previous[1]-previous[0])-bottom_slope) < 1e-5)
            if same:
                previous[1], previous[3], previous[5] = b, top_b, bottom_b
                continue
        runs.append(run)
    vertices, triangles, records = [], [], []
    for a, b, ta, tb, fa, fb in runs:
        if b-a < min_span_m:
            continue
        # The outline is CCW in local U/V, with the inward cap at +normal.
        polygon = np.array([[a, ta-1e-5], [b, tb-1e-5], [b, fb+1e-5], [a, fa+1e-5]])
        local = np.vstack([np.column_stack([polygon, np.full(4, depth)])
                           for depth in [level, level+thickness_m]])
        t = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
                      [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5],
                      [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]])
        offset = len(vertices)*8
        vertices.append(local @ basis.T)
        triangles.append(t+offset)
        records.append({'span_m': b-a, 'horizontal_range_m': [a, b],
                        'gap_height_m': [(fa-ta)*abs(v[2]), (fb-tb)*abs(v[2])],
                        'triangles': 12})
    if not vertices:
        return np.empty((0, 3)), np.empty((0, 3), dtype=int), []
    return np.vstack(vertices), np.vstack(triangles), records
