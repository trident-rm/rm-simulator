#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Read-only source-CAD profile probe for the RMUC 2027 wavy road surface.

Run in rm-map-tools/ocpenv. Writes measurements only, never a mesh or package.
A face index is qualified by the required source-part SHA-256. The prospective
strip budget assumes rectangular trimming; this is not a replacement exporter.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys


def profile_indices(points, tolerance_mm):
    """Select a polyline with bounded distance at the supplied discrete samples.

    Points and tolerance use millimetres. This does not certify error between
    samples. Endpoints are retained, including for a closed or repeated segment.
    """
    import numpy as np
    points = np.asarray(points, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 2
            or not np.isfinite(points).all() or not np.isfinite(tolerance_mm)
            or tolerance_mm <= 0):
        raise ValueError('finite Nx3 samples and positive millimetre tolerance required')
    keep = {0, len(points) - 1}
    stack = [(0, len(points) - 1)]
    while stack:
        a, b = stack.pop()
        if b - a < 2:
            continue
        segment = points[b] - points[a]
        length2 = float(segment @ segment)
        samples = points[a + 1:b]
        fraction = (np.clip((samples - points[a]) @ segment / length2, 0, 1)
                    if length2 else np.zeros(len(samples)))
        distances = np.linalg.norm(samples - points[a] - fraction[:, None] * segment, axis=1)
        index = int(distances.argmax()) + a + 1
        if distances.max() > tolerance_mm:
            keep.add(index)
            stack.extend([(a, index), (index, b)])
    return sorted(keep)


def fit_periodic_profile(points_mm, extrusion_direction, periods):
    """Compare sinusoidal and semicircular profiles in the extrusion's local plane.

    Circle bumps sit on the measured baseline and use the measured pitch; their
    height equals their radius. Report residuals at all supplied CAD samples.
    """
    import numpy as np
    from scipy.optimize import least_squares
    p = np.asarray(points_mm, dtype=float)
    p = p[np.argsort(p[:, 0])]
    vertical = np.cross([1., 0., 0.], extrusion_direction)
    vertical /= np.linalg.norm(vertical)
    x = p[:, 0] - p[0, 0]
    h = (p - p[0]) @ vertical
    pitch = x[-1] / periods

    def sine(v):
        amplitude, period, phase, base, slope = v
        return base + slope*x + amplitude*(1-np.cos(2*np.pi*(x-phase)/period))/2

    fit = least_squares(lambda v: sine(v)-h, [np.ptp(h), pitch, 0, 0, 0],
                        bounds=([0, pitch*.95, -pitch*.05, -np.ptp(h), -.1],
                                [np.ptp(h)*2, pitch*1.05, pitch*.05, np.ptp(h), .1]),
                        xtol=1e-13, ftol=1e-13, gtol=1e-13)
    amplitude, period, phase, base, slope = fit.x
    delta = sine(fit.x)-h
    dx = x % pitch-pitch/2

    def circle(radius):
        return np.sqrt(np.maximum(radius*radius-dx*dx, 0))

    radius = float(least_squares(lambda v: circle(v[0])-h, [min(np.ptp(h), pitch*.49)],
                                bounds=([pitch*.001], [pitch/2])).x[0])
    circle_delta = circle(radius)-h
    return {'periods': periods,
            'local_vertical': vertical.tolist(),
            'sinusoidal': {'height_mm': float(amplitude), 'pitch_mm': float(period),
                           'phase_mm': float(phase), 'baseline_offset_mm': float(base),
                           'baseline_slope_mm_per_mm': float(slope),
                           'max_sampled_residual_mm': float(abs(delta).max()),
                           'rms_sampled_residual_mm': float(np.sqrt(np.mean(delta*delta)))},
            'semicircular': {'radius_mm': radius, 'pitch_mm': float(pitch),
                             'max_sampled_residual_mm': float(abs(circle_delta).max()),
                             'rms_sampled_residual_mm': float(np.sqrt(np.mean(circle_delta*circle_delta)))}}


def uniform_sine_budget(height_mm, width_mm, periods, error_mm, max_strip_width_mm=None):
    """Budget a uniform sine grid using its continuous interpolation-error bound.

    For height H, n cells per period have error <= H*pi^2/(4*n^2).
    Multiples of four put vertices exactly at peaks, valleys and quarter periods.
    The across-width extrusion is linear; width subdivisions affect aspect ratio
    and counts, not the surface error. The bound applies to the analytic sine,
    independently of its sampled fit residual against the source CAD.
    """
    import math
    values = [height_mm, width_mm, error_mm]
    if max_strip_width_mm is not None:
        values.append(max_strip_width_mm)
    if (any(not math.isfinite(v) or v <= 0 for v in values)
            or type(periods) is not int or periods < 1):
        raise ValueError('positive finite millimetre dimensions and integer periods required')
    cells = max(4, 4*math.ceil(math.pi/2*math.sqrt(height_mm/error_mm)/4))
    strips = 1 if max_strip_width_mm is None else math.ceil(width_mm/max_strip_width_mm)
    return {'segments_per_period': cells, 'strips_across_width': strips,
            'strip_width_mm': width_mm/strips,
            'surface_triangles': 2*cells*periods*strips,
            'analytic_interpolation_bound_mm': height_mm*math.pi**2/(4*cells*cells)}


def uniform_sine_surface_mm(height_mm, pitch_mm, width_mm, periods, error_mm,
                            max_strip_width_mm=None):
    """Generate a local x/y/z profile strip in millimetres with upward winding.

    The surface includes waves only, without side walls or end caps. Its local
    y axis is the extrusion direction; a caller must bind the frame, phase and
    baseline to source CAD before using it in a runtime package.
    """
    import numpy as np
    if not np.isfinite(pitch_mm) or pitch_mm <= 0:
        raise ValueError('positive finite pitch in millimetres required')
    budget = uniform_sine_budget(height_mm, width_mm, periods, error_mm, max_strip_width_mm)
    nx = budget['segments_per_period']*periods
    ny = budget['strips_across_width']
    x = np.linspace(0, pitch_mm*periods, nx+1)
    y = np.linspace(0, width_mm, ny+1)
    z = height_mm*(1-np.cos(2*np.pi*x/pitch_mm))/2
    vertices = np.column_stack([np.repeat(x, ny+1), np.tile(y, nx+1), np.repeat(z, ny+1)])
    a = (np.arange(nx)[:, None]*(ny+1)+np.arange(ny)[None, :]).ravel()
    b, c, d = a+1, a+ny+1, a+ny+2
    triangles = np.concatenate([np.column_stack([a, c, d]), np.column_stack([a, d, b])])
    return vertices, triangles


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-part', type=Path, required=True)
    parser.add_argument('--source-sha256', required=True)
    parser.add_argument('--face-index', type=int, required=True)
    parser.add_argument('--map-tools', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--error-mm', type=float, nargs='+', default=[.1, .25, .5, 1.])
    parser.add_argument('--periods', type=int, default=10)
    parser.add_argument('--max-strip-width-mm', type=float)
    args = parser.parse_args()
    if args.output.exists() or args.output.resolve() == args.source_part.resolve():
        parser.error('output must be new and distinct from the source')
    digest = hashlib.sha256(args.source_part.read_bytes()).hexdigest()
    if digest != args.source_sha256:
        parser.error('source-part checksum mismatch')
    import numpy as np
    if args.periods < 1 or (args.max_strip_width_mm is not None
                            and (not np.isfinite(args.max_strip_width_mm) or args.max_strip_width_mm <= 0)):
        parser.error('positive periods and optional strip width required')
    if any(not np.isfinite(e) or e <= 0 for e in args.error_mm):
        parser.error('finite positive millimetre tolerances required')
    sys.path.insert(0, str(args.map_tools.resolve() / 'python'))
    from validate_parts import read_part
    from audit_geometry import compound, shape_map
    from OCP.TopAbs import TopAbs_FACE, TopAbs_EDGE, TopAbs_IN
    from OCP.TopoDS import TopoDS
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.BRepTools import BRepTools
    from OCP.BRepClass import BRepClass_FaceClassifier
    from OCP.gp import gp_Pnt2d
    from OCP.GeomAbs import GeomAbs_SurfaceOfExtrusion, GeomAbs_BSplineCurve
    doc, _, error = read_part(str(args.source_part.resolve()))
    if error:
        raise ValueError(error)
    faces = shape_map(compound(doc), TopAbs_FACE)
    if not 1 <= args.face_index <= faces.Extent():
        parser.error('face index outside imported source')
    face = TopoDS.Face(faces.FindKey(args.face_index))
    surface = BRepAdaptor_Surface(face)
    if surface.GetType() != GeomAbs_SurfaceOfExtrusion:
        raise ValueError('expected a source extrusion surface')
    curve = surface.BasisCurve()
    if curve.GetType() != GeomAbs_BSplineCurve:
        raise ValueError('expected a BSpline profile')
    u0, u1, v0, v1 = BRepTools.UVBounds_s(face)
    if shape_map(face, TopAbs_EDGE).Extent() != 4:
        raise ValueError('expected a four-edge trim')
    for u in np.linspace(u0, u1, 9)[1:-1]:
        for v in np.linspace(v0, v1, 9)[1:-1]:
            if BRepClass_FaceClassifier(face, gp_Pnt2d(float(u), float(v)), 1e-6).State() != TopAbs_IN:
                raise ValueError('sampled parameter rectangle leaves the source trim')
    spline = curve.BSpline()
    knots = [spline.Knot(i) for i in range(1, spline.NbKnots() + 1)
             if u0 < spline.Knot(i) < u1]
    parameters = np.unique(np.r_[np.linspace(u0, u1, 16385), knots])
    points = np.array([surface.Value(float(u), v0).Coord() for u in parameters])
    # A separate, denser grid checks chord interpolation between the retained
    # parameters; it is not the point-to-line metric used to select samples.
    check_u = np.linspace(u0, u1, 32770)
    check = np.array([surface.Value(float(u), v0).Coord() for u in check_u])
    results = []
    for tolerance in args.error_mm:
        keep = profile_indices(points, tolerance)
        predicted = np.column_stack([np.interp(check_u, parameters[keep], points[keep, axis])
                                     for axis in range(3)])
        results.append({'selection_tolerance_mm': tolerance,
                        'profile_vertices': len(keep),
                        'prospective_surface_triangles': 2 * (len(keep) - 1),
                        'max_independent_same_parameter_deviation_mm':
                            float(np.linalg.norm(check - predicted, axis=1).max())})
    fits = fit_periodic_profile(points, np.array(surface.Direction().Coord()), args.periods)
    uniform = [dict(error_limit_mm=e, **uniform_sine_budget(
        fits['sinusoidal']['height_mm'], v1-v0, args.periods, e, args.max_strip_width_mm))
        for e in args.error_mm]
    for entry in uniform:
        vertices, triangles = uniform_sine_surface_mm(
            fits['sinusoidal']['height_mm'], fits['sinusoidal']['pitch_mm'], v1-v0,
            args.periods, entry['error_limit_mm'], args.max_strip_width_mm)
        entry['generated_vertices'] = len(vertices)
        assert len(triangles) == entry['surface_triangles']
    report = {'schema_version': 1, 'source_part_sha256': digest,
              'face_index': args.face_index, 'uv_bounds': [u0, u1, v0, v1],
              'source_profile_knots': spline.NbKnots(),
              'selection_samples': len(parameters), 'independent_check_samples': len(check_u),
              'trim_check': 'four edges and sampled interior; not certified rectangular trim',
              'scope': 'single source surface; excludes edge stitching, walls, caps and other geometry',
              'results': results, 'periodic_fits': fits, 'uniform_sinusoidal_grids': uniform}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
