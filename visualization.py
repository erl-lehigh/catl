'''
 Copyright (C) 2018-2020 Cristian Ioan Vasile <cvasile@lehigh.edu>
 Explainable Robotics Lab (ERL), Autonomous and Intelligent Robotics (AIR) Lab,
 Lehigh University
 Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Lab, Boston University
 See license.txt file for license information.
'''

import itertools as it

import numpy as np
import shapely.geometry as geom
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection


def drawPoint(viewport, point, color, style=None):
    '''Draws a point in the planar environment.'''
    if style:
        viewport.plot(point.x, point.y, color=color, **style)
    else:
        viewport.plot(point.x, point.y, color=color)

def drawRegion(viewport, shape, style=None, text=None, textStyle=None):
    '''Draws a polygonal region in a planar environment.'''
    polygon = geom.Polygon(shape)
    x, y = zip(*polygon.exterior.coords)
    if style:
        style['facecolor'] = style.get('facecolor', 'white')
        style['edgecolor'] = style.get('edgecolor', 'black')
    else:
        style = {'facecolor': 'white', 'edgecolor':'black'}
    viewport.fill(x, y, **style)
    if text:
        x, y = polygon.centroid.coords[0]
        if textStyle:
            textStyle['horizontalalignment'] = \
                                  textStyle.get('horizontalalignment', 'center')
            textStyle['verticalalignment'] = \
                                    textStyle.get('verticalalignment', 'center')
            textStyle['fontsize'] = textStyle.get('fontsize', 12)
        else:
            textStyle = {'horizontalalignment' : 'center',
                         'verticalalignment' : 'center', 'fontsize' : 12}
        viewport.text(x, y, text, **textStyle)
    return polygon.centroid.coords[0]

def drawGraph(viewport, g, node_color='blue', edge_color='black', zorder=2):
    '''Plots the given graph in the viewport.'''
    x, y = zip(*[d['position'] for _, d in g.nodes(data=True)])
    viewport.scatter(x, y, c=node_color, zorder=zorder+1)

    lines = [(g.node[u]['position'], g.node[v]['position'])
                                                for u, v in g.edges() if u != v]
    artist = LineCollection(lines, colors=edge_color, zorder=zorder)
    viewport.add_collection(artist)

def drawPolicy(viewport, solution, color='black', alpha_min=1.0, zorder=2):
    '''Draws the solution path with a fading effect.'''
    if alpha_min == 1.0:
        transparency = it.repeat(1.0)
    else:
        transparency = np.linspace(alpha_min, 1.0, len(solution)-1)

    for u, v, a in it.izip(solution, solution[1:], transparency):
        dx, dy = v.x - u.x, v.y - u.y
        plt.arrow(u.x, u.y, dx, dy, hold=True, color=color, alpha=a,
                  length_includes_head=True, head_width=0.08, zorder=zorder)

def show_environment(ts, save=None, figsize=None):
    '''Draws the environment and optionally save it to a file.'''
    fig = plt.figure(figsize=figsize)
    viewport = fig.add_subplot(111, aspect='equal')

    for u, d in ts.g.nodes(data=True):
        center = drawRegion(viewport, shape=d['shape'],
                            style={'facecolor': d['color']}, text=u)
        if 'position' not in d:
            d['position'] = (center[0], center[1] - 0.5)

    drawGraph(viewport, ts.g)

    if save is not None:
        plt.subplots_adjust(left=0.05, bottom=0.05, right=0.98, top=0.98,
                            wspace=0, hspace=0)
        plt.savefig(save, dpi=fig.dpi)

    plt.show()
