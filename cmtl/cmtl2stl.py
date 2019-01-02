'''
 Copyright (C) 2018 Cristian Ioan Vasile <cvasile@bu.edu>
 Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Lab, Boston University
 See license.txt file for license information.
'''

from antlr4 import InputStream, CommonTokenStream

from cmtl import Operation as CMTLOperation
from cmtl import CMTLFormula
from cmtl import CMTLAbstractSyntaxTreeExtractor

from cmtlLexer import cmtlLexer
from cmtlParser import cmtlParser

from stl import Operation as STLOperation
from stl import RelOperation as STLRelOperation
from stl import STLFormula


def cmtl2stl(cmtl_ast, ap_regions=None):
    '''Translates a CMTL abstract syntax tree to an STL one.
    The set of variables in the STL formula is the Cartesian product between
    the set of labeled regions and the set of capabilities in the CMTL formula.
    The variable labels are in the form "{region}_{capability}".
    The STL predicates capture the minimum number ``n''of agents with capability
    ``c'' needed at region ``r'' labeled with proposition ``p'', i.e.,
    ``r\_c \geq n''. Thus, a CMTL task is translated to STL as follows

    .. math ::

        T(d, \pi, \{(c_1, n_1), \ldots,(c_m, n_m)\}) \equiv
        \box_{[0, d]} \bigvee_{r : \pi \in \mathcal{L}(r)} \bigcup_{i=1}^{m}
         (r_{c_i} \geq n_i)

    NOTE: If the label to regions map is not provided then the encoding assumes
    that regions are uniquely identified by their propositions (labels). Thus,
    the propositions are used as region labels.
    '''
    if ap_regions is None:
        props = cmtl_ast.propositions()
        ap_regions = {p: (p,) for p in props}
        print ap_regions

    if cmtl_ast.op == CMTLOperation.BOOL:
        return STLFormula(STLOperation.BOOL, value=cmtl_ast.value)
    elif cmtl_ast.op == CMTLOperation.PRED:
        disjunction_terms = []
        for region in ap_regions[cmtl_ast.proposition]:
            var = region + '_{cap}'
            conjunction_terms = \
                [STLFormula(STLOperation.PRED, relation=STLRelOperation.GE,
                            variable=var.format(cap=cap), threshold=th)
                                   for cap, th in cmtl_ast.capability_requests]
            child = STLFormula(STLOperation.AND, children=conjunction_terms)
            disjunction_terms.append(child)

        if len(disjunction_terms) == 1:
            child = disjunction_terms[0]
        else:
            child = STLFormula(STLOperation.OR, children=disjunction_terms)
        return STLFormula(STLOperation.ALWAYS, low=0, high=cmtl_ast.duration,
                          child=child)
    elif cmtl_ast.op in (CMTLOperation.AND, CMTLOperation.OR):
        children = [cmtl2stl(ch, ap_regions) for ch in cmtl_ast.children]
        if cmtl_ast.op == CMTLOperation.AND:
            op = STLOperation.AND
        else:
            op = STLOperation.OR
        return STLFormula(op, children=children)
    elif cmtl_ast.op == CMTLOperation.IMPLIES:
        left = cmtl2stl(cmtl_ast.left, ap_regions)
        right = cmtl2stl(cmtl_ast.right, ap_regions)
        return STLFormula(STLOperation.IMPLIES, left=left, right=right)
    elif cmtl_ast.op == CMTLOperation.NOT:
        child = cmtl2stl(cmtl_ast.child, ap_regions)
        return STLFormula(STLOperation.NOT, child=child)
    elif cmtl_ast.op in (CMTLOperation.ALWAYS, CMTLOperation.EVENT):
        child = cmtl2stl(cmtl_ast.child, ap_regions)
        if cmtl_ast.op == CMTLOperation.ALWAYS:
            op = STLOperation.ALWAYS
        else:
            op = STLOperation.EVENT
        return STLFormula(op, child=child, low=cmtl_ast.low, high=cmtl_ast.high)
    elif cmtl_ast.op == CMTLOperation.UNTIL:
        left = cmtl2stl(cmtl_ast.left, ap_regions)
        right = cmtl2stl(cmtl_ast.right, ap_regions)
        return STLFormula(STLOperation.UNTIL, left=left, right=right,
                          low=cmtl_ast.low, high=cmtl_ast.high)


if __name__ == '__main__':
    lexer = cmtlLexer(InputStream('F[0, 2] T(4, test, {(a, 2), (b, 3)})'
                                  '&& G[1, 7] T(2, test, {(a, 1), (c, 4)})'
                                  '&& F[3, 5] T(3, test2, {(b, 1), (d, 2)})'))

    tokens = CommonTokenStream(lexer)

    parser = cmtlParser(tokens)
    t = parser.cmtlProperty()
    print t.toStringTree()

    ast = CMTLAbstractSyntaxTreeExtractor().visit(t)
    print 'CMTL:', ast

    stl = cmtl2stl(ast)
    print 'STL:', stl
