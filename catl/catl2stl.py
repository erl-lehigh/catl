'''
 Copyright (C) 2018-2020 Cristian Ioan Vasile <cvasile@lehigh.edu>
 Explainable Robotics Lab (ERL), Autonomous and Intelligent Robotics (AIR) Lab,
 Lehigh University
 Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Lab, Boston University
 See license.txt file for license information.
'''

from antlr4 import InputStream, CommonTokenStream

from catl import Operation as CATLOperation
from catl import CATLFormula
from catl import CATLAbstractSyntaxTreeExtractor

from catlLexer import catlLexer
from catlParser import catlParser

from stl import Operation as STLOperation
from stl import RelOperation as STLRelOperation
from stl import STLFormula


def catl2stl(catl_ast):
    '''Translates a CATL abstract syntax tree to an STL one.
    The set of variables in the STL formula is the Cartesian product between the
    set of atomic propositions and the set of capabilities in the CATL formula.
    The variable labels are in the form "z_{proposition}_{capability}".
    The STL predicates capture the minimum number ``n''of agents with capability
    ``c'' needed at regions labeled with proposition ``p'', i.e.,
    ``z_{\pi,c} \geq n''. Thus, a CATL task is translated to STL as follows

    .. math ::

        T(d, \pi, \{(c_1, n_1), \ldots,(c_m, n_m)\}) \equiv
        \box_{[0, d]} \bigcup_{i=1}^{m} (z_{\pi, c_i} \geq n_i)
    '''
    if catl_ast.op == CATLOperation.BOOL:
        return STLFormula(STLOperation.BOOL, value=catl_ast.value)
    elif catl_ast.op == CATLOperation.PRED:
        var = catl_ast.proposition + '_{cap}'
        conjunction_terms = \
            [STLFormula(STLOperation.PRED, relation=STLRelOperation.GE,
                        variable=var.format(cap=cap), threshold=th)
                               for cap, th in catl_ast.capability_requests]
        child = STLFormula(STLOperation.AND, children=conjunction_terms)

        return STLFormula(STLOperation.ALWAYS, low=0, high=catl_ast.duration,
                          child=child)
    elif catl_ast.op in (CATLOperation.AND, CATLOperation.OR):
        children = [catl2stl(ch) for ch in catl_ast.children]
        if catl_ast.op == CATLOperation.AND:
            op = STLOperation.AND
        else:
            op = STLOperation.OR
        return STLFormula(op, children=children)
    elif catl_ast.op == CATLOperation.IMPLIES:
        left = catl2stl(catl_ast.left)
        right = catl2stl(catl_ast.right)
        return STLFormula(STLOperation.IMPLIES, left=left, right=right)
    elif catl_ast.op == CATLOperation.NOT:
        child = catl2stl(catl_ast.child)
        return STLFormula(STLOperation.NOT, child=child)
    elif catl_ast.op in (CATLOperation.ALWAYS, CATLOperation.EVENT):
        child = catl2stl(catl_ast.child)
        if catl_ast.op == CATLOperation.ALWAYS:
            op = STLOperation.ALWAYS
        else:
            op = STLOperation.EVENT
        return STLFormula(op, child=child, low=catl_ast.low, high=catl_ast.high)
    elif catl_ast.op == CATLOperation.UNTIL:
        left = catl2stl(catl_ast.left)
        right = catl2stl(catl_ast.right)
        return STLFormula(STLOperation.UNTIL, left=left, right=right,
                          low=catl_ast.low, high=catl_ast.high)


if __name__ == '__main__':
    lexer = catlLexer(InputStream('F[0, 2] T(4, test, {(a, 2), (b, 3)})'
                                  '&& G[1, 7] T(2, test, {(a, 1), (c, 4)})'
                                  '&& F[3, 5] T(3, test2, {(b, 1), (d, 2)})'))

    tokens = CommonTokenStream(lexer)

    parser = catlParser(tokens)
    t = parser.catlProperty()
    print(t.toStringTree())

    ast = CATLAbstractSyntaxTreeExtractor().visit(t)
    print('CATL:', ast)

    stl = catl2stl(ast)
    print('STL:', stl)
