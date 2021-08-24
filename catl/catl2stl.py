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
        \bigcup \bigcup_{\ell=1}^{r} (z_{\pi, h_\ell} \geq \Delta_\ell)
    '''
    if catl_ast.op == CATLOperation.BOOL:
        return STLFormula(STLOperation.BOOL, value=catl_ast.value)
    elif catl_ast.op == CATLOperation.PRED:
        var = catl_ast.proposition + '_{cap}'
        capability_terms = \
            [STLFormula(STLOperation.PRED, relation=STLRelOperation.GE,
                        variable=var.format(cap=cap), threshold=th)
                               for cap, th in catl_ast.capability_requests]
        child = STLFormula(STLOperation.AND, children=capability_terms)
        cap_available = STLFormula(STLOperation.ALWAYS, low=0,
                                   high=catl_ast.duration, child=child)

        var = catl_ast.proposition + '_{res}'
        resource_terms = \
            [STLFormula(STLOperation.PRED, relation=STLRelOperation.GE,
                        variable=var.format(res=res), threshold=th)
                               for res, th in catl_ast.resource_requests]

        stl_ast = STLFormula(STLOperation.AND,
                             children=[cap_available] + resource_terms)
        stl_ast.task = catl_ast
        return stl_ast
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


def extract_stl_task_formulae(stl_ast):
    '''Extract tasks from STL abstract syntax trees obtained from CATL formulae.

    Input
    -----
    stl_ast : STLFormula
        STL abstract syntax tree.

    Output
    ------
    tasks : list of pairs of STLFormula and CATLFormula objects
        A list of pairs of abstract syntax trees in STL and CATL associated with
        each task.
    '''
    tasks = []
    stack = [stl_ast]
    while stack:
        stl_ast = stack.pop(0)
        assert stl_ast.op != STLOperation.PRED

        if stl_ast.op == STLOperation.AND:
            if hasattr(stl_ast, 'task'):
                tasks.append((stl_ast, stl_ast.task))
            stack.extend(stl_ast.children)
        elif stl_ast.op == STLOperation.OR:
            stack.extend(stl_ast.children)
        elif stl_ast.op in (STLOperation.ALWAYS, STLOperation.EVENT,
                            STLOperation.NOT):
            stack.append(stl_ast.child)
        elif stl_ast.op in (STLOperation.UNTIL, STLOperation.IMPLIES):
            stack.append(stl_ast.left)
            stack.append(stl_ast.right)
    return tasks


def stl_predicate_variables(catl_ast):
    '''Returns the sets of agent and resource predicate variables for a CaTL
    formula.

    TODO:
    '''
    stack = [catl_ast]
    capability_pred_vars = dict()
    for capability in catl_ast.capabilities():
        capability_pred_vars[capability] = set()
    resource_pred_vars = dict()
    for resource in catl_ast.resources():
        resource_pred_vars[resource] = set()
    while stack:
        formula = stack.pop()
        if formula.op == CATLOperation.PRED:
            var = formula.proposition + '_{cap}'
            for cr in formula.capability_requests:
                capability_pred_vars[cr.capability].add(
                                                var.format(cap=cr.capability))
            var = formula.proposition + '_{res}'
            for h in formula.resource_requests:
                resource_pred_vars[h.resource].add(var.format(res=h.resource))
        elif formula.op in (CATLOperation.AND, CATLOperation.OR):
            stack.extend(formula.children)
        elif formula.op in (CATLOperation.IMPLIES, CATLOperation.UNTIL):
            stack.append(formula.left)
            stack.append(formula.right)
        elif formula.op in (CATLOperation.NOT, CATLOperation.ALWAYS,
                         CATLOperation.EVENT):
            stack.append(formula.child)

    return capability_pred_vars, resource_pred_vars


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

    stl_tasks = extract_task_variables(stl)
    for stl_formula, task in stl_tasks:
        print('Task:', task, 'STL formula:', stl_formula)
