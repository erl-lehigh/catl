'''
 Copyright (c) 2023, Explainable Robotics Lab (ERL)
 See license.txt file for license information.
 @author: Gustavo A. Cardona, Cristian-Ioan Vasile
'''

from antlr4 import InputStream, CommonTokenStream

from catl import Operation as CATLOperation
from catl import CATLFormula
from catl import CATLAbstractSyntaxTreeExtractor

from catlLexer import catlLexer
from catlParser import catlParser

from mtl import Operation as MTLOperation
from mtl import MTLFormula


def catl2mtl(catl_ast):
    '''Translates a CATL abstract syntax tree to an MTL one.
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
        return MTLFormula(MTLOperation.BOOL, value=catl_ast.value)
    elif catl_ast.op == CATLOperation.PRED:
        var = catl_ast.proposition + '_{cap}'
        capability_terms = \
            [MTLFormula(MTLOperation.PRED, variable=var.format(cap=cap))
                               for cap, _ in catl_ast.capability_requests]
        child = MTLFormula(MTLOperation.AND, children=capability_terms)
        cap_available = MTLFormula(MTLOperation.ALWAYS, low=0,
                                   high=catl_ast.duration, child=child)

        var = catl_ast.proposition + '_{res}'
        resource_terms = \
            [MTLFormula(MTLOperation.PRED, variable=var.format(res=res))
                               for res, _ in catl_ast.resource_requests]
        mtl_ast = MTLFormula(MTLOperation.AND,
                             children=[cap_available] + resource_terms)
        mtl_ast.task = catl_ast
        return mtl_ast
    elif catl_ast.op in (CATLOperation.AND, CATLOperation.OR):
        children = [catl2mtl(ch) for ch in catl_ast.children]
        if catl_ast.op == CATLOperation.AND:
            op = MTLOperation.AND
        else:
            op = MTLOperation.OR
        return MTLFormula(op, children=children)
    elif catl_ast.op == CATLOperation.IMPLIES:
        left = catl2mtl(catl_ast.left)
        right = catl2mtl(catl_ast.right)
        return MTLFormula(MTLOperation.IMPLIES, left=left, right=right)
    elif catl_ast.op == CATLOperation.NOT:
        child = catl2mtl(catl_ast.child)
        return MTLFormula(MTLOperation.NOT, child=child)
    elif catl_ast.op in (CATLOperation.ALWAYS, CATLOperation.EVENT):
        child = catl2mtl(catl_ast.child)
        if catl_ast.op == CATLOperation.ALWAYS:
            op = MTLOperation.ALWAYS
        else:
            op = MTLOperation.EVENT
        return MTLFormula(op, child=child, low=catl_ast.low, high=catl_ast.high)
    elif catl_ast.op == CATLOperation.UNTIL:
        left = catl2mtl(catl_ast.left)
        right = catl2mtl(catl_ast.right)
        return MTLFormula(MTLOperation.UNTIL, left=left, right=right,
                          low=catl_ast.low, high=catl_ast.high)


def extract_mtl_task_formulae(mtl_ast):
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
    stack = [mtl_ast]
    while stack:
        mtl_ast = stack.pop(0)
        assert mtl_ast.op != MTLOperation.PRED

        if mtl_ast.op == MTLOperation.AND:
            if hasattr(mtl_ast, 'task'):
                tasks.append((mtl_ast, mtl_ast.task))
            else:
                stack.extend(mtl_ast.children)
        elif mtl_ast.op == MTLOperation.OR:
            stack.extend(mtl_ast.children)
        elif mtl_ast.op in (MTLOperation.ALWAYS, MTLOperation.EVENT,
                            MTLOperation.NOT):
            stack.append(mtl_ast.child)
        elif mtl_ast.op in (MTLOperation.UNTIL, MTLOperation.IMPLIES):
            stack.append(mtl_ast.left)
            stack.append(mtl_ast.right)
    return tasks


def mtl_predicate_variables(catl_ast):
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
    # print('CATL:', ast)

    stl = catl2mtl(ast)
    # print('STL:', stl)

    cap_pred, res_pred = mtl_predicate_variables(ast)


    print('HEREEEEEEE', cap_pred, res_pred)

    stl_tasks = extract_mtl_task_formulae(stl)
    # for stl_formula, task in stl_tasks:
        # print('Task:', task, 'STL formula:', stl_formula)
