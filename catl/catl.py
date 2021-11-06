'''
 Copyright (C) 2018-2020 Cristian Ioan Vasile <cvasile@lehigh.edu>
 Explainable Robotics Lab (ERL), Autonomous and Intelligent Robotics (AIR) Lab,
 Lehigh University
 Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Lab, Boston University
 See license.txt file for license information.
'''

import itertools as it
from collections import namedtuple

from antlr4 import InputStream, CommonTokenStream, TerminalNode

from catlLexer import catlLexer
from catlParser import catlParser
from catlVisitor import catlVisitor


class Operation(object):
    '''CATL operations'''
    NOP, NOT, OR, AND, IMPLIES, UNTIL, EVENT, ALWAYS, PRED, LIMIT, BOOL = \
        range(11)
    opnames = [None, '!', '||', '&&', '=>', 'U', 'F', 'G', 'T', 'L', 'bool']
    opcodes = {'!': NOT, '&&': AND, '||' : OR, '=>': IMPLIES,
               'U': UNTIL, 'F': EVENT, 'G': ALWAYS, 'T': PRED, 'L': LIMIT}

    @classmethod
    def getCode(cls, text):
        ''' Gets the code corresponding to the string representation.'''
        return cls.opcodes.get(text, cls.NOP)

    @classmethod
    def getString(cls, op):
        '''Gets custom string representation for each operation.'''
        return cls.opnames[op]

CapabilityRequest = namedtuple('CapabilityRequest', ['capability', 'count'])
ResourceRequest = namedtuple('ResourceRequest', ['resource', 'quantity'])

class CATLFormula(object):
    '''Abstract Syntax Tree representation of an CATL formula'''

    def __init__(self, operation, **kwargs):
        '''Constructor'''
        self.op = operation

        if self.op == Operation.BOOL:
            self.value = kwargs['value']
        elif self.op == Operation.PRED:
            self.duration = kwargs['duration']
            self.proposition = kwargs['proposition']
            self.capability_requests = kwargs['capabilities']
            self.resource_requests = kwargs['resources']
        elif self.op == Operation.LIMIT:
            self.proposition = kwargs['proposition']
            self.capability_requests = kwargs['capabilities']
            self.resource_requests = kwargs['resources']
        elif self.op in (Operation.AND, Operation.OR):
            self.children = kwargs['children']
        elif self.op == Operation.IMPLIES:
            self.left = kwargs['left']
            self.right = kwargs['right']
        elif self.op == Operation.NOT:
            self.child = kwargs['child']
        elif self.op in(Operation.ALWAYS, Operation.EVENT):
            self.low = kwargs['low']
            self.high = kwargs['high']
            self.child = kwargs['child']
        elif self.op == Operation.UNTIL:
            self.low = kwargs['low']
            self.high = kwargs['high']
            self.left = kwargs['left']
            self.right = kwargs['right']

        self.__string = None
        self.__hash = None

    def robustness(self, s, t):
        '''Computes the robustness of the CATL formula.'''
        raise NotImplementedError

    def bound(self):
        '''Computes the bound of the CATL formula.'''
        if self.op == Operation.BOOL:
            return 0
        elif self.op == Operation.PRED:
            return self.duration
        elif self.op == Operation.LIMIT:
            return 0
        elif self.op in (Operation.AND, Operation.OR):
            return max([ch.bound() for ch in self.children])
        elif self.op == Operation.IMPLIES:
            return max(self.left.bound(), self.right.bound())
        elif self.op == Operation.NOT:
            return self.child.bound()
        elif self.op == Operation.UNTIL:
            return self.high + max(self.left.bound(), self.right.bound())
        elif self.op in (Operation.ALWAYS, Operation.EVENT):
            return self.high + self.child.bound()

    def propositions(self):
        '''Computes the set of propositions involved in the CATL formula.'''
        if self.op in (Operation.PRED, Operation.LIMIT):
            return {self.proposition}
        elif self.op in (Operation.AND, Operation.OR):
            return set.union(*[child.propositions() for child in self.children])
        elif self.op in (Operation.IMPLIES, Operation.UNTIL):
            return self.left.propositions() | self.right.propositions()
        elif self.op in (Operation.NOT, Operation.ALWAYS, Operation.EVENT):
            return self.child.propositions()

    def capabilities(self):
        '''Computes the set of capabilities involved in the CATL formula.'''
        if self.op in (Operation.PRED, Operation.LIMIT):
            return {cr.capability for cr in self.capability_requests}
        elif self.op in (Operation.AND, Operation.OR):
            return set.union(*[child.capabilities() for child in self.children])
        elif self.op in (Operation.IMPLIES, Operation.UNTIL):
            return self.left.capabilities() | self.right.capabilities()
        elif self.op in (Operation.NOT, Operation.ALWAYS, Operation.EVENT):
            return self.child.capabilities()

    def resources(self):
        '''Computes the set of resources involved in the CATL formula.'''
        if self.op in (Operation.PRED, Operation.LIMIT):
            return {cr.resource for cr in self.resource_requests}
        elif self.op in (Operation.AND, Operation.OR):
            return set.union(*[child.resources() for child in self.children])
        elif self.op in (Operation.IMPLIES, Operation.UNTIL):
            return self.left.resources() | self.right.resources()
        elif self.op in (Operation.NOT, Operation.ALWAYS, Operation.EVENT):
            return self.child.resources()

    def identifier(self):
        '''Computes an integer identifier for the formula based on the object's
        hash value.
        '''
        h = hash(self)
        if h < 0:
            h = hex(ord('-'))[2:] + hex(-h)[1:]
        else:
            h = hex(ord('+'))[2:] + hex(h)[1:]
        return h

    def __hash__(self):
        if self.__hash is None:
            self.__hash = hash(str(self))
        return self.__hash

    def __eq__(self, other):
        return str(self) == str(other)

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        if self.__string is not None:
            return self.__string

        opname = Operation.getString(self.op)
        if self.op == Operation.BOOL:
            s = str(self.value)
        elif self.op == Operation.PRED:
            s = '({p} {d} {caps} {res})'.format(p=self.proposition,
                                                d=self.duration,
                                                caps=self.capability_requests,
                                                res=self.resource_requests)
        elif self.op == Operation.LIMIT:
            s = '(! {p} {caps} {res})'.format(p=self.proposition,
                                                caps=self.capability_requests,
                                                res=self.resource_requests)
        elif self.op == Operation.IMPLIES:
            s = '({left} {op} {right})'.format(left=self.left, op=opname,
                                               right=self.right)
        elif self.op in (Operation.AND, Operation.OR):
            children = [str(child) for child in self.children]
            s = '(' + ' {op} '.format(op=opname).join(children) + ')'
        elif self.op == Operation.NOT:
            s = '{op} {child}'.format(op=opname, child=self.child)
        elif self.op == Operation.UNTIL:
            s = '({left} {op}[{low}, {high}] {right})'.format(op=opname,
                 left=self.left, right=self.right, low=self.low, high=self.high)
        elif self.op in (Operation.ALWAYS, Operation.EVENT):
            s = '({op}[{low}, {high}] {child})'.format(op=opname,
                                 low=self.low, high=self.high, child=self.child)
        else:
            raise ValueError('Unknown operation {}!'.format(self.op))
        self.__string = s
        return self.__string

    @classmethod
    def from_formula(cls, formula, binary_ops=False):
        '''Creates a CATLFormula object from a formula string.

        Parameters
        ----------
        formula (str) CATL formula
        binary_ops (bool) if set to true, treats AND/OR as binary operators

        Returns
        -------
        (CATLFormula) an AST of the CATL formula
        '''
        lexer = catlLexer(InputStream(formula))
        tokens = CommonTokenStream(lexer)
        parser = catlParser(tokens)
        t = parser.catlProperty()
        return CATLAbstractSyntaxTreeExtractor(binary_ops).visit(t)


class CATLAbstractSyntaxTreeExtractor(catlVisitor):
    '''Parse Tree visitor that constructs the AST of an CATL formula'''

    def __init__(self, binary_ops=False):
        '''Initialize a CATLAbstractSyntaxTreeExtractor

        Parameters
        ----------
        binary_ops (bool) if set to true, AND/OR operations will
                          binary instead of n-ary. Default=False

        Returns
        -------
        CATLAbstractSyntaxTreeExtractor
        '''
        super(CATLAbstractSyntaxTreeExtractor, self).__init__()
        self._binary_ops = binary_ops

    def visitFormula(self, ctx):
        op = Operation.getCode(ctx.op.text)
        ret = None
        low = -1
        high = -1
        if op in (Operation.AND, Operation.OR):
            left = self.visit(ctx.left)
            right = self.visit(ctx.right)
            if self._binary_ops:
                children = [left, right]
            else:
                assert op != right.op
                if left.op == op:
                    children = left.children
                else:
                    children = [left]
                children.append(right)
            ret = CATLFormula(op, children=children)
        elif op == Operation.IMPLIES:
            ret = CATLFormula(op, left=self.visit(ctx.left),
                             right=self.visit(ctx.right))
        elif op == Operation.NOT:
            ret = CATLFormula(op, child=self.visit(ctx.child))
        elif op == Operation.UNTIL:
            low = float(ctx.low.text)
            high = float(ctx.high.text)
            ret = CATLFormula(op, left=self.visit(ctx.left),
                             right=self.visit(ctx.right), low=low, high=high)
        elif op in (Operation.ALWAYS, Operation.EVENT):
            low = float(ctx.low.text)
            high = float(ctx.high.text)
            ret = CATLFormula(op, child=self.visit(ctx.child),
                             low=low, high=high)
        else:
            print('Error: unknown operation!')
        return ret

    def visitCatlPredicate(self, ctx):
        return self.visit(ctx.predicate())

    def visitPredicate(self, ctx):
        if Operation.getCode(ctx.op.text) == Operation.PRED:
            if ctx.resources():
                resources = self.visit(ctx.resources())
            else:
                resources = set()
            return CATLFormula(Operation.PRED, duration=int(ctx.duration.text),
                               proposition=ctx.proposition.text,
                               capabilities=self.visit(ctx.capabilities()),
                               resources=resources)
        return CATLFormula(Operation.BOOL, value=bool(ctx.op.text))

    def visitCatlLimit(self, ctx):
        return self.visit(ctx.limit())

    def visitLimit(self, ctx):
        if Operation.getCode(ctx.op.text) == Operation.LIMIT:
            if ctx.capabilities():
                capabilities = self.visit(ctx.capabilities())
            else:
                capabilities = set()
            if ctx.resources():
                resources = self.visit(ctx.resources())
            else:
                resources = set()
            if not capabilities and not resources:
                raise Exception('Limit operator must constrain either agents'
                                ' or resources!')
            return CATLFormula(Operation.LIMIT,
                               proposition=ctx.proposition.text,
                               capabilities=capabilities,
                               resources=resources)
        return CATLFormula(Operation.BOOL, value=bool(ctx.op.text))

    def visitCapabilities(self, ctx):
        return {self.visit(ch) for ch in ctx.children
                                            if not isinstance(ch, TerminalNode)}

    def visitCapabilityRequest(self, ctx):
        return CapabilityRequest(capability=ctx.cap.text,
                                 count=int(ctx.count.text))

    def visitResources(self, ctx):
        return {self.visit(ch) for ch in ctx.children
                                            if not isinstance(ch, TerminalNode)}

    def visitResourceRequest(self, ctx):
        return ResourceRequest(resource=ctx.res.text,
                               quantity=float(ctx.quantity.text))

    def visitParprop(self, ctx):
        return self.visit(ctx.child);


if __name__ == '__main__':
    ast = CATLFormula.from_formula(
            'F[0, 2] T(4, test, {(a, 2), (b, 3)})'
            '&& G[1, 7] T(2, test, {(a, 1), (c, 4)}, {(h1, 2.3), (h2, 5)})'
            '&& F[3, 5] T(3, test2, {(b, 1), (d, 2)})'
            '&& F[3, 5] L(test2, {(b, 1), (d, 2)})'
            '&& F[3, 5] L(test2, {}, {(h1, 2.3), (h2, 5)})'
            '&& F[3, 5] L(test2, {(b, 1), (d, 2)}, {(h1, 2.3), (h2, 5)})')
    print('AST:', str(ast))
    print('Propositions:', ast.propositions())
    print('Capabilities:', ast.capabilities())
    print('Resources:', ast.resources())
    print('Bound:', ast.bound())
