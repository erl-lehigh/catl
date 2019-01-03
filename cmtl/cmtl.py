'''
 Copyright (C) 2018 Cristian Ioan Vasile <cvasile@bu.edu>
 Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Lab, Boston University
 See license.txt file for license information.
'''

import itertools as it
from collections import namedtuple

from antlr4 import InputStream, CommonTokenStream, TerminalNode

from cmtlLexer import cmtlLexer
from cmtlParser import cmtlParser
from cmtlVisitor import cmtlVisitor


class Operation(object):
    '''CMTL operations'''
    NOP, NOT, OR, AND, IMPLIES, UNTIL, EVENT, ALWAYS, PRED, BOOL = range(10)
    opnames = [None, '!', '||', '&&', '=>', 'U', 'F', 'G', 'T', 'bool']
    opcodes = {'!': NOT, '&&': AND, '||' : OR, '=>': IMPLIES,
               'U': UNTIL, 'F': EVENT, 'G': ALWAYS, 'T': PRED}

    @classmethod
    def getCode(cls, text):
        ''' Gets the code corresponding to the string representation.'''
        return cls.opcodes.get(text, cls.NOP)

    @classmethod
    def getString(cls, op):
        '''Gets custom string representation for each operation.'''
        return cls.opnames[op]

CapabilityRequest = namedtuple('CapabilityRequest', ['capability', 'count'])

class CMTLFormula(object):
    '''Abstract Syntax Tree representation of an CMTL formula'''

    def __init__(self, operation, **kwargs):
        '''Constructor'''
        self.op = operation

        if self.op == Operation.BOOL:
            self.value = kwargs['value']
        elif self.op == Operation.PRED:
            self.duration = kwargs['duration']
            self.proposition = kwargs['proposition']
            self.capability_requests = kwargs['capabilities']
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
        '''Computes the robustness of the CMTL formula.'''
        raise NotImplementedError

    def bound(self):
        '''Computes the bound of the CMTL formula.'''
        if self.op == Operation.BOOL:
            return 0
        elif self.op == Operation.PRED:
            return self.duration
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
        '''Computes the set of propositions involved in the CMTL formula.'''
        if self.op == Operation.PRED:
            return {self.proposition}
        elif self.op in (Operation.AND, Operation.OR):
            return set.union(*[child.propositions() for child in self.children])
        elif self.op in (Operation.IMPLIES, Operation.UNTIL):
            return self.left.propositions() | self.right.propositions()
        elif self.op in (Operation.NOT, Operation.ALWAYS, Operation.EVENT):
            return self.child.propositions()

    def capabilities(self):
        '''Computes the set of capabilities involved in the CMTL formula.'''
        if self.op == Operation.PRED:
            return {cr.capability for cr in self.capability_requests}
        elif self.op in (Operation.AND, Operation.OR):
            return set.union(*[child.capabilities() for child in self.children])
        elif self.op in (Operation.IMPLIES, Operation.UNTIL):
            return self.left.capabilities() | self.right.capabilities()
        elif self.op in (Operation.NOT, Operation.ALWAYS, Operation.EVENT):
            return self.child.capabilities()

    def identifier(self):
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
            s = '({p} {d} {caps})'.format(p=self.proposition, d=self.duration,
                                          caps=self.capability_requests)
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
    def from_formula(cls, formula):
        '''TODO:
        '''
        lexer = cmtlLexer(InputStream(formula))
        tokens = CommonTokenStream(lexer)
        parser = cmtlParser(tokens)
        t = parser.cmtlProperty()
        return CMTLAbstractSyntaxTreeExtractor().visit(t)


class CMTLAbstractSyntaxTreeExtractor(cmtlVisitor):
    '''Parse Tree visitor that constructs the AST of an CMTL formula'''

    def visitFormula(self, ctx):
        op = Operation.getCode(ctx.op.text)
#         print ctx.op.text, op
        ret = None
        low = -1
        high = -1
        if op in (Operation.AND, Operation.OR):
            left = self.visit(ctx.left)
            right = self.visit(ctx.right)
            assert op != right.op
            if left.op == op:
                children = left.children
            else:
                children = [left]
            children.append(right)
            ret = CMTLFormula(op, children=children)
        elif op == Operation.IMPLIES:
            ret = CMTLFormula(op, left=self.visit(ctx.left),
                             right=self.visit(ctx.right))
        elif op == Operation.NOT:
            ret = CMTLFormula(op, child=self.visit(ctx.child))
        elif op == Operation.UNTIL:
            low = float(ctx.low.text)
            high = float(ctx.high.text)
            ret = CMTLFormula(op, left=self.visit(ctx.left),
                             right=self.visit(ctx.right), low=low, high=high)
        elif op in (Operation.ALWAYS, Operation.EVENT):
#             print 'EVENT/ALWAYS:', ctx.low.text, ctx.high.text
            low = float(ctx.low.text)
            high = float(ctx.high.text)
            ret = CMTLFormula(op, child=self.visit(ctx.child),
                             low=low, high=high)
        else:
            print('Error: unknown operation!')
        return ret

    def visitCmtlPredicate(self, ctx):
        return self.visit(ctx.predicate())

    def visitPredicate(self, ctx):
        if Operation.getCode(ctx.op.text) == Operation.PRED:
            return CMTLFormula(Operation.PRED, duration=int(ctx.duration.text),
                               proposition=ctx.proposition.text,
                               capabilities=self.visit(ctx.capabilities()))
        return CMTLFormula(Operation.BOOL, value=bool(ctx.op.text))

    def visitCapabilities(self, ctx):
        return {self.visit(ch) for ch in ctx.children
                                            if not isinstance(ch, TerminalNode)}

    def visitCapabilityRequest(self, ctx):
#         print CapabilityRequest(capability=ctx.cap.text,
#                                 count=int(ctx.count.text))
        return CapabilityRequest(capability=ctx.cap.text,
                                 count=int(ctx.count.text))

    def visitParprop(self, ctx):
        return self.visit(ctx.child);


if __name__ == '__main__':
    ast = CMTLFormula.from_formula('F[0, 2] T(4, test, {(a, 2), (b, 3)})'
                                  '&& G[1, 7] T(2, test, {(a, 1), (c, 4)})'
                                  '&& F[3, 5] T(3, test2, {(b, 1), (d, 2)})')
    print 'AST:', ast
    print 'Propositions:', ast.propositions()
    print 'Capabilities:', ast.capabilities()
    print 'Bound:', ast.bound()

#     s = () #TODO:
#     print 'r:', ast.robustness(s, 0)
