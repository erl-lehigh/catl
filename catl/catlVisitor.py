# Generated from catl.g4 by ANTLR 4.8
from antlr4 import *
if __name__ is not None and "." in __name__:
    from .catlParser import catlParser
else:
    from catlParser import catlParser

'''
 Copyright (C) 2018-2020 Cristian Ioan Vasile <cvasile@lehigh.edu>
 Explainable Robotics Lab (ERL), Autonomous and Intelligent Robotics (AIR) Lab,
 Lehigh University
 Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Lab, Boston University
 See license.txt file for license information.
'''


# This class defines a complete generic visitor for a parse tree produced by catlParser.

class catlVisitor(ParseTreeVisitor):

    # Visit a parse tree produced by catlParser#catlPredicate.
    def visitCatlPredicate(self, ctx:catlParser.CatlPredicateContext):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by catlParser#catlLimit.
    def visitCatlLimit(self, ctx:catlParser.CatlLimitContext):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by catlParser#formula.
    def visitFormula(self, ctx:catlParser.FormulaContext):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by catlParser#parprop.
    def visitParprop(self, ctx:catlParser.ParpropContext):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by catlParser#predicate.
    def visitPredicate(self, ctx:catlParser.PredicateContext):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by catlParser#limit.
    def visitLimit(self, ctx:catlParser.LimitContext):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by catlParser#capabilities.
    def visitCapabilities(self, ctx:catlParser.CapabilitiesContext):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by catlParser#capabilityRequest.
    def visitCapabilityRequest(self, ctx:catlParser.CapabilityRequestContext):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by catlParser#resources.
    def visitResources(self, ctx:catlParser.ResourcesContext):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by catlParser#resourceRequest.
    def visitResourceRequest(self, ctx:catlParser.ResourceRequestContext):
        return self.visitChildren(ctx)



del catlParser